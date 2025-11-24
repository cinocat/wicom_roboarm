#!/usr/bin/env python3
import rospy
import time
import sys
from sensor_msgs.msg import Range
from smbus2 import SMBus

# ================== Multiplexer helper ==================
class I2CMux:
    def __init__(self, bus: SMBus, address=0x70, settle=0.001):
        self.bus = bus
        self.address = address
        self.settle = settle
        self._current = None
    def select(self, channel, force=False):
        if not (0 <= channel <= 7):
            raise ValueError("Kênh mux phải 0–7")
        if self._current == channel and not force:
            return
        self.bus.write_byte(self.address, 1 << channel)
        time.sleep(self.settle)
        self._current = channel

# ================== Flexible VL53 import ==================
VL53_CLASS = None
VL53_VARIANT = None

def _try_import():
    global VL53_CLASS, VL53_VARIANT
    # 1. from vl53l1x import VL53L1X
    try:
        from vl53l1x import VL53L1X as C
        VL53_CLASS = C
        VL53_VARIANT = "vl53l1x:direct"
        return
    except Exception:
        pass
    # 2. from vl53l1x.vl53l1x import VL53L1X
    try:
        from vl53l1x.vl53l1x import VL53L1X as C
        VL53_CLASS = C
        VL53_VARIANT = "vl53l1x.vl53l1x:direct"
        return
    except Exception:
        pass
    # 3. import vl53l1x rồi quét
    try:
        import vl53l1x as M
        for attr in dir(M):
            if "L1" in attr or "VL53" in attr:
                A = getattr(M, attr)
                if isinstance(A, type):
                    VL53_CLASS = A
                    VL53_VARIANT = f"vl53l1x:scan({attr})"
                    return
    except Exception:
        pass
    # 4. Thử module tên viết hoa
    try:
        import VL53L1X as M2
        for attr in dir(M2):
            if "L1" in attr or "VL53" in attr:
                A = getattr(M2, attr)
                if isinstance(A, type):
                    VL53_CLASS = A
                    VL53_VARIANT = f"VL53L1X:scan({attr})"
                    return
    except Exception:
        pass
    # 5. Fallback thử L0X
    try:
        import VL53L0X as M3
        for attr in dir(M3):
            if "L0" in attr or "VL53" in attr:
                A = getattr(M3, attr)
                if isinstance(A, type):
                    VL53_CLASS = A
                    VL53_VARIANT = f"VL53L0X:scan({attr})"
                    return
    except Exception:
        pass

_try_import()

# ================== Node implementation ==================
class DualVL53Node:
    def __init__(self):
        rospy.init_node("vl53_dual_node")

        self.bus_num      = rospy.get_param("~i2c_bus", 1)
        self.mux_addr     = rospy.get_param("~mux_address", 0x70)
        self.ch_short     = rospy.get_param("~channel_short", 0)
        self.ch_long      = rospy.get_param("~channel_long", 1)
        self.addr_short   = rospy.get_param("~addr_short", 0x29)
        self.addr_long    = rospy.get_param("~addr_long", 0x29)
        self.rate_hz      = rospy.get_param("~publish_rate_hz", 20.0)
        self.frame_short  = rospy.get_param("~frame_id_short", "vl53_short")
        self.frame_long   = rospy.get_param("~frame_id_long", "vl53_long")

        self.bus = SMBus(self.bus_num)
        self.mux = I2CMux(self.bus, self.mux_addr)

        if VL53_CLASS:
            rospy.loginfo("Dùng driver VL53: %s (class=%s)", VL53_VARIANT, VL53_CLASS.__name__)
        else:
            rospy.logerr("Không tìm thấy lớp driver VL53L1X/L0X. Sẽ chạy mô phỏng (publish NaN).")

        self.sensor_short = self._init_sensor(self.ch_short, self.addr_short, mode="short") if VL53_CLASS else None
        self.sensor_long  = self._init_sensor(self.ch_long,  self.addr_long,  mode="long")  if VL53_CLASS else None

        active = []
        if self.sensor_short: active.append(f"short@ch{self.ch_short}")
        if self.sensor_long:  active.append(f"long@ch{self.ch_long}")
        rospy.loginfo("Sensors active: %s", ", ".join(active) if active else "NONE")

        self.pub_short = rospy.Publisher("/vl53/short_range", Range, queue_size=10)
        self.pub_long  = rospy.Publisher("/vl53/long_range", Range, queue_size=10)

    def _probe(self, addr):
        try:
            self.bus.read_byte(addr)
            return True
        except:
            return False

    def _init_sensor(self, channel, address, mode="short"):
        self.mux.select(channel, force=True)
        if not self._probe(address):
            rospy.logwarn("Không probe được 0x%02X trên kênh %d", address, channel)
            return None
        # Khởi tạo linh hoạt
        try:
            sensor = VL53_CLASS(i2c_bus=self.bus_num, i2c_address=address)
        except TypeError:
            try:
                sensor = VL53_CLASS(i2c_address=address)
            except Exception as e:
                rospy.logwarn("Init sensor lớp=%s kênh %d thất bại: %s", VL53_CLASS.__name__, channel, e)
                return None
        for m in ["open", "start_ranging", "start"]:
            if hasattr(sensor, m):
                try: getattr(sensor, m)()
                except Exception: pass
        # Set distance mode nếu có
        if hasattr(sensor, "set_distance_mode"):
            try:
                sensor.set_distance_mode(1 if mode == "short" else 3)
            except Exception as e:
                rospy.logwarn("set_distance_mode (%s) thất bại kênh %d: %s", mode, channel, e)
        rospy.loginfo("Init %s OK kênh %d", mode, channel)
        return sensor

    def _read_distance(self, sensor, channel):
        if sensor is None:
            return None
        self.mux.select(channel)
        for m in ["get_distance", "read_distance", "distance", "getDistance"]:
            if hasattr(sensor, m):
                try:
                    mm = getattr(sensor, m)()
                    if mm is None or mm <= 0:
                        return None
                    # Nếu driver trả mét (rất hiếm) thì mm < 10 sẽ bị chia nhỏ; ta chấp nhận mm là mm chuẩn.
                    return mm / 1000.0
                except Exception as e:
                    rospy.logwarn_throttle(5.0, f"Lỗi đọc kênh {channel}: {e}")
                    return None
        return None

    def _range_msg(self, frame, min_r, max_r, dist):
        msg = Range()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.52
        msg.min_range = min_r
        msg.max_range = max_r
        msg.range = dist if dist is not None else float('nan')
        return msg

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            d_short = self._read_distance(self.sensor_short, self.ch_short)
            d_long  = self._read_distance(self.sensor_long,  self.ch_long)
            self.pub_short.publish(self._range_msg(self.frame_short, 0.03, 2.0, d_short))
            self.pub_long.publish(self._range_msg(self.frame_long, 0.03, 4.0, d_long))
            rate.sleep()

    def shutdown(self):
        for sensor, ch in [(self.sensor_short, self.ch_short),
                           (self.sensor_long,  self.ch_long)]:
            if sensor:
                self.mux.select(ch)
                for m in ["stop_ranging", "stop", "close"]:
                    if hasattr(sensor, m):
                        try: getattr(sensor, m)()
                        except Exception: pass
        try: self.bus.close()
        except Exception: pass

if __name__ == "__main__":
    node = DualVL53Node()
    try:
        node.spin()
    finally:
        node.shutdown()