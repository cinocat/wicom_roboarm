#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import Range

import board
import busio
import adafruit_tca9548a
import adafruit_vl53l1x
import adafruit_vl53l0x

class DualVL53Node:
    def __init__(self):
        rospy.init_node("vl53_dual_node")

        self.rate_hz     = rospy.get_param("~publish_rate_hz", 20.0)
        # Mapping channel (VL53L0X channel 0 short, VL53L1X channel 1 long)
        self.ch_short    = rospy.get_param("~channel_short", 0)   # VL53L0X
        self.ch_long     = rospy.get_param("~channel_long", 1)    # VL53L1X
        self.frame_short = rospy.get_param("~frame_id_short", "vl53_short")
        self.frame_long  = rospy.get_param("~frame_id_long", "vl53_long")

        rospy.loginfo("Init I2C bus and multiplexer TCA9548A...")
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            # TCA9548A: 0x70
            mux_addr = rospy.get_param("~mux_address", 0x70)
            self.tca = adafruit_tca9548a.TCA9548A(self.i2c, address=mux_addr)
        except Exception as e:
            rospy.logfatal("Init failed I2C/TCA9548A: %s", e)
            raise

        rospy.loginfo("Initializing VL53L1X (long) on channel %d...", self.ch_long)
        self.sensor_long = None
        try:
            self.sensor_long = adafruit_vl53l1x.VL53L1X(self.tca[self.ch_long])
            self.sensor_long.start_ranging()
            rospy.loginfo("VL53L1X initialized.")
        except Exception as e:
            rospy.logwarn("VL53L1X init error channel %d: %s", self.ch_long, e)

        rospy.loginfo("Initializing VL53L0X (short) on channel %d...", self.ch_short)
        self.sensor_short = None
        try:
            self.sensor_short = adafruit_vl53l0x.VL53L0X(self.tca[self.ch_short])
            rospy.loginfo("VL53L0X initialized.")
        except Exception as e:
            rospy.logwarn("VL53L0X init error channel %d: %s", self.ch_short, e)

        active = []
        if self.sensor_short: active.append(f"VL53L0X@ch{self.ch_short}")
        if self.sensor_long:  active.append(f"VL53L1X@ch{self.ch_long}")
        rospy.loginfo("Sensors active: %s", ", ".join(active) if active else "NONE")

        self.pub_short = rospy.Publisher("/vl53/short_range", Range, queue_size=10)
        self.pub_long  = rospy.Publisher("/vl53/long_range", Range, queue_size=10)

    def _range_msg(self, frame, min_r, max_r, dist_m):
        msg = Range()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.52
        msg.min_range = min_r
        msg.max_range = max_r
        msg.range = dist_m if dist_m is not None else float('nan')
        return msg

    def _read_short(self):
        # VL53L0X: range (mm)
        if not self.sensor_short:
            return None
        try:
            mm = self.sensor_short.range
            if mm is None or mm <= 0:
                return None
            return mm / 1000.0
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Read error VL53L0X: {e}")
            return None

    def _read_long(self):
        # VL53L1X: check data_ready, distance (mm)
        if not self.sensor_long:
            return None
        try:
            if hasattr(self.sensor_long, "data_ready"):
                if not self.sensor_long.data_ready:
                    return None
            mm = self.sensor_long.distance
            if mm is None or mm <= 0:
                return None
            return mm / 1000.0
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Read error VL53L1X: {e}")
            return None

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            d_short = self._read_short()
            d_long  = self._read_long()
            
            self.pub_short.publish(self._range_msg(self.frame_short, 0.03, 2.0, d_short))
            self.pub_long.publish(self._range_msg(self.frame_long, 0.03, 4.0, d_long))
            rate.sleep()

    def shutdown(self):
        rospy.loginfo("Shutdown VL53 nodes...")
        try:
            if self.sensor_long and hasattr(self.sensor_long, "stop_ranging"):
                self.sensor_long.stop_ranging()
        except Exception:
            pass
        rospy.loginfo("VL53 nodes shutdown complete.")

if __name__ == "__main__":
    node = DualVL53Node()
    try:
        node.spin()
    finally:
        node.shutdown()