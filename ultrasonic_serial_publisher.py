#!/usr/bin/env python3
"""
ultrasonic_serial_publisher.py

ROS1 (Noetic) node that reads a USB-serial stream from an Arduino and publishes
two ultrasonic distance measurements as Float32 topics.

Expected Arduino line format (one per line):
    start_cm,middle_cm

This implementation is intentionally tolerant of common formatting variations:
- leading decimal: .56,67.43
- trailing decimal: 12.,34.
- extra spaces:  12.3 ,  45.6
- missing value (optional handling): ,53.7  or  12.3,
  -> treated as NaN and (by default) skipped

Publishes:
- /ultrasonic_distance_start  (std_msgs/Float32)
- /ultrasonic_distance_middle (std_msgs/Float32)

Parameters (private ~):
- ~port (str): serial device path (default: /dev/ttyACM0)
- ~baud (int): baud rate (default: 9600)
- ~rate_hz (float): loop rate (default: 30.0)
- ~topic_start (str): topic name (default: /ultrasonic_distance_start)
- ~topic_middle (str): topic name (default: /ultrasonic_distance_middle)
- ~startup_flush_s (float): seconds to ignore input after opening port (default: 1.0)
- ~allow_missing (bool): if True, allow missing values and publish only valid ones (default: False)
- ~min_valid_cm (float): discard values below this (default: 0.0)
- ~max_valid_cm (float): discard values above this (default: 500.0)

Run examples:
  rosrun ur3e_moveit_config ultrasonic_serial_publisher.py _port:=/dev/ttyACM1 _baud:=9600
"""

import math
import re
import serial
import rospy
from std_msgs.msg import Float32


# Number pattern that accepts:
#  12, 12.3, 12., .56, -0.7, -.25, +3.0
_NUM = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)"
_LINE_RE = re.compile(rf"^\s*({_NUM})?\s*,\s*({_NUM})?\s*$")


def _to_float_or_nan(s: str):
    if s is None:
        return float("nan")
    s = s.strip()
    if not s:
        return float("nan")
    try:
        return float(s)
    except ValueError:
        return float("nan")


def parse_line(line: str):
    """
    Returns (start_cm, middle_cm) as floats.
    If parsing fails, returns None.
    Missing values become NaN.
    """
    m = _LINE_RE.match(line)
    if not m:
        return None
    start_cm = _to_float_or_nan(m.group(1))
    middle_cm = _to_float_or_nan(m.group(2))
    return start_cm, middle_cm


def is_valid(val: float, min_cm: float, max_cm: float) -> bool:
    if math.isnan(val):
        return False
    return (val >= min_cm) and (val <= max_cm)


def open_serial(port: str, baud: int):
    """
    Open serial port with conservative settings.
    """
    return serial.Serial(
        port=port,
        baudrate=baud,
        timeout=1.0,
        write_timeout=1.0,
    )


def main():
    rospy.init_node("ultrasonic_serial_publisher", anonymous=False)

    port = rospy.get_param("~port", "/dev/ttyACM0")
    baud = int(rospy.get_param("~baud", 9600))
    rate_hz = float(rospy.get_param("~rate_hz", 30.0))

    topic_start = rospy.get_param("~topic_start", "/ultrasonic_distance_start")
    topic_middle = rospy.get_param("~topic_middle", "/ultrasonic_distance_middle")

    startup_flush_s = float(rospy.get_param("~startup_flush_s", 1.0))
    allow_missing = bool(rospy.get_param("~allow_missing", False))

    min_valid_cm = float(rospy.get_param("~min_valid_cm", 0.0))
    max_valid_cm = float(rospy.get_param("~max_valid_cm", 500.0))

    pub_start = rospy.Publisher(topic_start, Float32, queue_size=10)
    pub_middle = rospy.Publisher(topic_middle, Float32, queue_size=10)

    rospy.loginfo(f"[ultrasonic_serial_publisher] Opening serial: {port} @ {baud}")
    try:
        ser = open_serial(port, baud)
    except Exception as e:
        rospy.logerr(f"[ultrasonic_serial_publisher] Failed to open {port}: {e}")
        return

    # Give Arduino time to reset after serial open, then flush junk.
    if startup_flush_s > 0:
        rospy.sleep(startup_flush_s)
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    rate = rospy.Rate(rate_hz)
    bad_line_count = 0

    while not rospy.is_shutdown():
        try:
            raw = ser.readline()
            if not raw:
                rate.sleep()
                continue

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                rate.sleep()
                continue

            parsed = parse_line(line)
            if parsed is None:
                bad_line_count += 1
                rospy.logwarn_throttle(
                    2.0,
                    f"[ultrasonic_serial_publisher] Unparsed line (count={bad_line_count}): {line!r}",
                )
                rate.sleep()
                continue

            start_cm, middle_cm = parsed

            start_ok = is_valid(start_cm, min_valid_cm, max_valid_cm)
            middle_ok = is_valid(middle_cm, min_valid_cm, max_valid_cm)

            if not allow_missing:
                # Require both values valid before publishing.
                if not (start_ok and middle_ok):
                    rospy.logwarn_throttle(
                        2.0,
                        f"[ultrasonic_serial_publisher] Dropped invalid sample: start={start_cm}, middle={middle_cm}",
                    )
                    rate.sleep()
                    continue
                pub_start.publish(Float32(data=start_cm))
                pub_middle.publish(Float32(data=middle_cm))
            else:
                # Publish whichever is valid.
                if start_ok:
                    pub_start.publish(Float32(data=start_cm))
                if middle_ok:
                    pub_middle.publish(Float32(data=middle_cm))
                if not (start_ok or middle_ok):
                    rospy.logwarn_throttle(
                        2.0,
                        f"[ultrasonic_serial_publisher] Dropped invalid sample: start={start_cm}, middle={middle_cm}",
                    )

        except serial.SerialException as e:
            rospy.logerr(f"[ultrasonic_serial_publisher] Serial error: {e}")
            rospy.sleep(1.0)
            # Attempt reopen
            try:
                ser.close()
            except Exception:
                pass
            try:
                rospy.loginfo(f"[ultrasonic_serial_publisher] Re-opening serial: {port} @ {baud}")
                ser = open_serial(port, baud)
                if startup_flush_s > 0:
                    rospy.sleep(startup_flush_s)
                ser.reset_input_buffer()
            except Exception as e2:
                rospy.logerr(f"[ultrasonic_serial_publisher] Re-open failed: {e2}")
                rospy.sleep(2.0)

        except Exception as e:
            rospy.logwarn_throttle(2.0, f"[ultrasonic_serial_publisher] Unexpected error: {e}")

        rate.sleep()

    try:
        ser.close()
    except Exception:
        pass


if __name__ == "__main__":
    main()