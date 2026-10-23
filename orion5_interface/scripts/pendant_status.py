#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pendant_status.py — print selected keys of /pendant/status (verification helper).

  rosrun orion5_interface pendant_status.py                 # whole status once
  rosrun orion5_interface pendant_status.py state plan tcp  # selected keys once
  rosrun orion5_interface pendant_status.py --watch task    # print task step transitions until Ctrl-C
  rosrun orion5_interface pendant_status.py --info          # /pendant/robot_info once
"""
import json
import sys
import rospy
from std_msgs.msg import String


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("__")]
    rospy.init_node("pendant_status", anonymous=True)
    if "--info" in args:
        msg = rospy.wait_for_message("/pendant/robot_info", String, timeout=10.0)
        print(json.dumps(json.loads(msg.data), indent=2, ensure_ascii=False))
        return
    if "--watch" in args:
        last = {}

        def cb(msg):
            st = json.loads(msg.data)
            t = st.get("task", {})
            key = (st.get("state"), t.get("step_index"), t.get("running"), t.get("result"),
                   tuple(s.get("state") for s in t.get("steps", [])))
            if key != last.get("k"):
                last["k"] = key
                print("%7.2fs  state=%-15s step=%s/%s %-14s running=%s result=%s cycle=%ss err=%s" % (
                    rospy.get_time() % 1000, st.get("state"), (t.get("step_index") or 0) + 1, t.get("step_count"),
                    t.get("step_name"), t.get("running"), t.get("result"), t.get("cycle_time_s"), st.get("last_error")))
                if t.get("result") and not t.get("running"):
                    print("steps:", json.dumps(t.get("steps"), ensure_ascii=False))
                    print("last_cycle_time_s:", t.get("last_cycle_time_s"), "count:", t.get("count"))
        rospy.Subscriber("/pendant/status", String, cb, queue_size=1)
        rospy.spin()
        return
    msg = rospy.wait_for_message("/pendant/status", String, timeout=10.0)
    st = json.loads(msg.data)
    keys = [a for a in args if not a.startswith("--")]
    out = {k: st.get(k) for k in keys} if keys else st
    print(json.dumps(out, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
