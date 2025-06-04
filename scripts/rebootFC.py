#!/usr/bin/env python3
import rospy
from mavros_msgs.srv import CommandLong, CommandLongRequest

def reboot_px4(ns=""):
    rospy.init_node("reboot_px4_node")
    service_name = f"/{ns}/mavros/cmd/command"
    rospy.wait_for_service(service_name)

    try:
        cmd_client = rospy.ServiceProxy(service_name, CommandLong)

        req = CommandLongRequest()      # ← 关键
        req.broadcast     = False       # 只发给当前飞控
        req.command       = 246         # PREFLIGHT_REBOOT_SHUTDOWN
        req.confirmation  = 0
        req.param1        = 1           # 1 = reboot autopilot（0 不动作，2 关机）
        req.param2 = req.param3 = req.param4 = \
        req.param5 = req.param6 = req.param7 = 0

        resp = cmd_client(req)
        if resp.success:
            rospy.loginfo(f"[{ns}] PX4 正在重启...")
        else:
            rospy.logwarn(f"[{ns}] 重启失败，result={resp.result}")

    except rospy.ServiceException as e:
        rospy.logerr(f"调用 {service_name} 失败：{e}")

if __name__ == "__main__":
    reboot_px4()
