#!/usr/bin/env python
import rospy
from mavros_msgs.srv import CommandLong
from mavros_msgs.srv import CommandLong as CommandLongSrv

def reboot_px4():
    rospy.init_node('reboot_px4_node', anonymous=True)

    # 等待 mavros 服务
    rospy.wait_for_service('/mavros/cmd/command')
    try:
        # 创建 mavros 服务客户端
        command_srv = rospy.ServiceProxy('/mavros/cmd/command', CommandLongSrv)
        
        # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN 的命令 ID 是 176
        # 参数：0 表示重启，1 表示关机
        reboot_command = CommandLong(
            command=176,  # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
            param1=0,     # 0 表示重启，1 表示关机
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
        
        # 发送重启命令
        response = command_srv(reboot_command)
        if response.success:
            rospy.loginfo("重启命令已成功发送，飞控正在重启...")
        else:
            rospy.logwarn("发送重启命令失败")
    
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: %s" % e)

if __name__ == "__main__":
    try:
        reboot_px4()
    except rospy.ROSInterruptException:
        pass