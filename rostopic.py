import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8, String, Int8, Empty
from tf.transformations import quaternion_from_euler
from nav_function import nav_functionRequest

import os
import math
import tempfile
from enum import IntEnum

class CommandEnum(IntEnum):
    CMD_NAV_GOAL_STOP     = 0
    CMD_POS_ESTIMATE      = 1
    CMD_NAV_GOAL_START    = 2
    CMD_BYPASS_STOP       = 16
    CMD_BYPASS_START      = 17

def goal_callback(self, msg: PoseStamped):
         """将接收到的目标点封装成服务请求"""
         req = nav_functionRequest()
         req.cmd = "nav_point"
         req.name = str(self.current_map_id_)
         # 获取目标点坐标
         target_x = msg.pose.position.x
         target_y = msg.pose.position.y
         target_z = msg.pose.position.z

         if self.current_pose_ is not None:
             current_x = self.current_pose_.position.x
             current_y = self.current_pose_.position.y
             # 计算朝向角度
             delta_x = target_x - current_x
             delta_y = target_y - current_y
             angle = math.atan2(delta_y, delta_x)
             # 转换为四元数
             q = quaternion_from_euler(0, 0, angle)
             req.qx = q[0]
             req.qy = q[1]
             req.qz = q[2]
             req.qw = q[3]
         else:
             rospy.logwarn("未获取到当前位置，使用默认朝向")
             req.qx = msg.pose.orientation.x
             req.qy = msg.pose.orientation.y
             req.qz = msg.pose.orientation.z
             req.qw = msg.pose.orientation.w

         req.x = target_x
         req.y = target_y
         req.z = target_z
         req.align = False  # True不对齐 False对齐

         try:
             response = self.nav_srv(req)
             if response.success:
                 rospy.loginfo(f"导航服务调用成功！{req}")
             else:
                 rospy.logwarn("导航服务调用失败。")
         except rospy.ServiceException as e:
             rospy.logerr(f"服务调用异常: {e}")

def odom_callback(msg: Odometry):
    """
    将Odometry消息转为PoseStamped并发布到current_pose，并缓存到位姿文件中
    使用临时文件保证写入完整性
    """
    pose_stamped = PoseStamped()
    pose_stamped.header = msg.header
    pose_stamped.pose = msg.pose.pose
    
    rospy.loginfo(msg.header)
    rospy.loginfo(msg.pose.pose.position)
    rospy.loginfo(msg.pose.pose.orientation)
    
    # self.current_pose_ = msg.pose.pose
    # self.current_pose_pub.publish(pose_stamped)
    # self.last_laser_odom_time_ = rospy.Time.now()

    # # 使用临时文件写入，保证原子性
    # temp_dir = os.path.dirname(self.cache_pose_file_path)
    # try:
    #     with tempfile.NamedTemporaryFile(dir=temp_dir, delete=False) as tf:
    #         tf.write(f"{self.current_map_id_}\t"
    #                 f"{msg.pose.pose.position.x}\t"
    #                 f"{msg.pose.pose.position.y}\t"
    #                 f"{msg.pose.pose.position.z}\t"
    #                 f"{msg.pose.pose.orientation.w}\t"
    #                 f"{msg.pose.pose.orientation.x}\t"
    #                 f"{msg.pose.pose.orientation.y}\t"
    #                 f"{msg.pose.pose.orientation.z}\n".encode())
    #         temp_name = tf.name

    #     # 写入成功后替换原文件
    #     os.replace(temp_name, self.cache_pose_file_path)
    # except Exception as e:
    #     rospy.logerr(f"写入 cache_pose 文件失败: {e}")

# def cmd_vel_callback(self, msg: Twist):
#     """转发/cmd_vel到proxy_cmd_vel"""
#     self.cmd_vel_pub.publish(msg)
# def command_callback(self, msg):
#     """
#     处理接收到的 UInt8 命令
#     使用枚举匹配命令值
#     """
#     try:
#         cmd = CommandEnum(msg.data)
#     except ValueError:
#         rospy.logwarn(f"未知命令码: {msg.data}")
#         return

#     if cmd == CommandEnum.CMD_NAV_GOAL_STOP:
#         rospy.loginfo("执行命令：停止导航目标")
#         self.web_cmd_pub.publish(String(data="Nav stop"))  # 发布 web_cmd
#     elif cmd == CommandEnum.CMD_POS_ESTIMATE:
#         rospy.loginfo("执行命令：位置估计")
#     elif cmd == CommandEnum.CMD_NAV_GOAL_START:
#         rospy.loginfo("执行命令：开始导航目标")
#         self.web_cmd_pub.publish(String(data="Nav continue"))  # 发布 web_cmd
#     elif cmd == CommandEnum.CMD_BYPASS_STOP:
#         rospy.loginfo("执行命令：绕障停止")
#     elif cmd == CommandEnum.CMD_BYPASS_START:
#         rospy.loginfo("执行命令：绕障开始")
#     else:
#         rospy.logwarn("未处理的命令")

def nav_state_callback(msg):
    """
    接收并处理 /nav/state 的状态消息
    :param msg: std_msgs/String 类型
    """
    state_str = msg.data.strip()
    rospy.logdebug(f"收到导航状态: {state_str}")
    print('AR >>>', state_str)

    # if state_str == getattr(self, 'last_nav_state', None):
        # rospy.logdebug(f"收到导航状态: {state_str}")
        # return
    # self.last_nav_state = state_str

    # try:
    #     # 尝试直接转换为整数
    #     state_code = int(state_str)

    #     # 发布状态码到 /navigation_state
    #     # self.navigation_state_pub.publish(Int8(data=state_code))

    #     if state_code == -1:
    #         rospy.loginfo("状态：空闲，无导航任务")
    #     elif state_code == 0:
    #         rospy.loginfo("状态：正常导航中")
    #     elif state_code == 2:
    #         rospy.logwarn("状态：检测到障碍物，正在避障")
    #     elif state_code == 3:
    #         rospy.logerr("状态：避障失败，等待重新全局路径规划")
    #     elif state_code == 4:
    #         rospy.logwarn("状态：避障失败，重新全局路径规划成功")
    #     elif state_code == 5:
    #         rospy.logerr("状态：避障失败，重新全局路径规划失败")
    #         # self.web_cmd_pub.publish(String(data="Nav stop"))  # 发布 web_cmd
    #     elif state_code == 6:
    #         rospy.logwarn("状态：偏离轨迹，重新路径规划成功")
    #     elif state_code == 7:
    #         rospy.logerr("状态：偏离轨迹，重新路径规划失败")
    #         # self.web_cmd_pub.publish(String(data="Nav stop"))  # 发布 web_cmd
    #     elif state_code == 8:
    #         rospy.logwarn("状态：出现异常，重新路径规划成功")
    #     elif state_code == 9:
    #         rospy.logerr("状态：出现异常，重新路径规划失败")
    #         # self.web_cmd_pub.publish(String(data="Nav stop"))  # 发布 web_cmd
    #     elif state_code == 12:
    #         rospy.loginfo("状态：即将到达目标点，正在对齐终点")
    #     elif state_code == 13:
    #         rospy.loginfo("状态：已到达目标点")
    #         # self.goal_reach_pub.publish(Empty())  # 发布 Empty 消息
    #     elif state_code == 14:
    #         rospy.loginfo("状态：接收到目标点，路径规划成功")
    #     elif state_code == 15:
    #         rospy.logwarn("状态：接收到目标点，路径规划失败")
    #         # self.web_cmd_pub.publish(String(data="Nav stop"))  # 发布 web_cmd
    #     elif state_code == 16:
    #         rospy.logerr("状态：未接收到 base_link/odom，无法导航，请重定位")
    #         # self.web_cmd_pub.publish(String(data="Nav stop"))  # 发布 web_cmd
    #     else:
    #         rospy.logwarn(f"未知状态码: {state_code}")
    #         # self.web_cmd_pub.publish(String(data="Nav stop"))  # 发布 web_cmd

    # except ValueError:
    #     # 如果不是纯数字，则尝试解析带参数的状态码
    #     parts = state_str.split('#')
    #     if not parts:
    #         rospy.logwarn("无效状态信息")
    #         return

    #     try:
    #         main_code = int(parts[0])

    #         # # 发布状态码到 /navigation_state
    #         # self.navigation_state_pub.publish(Int8(data=main_code))

    #         if main_code == 1:
    #             index = int(parts[1])
    #             total = int(parts[2])
    #             rospy.loginfo(f"状态：路径点 {index}/{total}")
    #         elif main_code == 10 or main_code == 11:
    #             target_id = int(parts[1])
    #             current_idx = int(parts[2])
    #             total_points = int(parts[3])
    #             status = "成功" if main_code == 10 else "失败"
    #             rospy.loginfo(f"状态：多点导航，目标 {target_id}，当前点 {current_idx}/{total_points}，路径规划{status}")
    #         elif main_code == 17 or main_code == 18 or main_code == 19:
    #             current_idx = int(parts[1])
    #             total_points = int(parts[2])
    #             status = {
    #                 17: "成功",
    #                 18: "失败",
    #                 19: "开始等待"
    #             }.get(main_code, "未知")
    #             rospy.loginfo(f"状态：ROS接口多点导航，当前点 {current_idx}/{total_points}，路径规划{status}")
    #         else:
    #             rospy.logwarn(f"未定义的复合状态码: {main_code}")

    #     except (ValueError, IndexError) as e:
    #         rospy.logerr(f"解析状态码失败: {e}")
    #         # self.web_cmd_pub.publish(String(data="Nav stop"))  # 发布 web_cmd

# def map_id_callback(self, msg: Int8):
#     """
#     接收并处理 /map_id 的地图ID
#     :param msg: std_msgs/Int8 类型
#     """
#     map_id = msg.data
#     rospy.loginfo(f"收到地图ID: {map_id}")
#     # 此处可以添加进一步处理逻辑，例如保存到成员变量或触发其他操作
#     self.current_map_id_ = map_id
# def web_cmd_callback(self, msg: String):
#     """
#     接收并处理 /web_cmd 消息，提取地图ID
#     :param msg: std_msgs/String 类型
#     """
#     try:
#         if msg.data == "Nav stop" or msg.data == "Nav continue":
#             return

#         parts = msg.data.split('#')
#         if len(parts) >= 2 and parts[0] == "current map name":
#             map_id_str = parts[1]
#             if not map_id_str.isdigit():
#                 raise ValueError("无效的地图ID")
#             received_map_id = int(map_id_str)

#             if self.current_map_id_ != received_map_id:
#                 # 更新内存中的地图ID
#                 self.current_map_id_ = received_map_id

#                 # # 写入文件
#                 # try:
#                 #     with open(self.map_id_file_path, 'w') as f:
#                 #         f.write(str(received_map_id))
#                 #     rospy.loginfo(f"地图ID已更新并写入文件: {received_map_id}")
#                 # except Exception as e:
#                 #     rospy.logerr(f"写入 map_id 文件失败: {e}")

#         else:
#             rospy.logwarn(f"无效的 /web_cmd 格式: {msg.data}")
#     except ValueError as e:
#         rospy.logerr(f"解析地图ID失败: {e}")
# def publish_probability_callback(self, event):
    """
    定时检查是否收到 /laser/odom 数据，并发布 probability
    """
    if self.last_laser_odom_time_ is None:
        rospy.logwarn("未收到 /laser/odom 数据")
        return
    time_diff = (rospy.Time.now() - self.last_laser_odom_time_).to_sec()

    if time_diff < 3.0:  # 如果最近3秒内收到过数据
        prob = UInt8()
        prob.data = int(self.prob_)
        self.probability_pub.publish(prob)
    else:
        prob = UInt8()
        prob.data = 1
        self.probability_pub.publish(prob)
