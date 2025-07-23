# import logging
# from typing import Dict


# logger = logging.getLogger()
# logger.setLevel(logging.INFO)
# formatter = logging.Formatter(
#     "%(asctime)s.%(msecs)03d [RobotDog] %(levelname)-10s %(message)s",
#     datefmt="%Y-%m-%d %H:%M:%S",
# )
# console_handler = logging.StreamHandler()
# console_handler.setLevel(logging.DEBUG)
# console_handler.setFormatter(formatter)
# logger.addHandler(console_handler)

import sys
print('----------------------')
sys.path.append("/ar_data/z1_sdk/lib")
import unitree_arm_interface
import time
import numpy as np
np.set_printoptions(precision=3, suppress=True)

import os
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8, String, Int8, Empty
import rostopic as roscb

import threading
import time
from fastapi import FastAPI
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field

from unitree_sdk2py.b2.sport.sport_client import SportClient
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from nav_function import  nav_function, nav_functionRequest

class Movement(BaseModel):
    px: float
    py: float
    pz: float
    qx: float
    qy: float
    qz: float
    qw: float


battery_status_lock = threading.Lock()
battery_status = {}


ChannelFactoryInitialize(0, "eth0")

def ut_lf_lowstate_callback(msg: LowState_):
    # print(msg)
    # print(msg.imu_state)
    # print(msg.motor_state)
    # print(msg.bms_state)
    # print(msg.power_v)
    # print(msg.power_a)
    # print(msg.wireless_remote)
    global battery_status
    with battery_status_lock:
        battery_status = {
            "soc": msg.bms_state.soc,
            "v": msg.power_v,
            "a": msg.power_a,
        }

def ut_lf_odomstate_callback(msg: SportModeState_):
    # print(msg)
    # print(msg.imu_state)
    # print("pos:", msg.position)
    # print("v:", msg.velocity)
    # print("euler ang:", msg.imu_state.rpy)
    # print("q:", msg.imu_state.quaternion)
    pass

ut_lf_lowstate_sub = ChannelSubscriber("rt/lf/lowstate", LowState_)
ut_lf_lowstate_sub.Init(ut_lf_lowstate_callback, 10)
print("Unitree LowState_ 订阅成功")

ut_lf_odomstate_sub = ChannelSubscriber("rt/lf/odommodestate", SportModeState_)
ut_lf_odomstate_sub.Init(ut_lf_odomstate_callback, 10)
print("Unitree OdomState_ 订阅成功")

b2_client = SportClient()
b2_client.SetTimeout(10.0)
b2_client.Init()
ret = b2_client.SpeedLevel(-1)
print(f"Unitree SpeedLevel << -1: {ret}")
ret = b2_client.WalkStair(True)
print(f"Unitree WalkStair << T: {ret}")


B2_SPEED = 0.5


status_lock = threading.Lock()
odom = PoseStamped()

nav_status_lock = threading.Lock()
navs_msg = "-2"

print('DEBUG ================ 6 == ')

rospy.init_node("ar_control_node")
#rospy.init_node("ar_control_node", anonymous=True)
# 1. 订阅 /move_base_simple/goal 并调用服务
# nav_goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, roscb.goal_callback)
# 2. 订阅 /base_link/odom 并发布 current_pose
move_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
rospy.loginfo("MOVE 订阅成功")

def odom_callback(msg: Odometry):
    global odom
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    # rospy.logwarn(msg)
    with status_lock:
        odom.pose = msg.pose.pose

odom_sub = rospy.Subscriber("/base_link/odom", Odometry, odom_callback)
rospy.loginfo("ODOM 订阅成功")

def navs_callback(msg):
    state_code = msg.data.strip()

    global navs_msg
    with nav_status_lock:
        navs_msg = state_code

    if state_code == "-1":
        rospy.loginfo("状态：空闲，无导航任务")
    elif state_code == "0":
        rospy.logwarn("状态：正常导航中")
    elif state_code == "2":
        rospy.logwarn("状态：检测到障碍物，正在避障")
    elif state_code == "3":
        rospy.logerr("状态：避障失败，等待重新全局路径规划")
    elif state_code == "4":
        rospy.logwarn("状态：避障失败，重新全局路径规划成功")
    elif state_code == "5":
        rospy.logerr("状态：避障失败，重新全局路径规划失败")
    elif state_code == "6":
        rospy.logwarn("状态：偏离轨迹，重新路径规划成功")
    elif state_code == "7":
        rospy.logerr("状态：偏离轨迹，重新路径规划失败")
    elif state_code == "8":
        rospy.logwarn("状态：出现异常，重新路径规划成功")
    elif state_code == "9":
        rospy.logerr("状态：出现异常，重新路径规划失败")
    elif state_code == "12":
        rospy.logwarn("状态：即将到达目标点，正在对齐终点")
    elif state_code == "13":
        rospy.logwarn("状态：已到达目标点")
    elif state_code == "14":
        rospy.loginfo("状态：接收到目标点，路径规划成功")
    elif state_code == "15":
        rospy.logwarn("状态：接收到目标点，路径规划失败")
    elif state_code == "16":
        rospy.logerr("状态：未接收到 base_link/odom，无法导航，请重定位")
    else:
        rospy.logerr(f"未知状态码: {state_code}")
    
navs_sub = rospy.Subscriber("/nav/state", String, navs_callback)
rospy.loginfo("NAV/STATE 订阅成功")
# current_pose_pub = rospy.Publisher("current_pose", PoseStamped, queue_size=10)
# # 3. 订阅 /cmd_vel 并转发
# cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, roscb.cmd_vel_callback)
# cmd_vel_pub = rospy.Publisher("proxy_cmd_vel", Twist, queue_size=10)
# # 4. 订阅 /command 并处理命令
# command_sub = rospy.Subscriber("/command", UInt8, roscb.command_callback)
# # 5. 发布 /web_cmd 控制指令
web_cmd_pub = rospy.Publisher("/web_cmd", String, queue_size=10)
rospy.loginfo("WEB_CMD(Pub) 订阅成功")
# 6. 订阅 /nav/state 状态信息
# nav_state_sub = rospy.Subscriber("/nav/state", String, roscb.nav_state_callback)
# # 7. 发布 /goal_reach 目标点到达通知
# goal_reach_pub = rospy.Publisher("/goal_reach", Empty, queue_size=10)
# # 8. 发布 /navigation_state 状态码
# navigation_state_pub = rospy.Publisher("/navigation_state", Int8, queue_size=10)
# # 9. 订阅 /map_id 地图ID
# map_id_sub = rospy.Subscriber("/map_id", Int8, map_id_callback)
# # 10. 订阅 /web_cmd 控制指令
# web_cmd_sub = rospy.Subscriber("/web_cmd", String, web_cmd_callback)
# 等待服务可用
# rospy.wait_for_service("/z_nav/nav_function")
# try:
#     self.nav_srv = rospy.ServiceProxy("/z_nav/nav_function", nav_function)
# except rospy.ServiceException as e:
#     rospy.logerr(f"Service initialization failed: {e}")
#     raise
nav_srv = rospy.ServiceProxy("/z_nav/nav_function", nav_function)
print('DEBUG ================ 7 == ')

app = FastAPI()

@app.post("/api/v1/robot/status")
def pos_status():
    with status_lock:
        pos = odom.pose.position
        ori = odom.pose.orientation
        data = {
                "pos": {"x": pos.x, "y": pos.y, "z": pos.z},
                "ori": {"x": ori.x, "y": ori.y, "z": ori.z, "w": ori.w},
        }
        return JSONResponse(content=data)

@app.post("/api/v1/robot/navs_status")
def nav_status():
    with nav_status_lock:
        data = {"navs": navs_msg}
        return JSONResponse(content=data)

@app.post("/api/v1/robot/battery")
def battery_status():
    with battery_status_lock:
        return JSONResponse(content=battery_status)


@app.post("/api/v1/robot/nav_stop")
def nav_stop():
    web_cmd_pub.publish(String(data="Nav stop"))
    return JSONResponse(content={"code": 200, "msg": "pub nav_stop"})

@app.post("/api/v1/robot/nav_continue")
def nav_stop():
    web_cmd_pub.publish(String(data="Nav continue"))
    return JSONResponse(content={"code": 200, "msg": "pub nav_continue"})


@app.post("/api/v1/robot/go_to")
def go_to(move: Movement):
    req = nav_functionRequest()
    req.cmd = "nav_point"
    req.name = '070913'
    # 获取目标点坐标
    req.x = move.px
    req.y = move.py
    req.z = move.pz
    req.qx = move.qx
    req.qy = move.qy
    req.qz = move.qz
    req.qw = move.qw
    req.align = False
    # 发布目标
    rospy.loginfo("Publishing pos: (x=%.2f, y=%.2f, z=%.2f)", move.px, move.py, move.pz)
    rospy.loginfo("Publishing ori: (x=%.2f, y=%.2f, z=%.2f, w=%.2f)", move.qx, move.qy, move.qz, move.qw)
    nav_srv(req)
    return JSONResponse(content={"code": 200, "msg": "ok"})


@app.post("/api/v1/robot/go_to_0")
def go_to():
    req = nav_functionRequest()
    req.cmd = "nav_point"
    req.name = '070913'
    # 获取目标点坐标
    req.x = 0.00
    req.y = 0.00
    req.z = 0.00
    req.qx = 0.00
    req.qy = 0.00
    req.qz = 0.00
    req.qw = 0.99
    req.align = False
    # 发布目标
    rospy.loginfo("Publishing pos: (x=%.2f, y=%.2f, z=%.2f)", req.x, req.y, req.z)
    rospy.loginfo("Publishing ori: (x=%.2f, y=%.2f, z=%.2f, w=%.2f)", req.x, req.y, req.z, req.w)
    nav_srv(req)
    return JSONResponse(content={"code": 200, "msg": "ok"})





#@app.post("/api/v1/robot/go_to")
#def go_to(move: Movement):
#    goal = PoseStamped()
#    goal.header.frame_id = "map"
#    goal.header.stamp = rospy.Time.now()
#
#    goal.pose.position.x = move.px
#    goal.pose.position.y = move.py
#    goal.pose.position.z = move.pz
#    """
#    # 将 yaw（角度）转换为四元数
#    yaw_rad = yaw_deg * 3.1415926 / 180.0
#    q = quaternion_from_euler(0, 0, yaw_rad)
#    goal.pose.orientation.x = q[0]
#    goal.pose.orientation.y = q[1]
#    goal.pose.orientation.z = q[2]
#    goal.pose.orientation.w = q[3]
#    """
#    goal.pose.orientation.x = move.qx
#    goal.pose.orientation.y = move.qy
#    goal.pose.orientation.z = move.qz
#    goal.pose.orientation.w = move.qw
#
#    # 发布目标
#    rospy.loginfo("Publishing pos: (x=%.2f, y=%.2f, z=%.2f)", move.px, move.py, move.pz)
#    rospy.loginfo("Publishing ori: (x=%.2f, y=%.2f, z=%.2f, w=%.2f)", move.qx, move.qy, move.qz, move.qw)
#    move_pub.publish(goal)
#    return JSONResponse(content={"code": 200, "msg": "ok"})

print('DEBUG ================ 8 == ')

#x = input('wait input....')


@app.post("/api/v1/robot/stand")
def stand():
    ret = b2_client.BalanceStand()
    rospy.logwarn(f"BalanceStand: {ret}")
    return JSONResponse(content={"code": ret, "msg": "ok"})

@app.post("/api/v1/robot/setWalkStairGait")
def walkstair():
    ret_s = b2_client.SpeedLevel(0)
    ret_g = b2_client.WalkStair(True)
    rospy.logwarn(f"WalkStair: {ret}")
    return JSONResponse(content={"code": ret_g, "msg": "ok", "ret_g": ret_g, "ret_s": ret_s})

@app.post("/api/v1/robot/setLowSpeed")
def walkstair():
    ret_s = b2_client.SpeedLevel(-1)
    rospy.logwarn(f"LowSpeed: {ret_s}")
    return JSONResponse(content={"code": ret_s, "msg": "ok"})

@app.post("/api/v1/robot/setNormalSpeed")
def walkstair():
    ret_s = b2_client.SpeedLevel(1)
    rospy.logwarn(f"NormalSpeed: {ret_s}")
    return JSONResponse(content={"code": ret_s, "msg": "ok"})

@app.post("/api/v1/robot/stop")
def stop():
    ret = b2_client.StopMove()
    rospy.logwarn(f"StopMove: {ret}")
    return JSONResponse(content={"code": ret, "msg": "ok"})

@app.post("/api/v1/robot/sit")
def sit():
    ret = b2_client.StandDown()
    rospy.logwarn(f"StandDown: {ret}")
    return JSONResponse(content={"code": ret, "msg": "ok"})

@app.post("/api/v1/robot/move_forward")
def move_forward():
    #ret = b2_client.BalanceStand()
    #print(f"BalanceStand: {ret}")
    ret = b2_client.Move(B2_SPEED, 0, 0)
    print(f"Move: {ret}")
    return JSONResponse(content={"code": ret, "msg": "ok"})

@app.post("/api/v1/robot/move_backward")
def move_backward():
    #ret = b2_client.BalanceStand()
    #print(f"BalanceStand: {ret}")
    ret = b2_client.Move(-B2_SPEED, 0, 0)
    print(f"Move: {ret}")
    return JSONResponse(content={"code": ret, "msg": "ok"})

@app.post("/api/v1/robot/turn_left")
def turn_left():
    #ret = b2_client.BalanceStand()
    #print(f"BalanceStand: {ret}")
    ret = b2_client.Move(0, 0, B2_SPEED)
    print(f"Move: {ret}")
    return JSONResponse(content={"code": ret, "msg": "ok"})

@app.post("/api/v1/robot/turn_right")
def turn_right():
    #ret = b2_client.BalanceStand()
    #print(f"BalanceStand: {ret}")
    ret = b2_client.Move(0, 0, -B2_SPEED)
    print(f"Move: {ret}")
    return JSONResponse(content={"code": ret, "msg": "ok"})

@app.post("/api/v1/robot/move_left")
def move_left():
    #ret = b2_client.BalanceStand()
    #print(f"BalanceStand: {ret}")
    ret = b2_client.Move(0, B2_SPEED, 0)
    print(f"Move: {ret}")
    return JSONResponse(content={"code": ret, "msg": "ok"})

@app.post("/api/v1/robot/move_right")
def move_right():
    #ret = b2_client.BalanceStand()
    #print(f"BalanceStand: {ret}")
    ret = b2_client.Move(0, -B2_SPEED, 0)
    print(f"Move: {ret}")
    return JSONResponse(content={"code": ret, "msg": "ok"})


@app.post("/api/v1/robot_arm/arm_control")
def robot_arm_control():
    arm = unitree_arm_interface.ArmInterface(hasGripper=True)
    armState = unitree_arm_interface.ArmFSMState
    arm.loopOn()
    time.sleep(0.5)
    #model = arm._ctrlComp.armModel
    print("q:", arm.q)
    arm.labelRun("test")
    time.sleep(1)
    arm.labelRun("back1")
    arm.labelRun("back2")
    arm.labelRun("back3")
    arm.backToStart()
    arm.loopOff()
    print(f"Control : {armState}")
    return JSONResponse(content={"code": "200", "msg": "ok"})

#rospy.spin()
