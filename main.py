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

import os
os.environ['ROS_MASTER_URI']='http://192.168.123.164'
os.environ['ROS_IP']='192.168.123.164'

import rospy

import threading
import time
from fastapi import FastAPI
from fastapi.resposne import JSONResponse
from pydantic import BaseModel

# from robot.unitree.libs.unitree_sdk2py.b2.sport.sport_client import SportClient
# from robot.unitree.libs.unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.b2.sport.sport_client import SportClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
import numpy  as np

app = FastAPI()

class Config(BaseModel):
    address: str


dog_client = SportClient()
dog_client.setTimeout(10.0)
dog_client.Init()

# state_client = RobotStateClient()
# state_client.SetTimeout(10.0)
# state_client.Init()

ChannelFactoryInitialize()
sub = ChannelSubscriber("rt/lf/odommodestate", SportModeState_)
sub.Init()

status_message = ""
status_lock = threading.Lock()

def read_status():
    global status_message

    while True:
        msg = sub.Read()
        if msg is not None:
            print("Subscribe success. msg:", msg)
            with status_lock:
                status_message = msg
        else:
            print("No data subscribed.")
            # break
        time.sleep(1)
    
    sub.Close()


status_thread = threading.Thread(target=read_status, args=())
status_thread.daemon = True
status_thread.start()


def _get_status() -> str:
    pass



@app.post("/api/v1/robot/status")
def status(config: Config):
    with status_lock:
        print(status_message)
        return JSONResponse(content={"status": status_message})

@app.post("/api/v1/robot/stand")
def stand(config: Config):
    b2.send_motion_control_balance_stand()
    pass

@app.post("/api/v1/robot/stop")
def stop(config: Config):
    b2.send_motion_control_stop()
    pass

@app.post("/api/v1/robot/sit")
def sit(config: Config):
    b2.send_motion_control_lie_down()
    pass

@app.post("/api/v1/robot/move_forward")
def move_forward(config: Config):
    b2.send_motion_control_forward()

@app.post("/api/v1/robot/move_backward")
def move_backward(config: Config):
    b2.send_motion_control_backward()

@app.post("/api/v1/robot/turn_left")
def turn_left(config: Config):
    b2.send_motion_control_turn_left()
    pass

@app.post("/api/v1/robot/turn_right")
def turn_right(config: Config):
    b2.send_motion_control_turn_right()
    pass

@app.move_left("/api/v1/robot/move_left")
def move_left(config: Config):
    b2.send_motion_control_sidestep_left()
    pass

@app.move_right("/api/v1/robot/move_rigth")
def move_right(config: Config):
    b2.send_motion_control_sidestep_right()
    pass

@app.go_to("/api/v1/robot/go_to")
def go_to(config: Config):
    b2.send_motion_control_backward()
    pass
