FROM ros:noetic-ros-base

RUN apt update && apt install -y \
  vim \
  python3-pip \
  ros-noetic-tf

RUN pip3 install \
  fastapi==0.115.13 \
  pydantic==2.10.6 \
  cyclonedds==0.10.2 \
  numpy==1.17.4 \
  opencv-python==4.11.0.86 \
  uvicorn==0.33.0 \
  -i https://pypi.tuna.tsinghua.edu.cn/simple

WORKDIR /app
ENTRYPOINT ["/ros_entrypoint.sh"]
