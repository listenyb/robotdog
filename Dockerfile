FROM ros:noetic-ros-base

RUN apt update && apt install -y \
  python3-pip

RUN pip3 install -y fastapi pydantic

WORKDIR /app


