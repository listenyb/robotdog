docker run -d \
  --name ros_control \
  --net=host \
  -v /home/oem/arthur:/ar_data \
  ros:noetic-ros-base sleep 400000
