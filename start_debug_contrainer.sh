docker run -d \
  --name rosc \
  --net=host \
  --entrypoint /ros_entrypoint.sh \
  -v /home/unitree/arthur/ros_control:/ar_data \
  arosc:dbg bash -c "sleep 300000"
  #-v /home/oem/arthur:/ar_data \
