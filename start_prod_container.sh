docker stop rosdog
docker rm -f rosdog
docker run -d \
  --rm \
  --name rosdog \
  --net=host \
  --entrypoint /ros_entrypoint.sh \
  -v /home/unitree/arthur/ros_control:/ar_data \
  -e LD_LIBRARY_PATH="/ar_data/z1_sdk/lib:$LD_LIBRARY_PATH" \
  arosc:dbg bash -c " cd /ar_data && bash run_uvicorn.sh"
