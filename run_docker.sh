export DISPLAY=:1.0
xhost +local:docker

docker run -it --net=host --device /dev/dri/ -e DISPLAY=$DISPLAY -v $HOME/bm_ws/:/opt/ros/overlay_ws/:rw black_mouth