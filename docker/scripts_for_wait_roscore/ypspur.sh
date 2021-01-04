#/bin/bash

ros_master_flg=0
while :; do
    rostopic list
    ros_master_flg=$?
    if [ $ros_master_flg -eq 1 ]; then
        echo "Wait 1 sec"
    else
        command: bash -c "source /opt/ros/melodic/setup.bash; rosrun ypspur_ros ypspur_ros _param_file:=/root/icart_param/icart-middle.param _device:=/dev/ttyACM0"
        break
    fi
    sleep 1

done
