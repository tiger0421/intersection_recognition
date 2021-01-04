#/bin/bash

ros_master_flg=0
while :; do
    rostopic list
    ros_master_flg=$?
    if [ $ros_master_flg -eq 1 ]; then
        echo "Wait 1 sec"
    else
        rviz
        break
    fi
    sleep 1

done
