#!/bin/bash

# create workspace
source /opt/ros/melodic/setup.bash

cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin build

# clone repo
cd ~/catkin_ws/src
git clone https://github.com/tiger0421/intersection_recognition.git
catkin build
cd ~/

echo '#!/bin/sh' > /tmp/docker-entrypoint.sh
echo 'bash' >> /tmp/docker-entrypoint.sh

exec "$0"
