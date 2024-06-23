# ROS2
source /opt/ros/rolling/setup.zsh
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
# argcomplete for ros2 & colcon
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"

# seasource-ros
# export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
source /home/pi/seasource-ros/install/local_setup.zsh
export ROS_HOME=/home/pi/.ros/
export ROS_LOG_DIR=$ROS_HOME/log/

export XDG_RUNTIME_DIR="/run/user/1000"
