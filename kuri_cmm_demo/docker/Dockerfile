FROM ros:melodic
#USER root
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
apt update && \
apt-get install -y -qq iputils-ping \
libboost-all-dev \
ros-melodic-catkin \
ros-melodic-base-local-planner \
ros-melodic-dynamic-reconfigure \
ros-melodic-dwa-local-planner \
ros-melodic-geometry-msgs \
ros-melodic-map-server \
ros-melodic-move-base-msgs \
ros-melodic-message-generation \
ros-melodic-message-runtime \
ros-melodic-nav-msgs \
ros-melodic-nodelet \
ros-melodic-pluginlib \
ros-melodic-robot-state-publisher \
ros-melodic-rosbash \
ros-melodic-roscpp \
ros-melodic-roslaunch \
ros-melodic-rospy \
ros-melodic-sensor-msgs \
ros-melodic-std-msgs \
ros-melodic-tf-conversions \
ros-melodic-tf2-ros \
ros-melodic-tf2-geometry-msgs \
ros-melodic-cv-bridge \
ros-melodic-ros-control \
ros-melodic-ros-controllers \
python-boto3 \
python-opencv \
python-numpy \
python-scipy

COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
