FROM ros:melodic
#USER root
# RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
# apt update && \
RUN apt update && \
apt-get install -y -qq iputils-ping \
libboost-all-dev \
python-pip \
ros-melodic-catkin \
ros-melodic-base-local-planner \
ros-melodic-control-msgs \
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
ros-melodic-tf2-geometry-msgs && \
pip install -U catkin_tools
# RUN sudo ln -s /usr/share/pyshared/lsb_release.py /usr/local/lib/python3.6/dist-packages/lsb_release.py

COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
