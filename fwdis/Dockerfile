FROM kazukitakahashi/ros_mpc

WORKDIR /root/catkin_ws/src

RUN git clone https://github.com/amslabtech/project_high_agility --depth=1

WORKDIR /root/catkin_ws

RUN /bin/bash -c "source /root/catkin_ws/devel/setup.bash && catkin_make"

WORKDIR /root
