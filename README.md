Mummer Navigation
=================

Install Notes
-------------

1. Get dependencies:
   ```
      # apt-get update && apt-get install -y ros-kinetic-tf ros-kinetic-move-base \
    ros-kinetic-costmap-converter ros-kinetic-interactive-markers \
    ros-kinetic-tf-conversions ros-kinetic-libg2o ros-kinetic-rosbridge-server \
    ros-kinetic-map-server ros-kinetic-tf2-web-republisher ros-kinetic-global-planner
   ```
2. In a catkin workspace:
   ```
    $ git clone git@protolab.aldebaran.com:mummer/mummer_navigation.git && \
    git clone  https://github.com/guilhembn/tf_hanp.git && \
    git clone  git@protolab.aldebaran.com:mummer/laas_navigation.git && \
    git clone -b time-to-goal git@protolab.aldebaran.com:mummer/teb_local_planner.git && \
    git clone -b time-to-goal https://github.com/harmishhk/hanp_msgs.git && \
    git clone https://github.com/harmishhk/hanp_prediction.git && \
    git clone https://github.com/LAAS-HRI/perspectives_msgs.git
   ```
3. Then:
   ```
   $ catkin build
   ```

Usage
-----

With the right catkin devel/setup sourced:
  `$ roslaunch mummer_navigation mummer_navigation.launch`

You can also download the standalone docker image here: https://github.com/LAAS-HRI/mummer\_integration.git
