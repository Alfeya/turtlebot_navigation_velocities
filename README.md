turtlebot_navigation
====================
To run the program the following steps in strict order.

1) Run turtlebot simulation
    
    roslaunch turtlebot_gazebo turtlebot_playground.launch

2) Run slam map generation

    a) edit file playground.yaml, set your full path to playground.pgm file
    
    b) roslaunch turtlebot_gazebo amcl_demo.launch map_file:=<full_path_to_playground.yaml>

3) (optional) Run map visualiser
  
    roslaunch turtlebot_rviz_launchers view_navigation.launch

4) (optional) Run keyboard control of the robot

    roslaunch turtlebot_teleop keyboard_teleop.launch

5) Build and run the program

    catkin_make --pkg turtlebot_navigation_velocities
    rosrun turtlebot_navigation_velocities turtlebot_navigation_velocities

* keyboard teleop program blocks turtlebot_navigation_velocities, so it should be killed before turtlebot_navigation_velocities may work.
