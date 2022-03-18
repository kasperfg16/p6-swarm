# p6-swarm

github for 6th semester project at AAU.

## Disclaimer

This project is using ROS2 galactic. Other configurations are used at own dispare and misery

## Prerequisites

1.
    Install ROS2 galaxtic:

    <https://docs.ros.org/en/galactic/Installation/Ubuntu-Development-Setup.html>

### ROS packages

Install extra ROS packages

1. In a terminal

    Install packages:

    ``` bash
    sudo apt install ros-galactic-joint-state-publisher-gui
    ```

    ``` bash
    sudo apt install ros-galactic-xacro
    ```

    ``` bash
    sudo apt install ros-galactic-ros2-control
    ```

    ``` bash
    sudo apt install ros-galactic-ros2-controllers
    ```

    ``` bash
    sudo apt install ros-galactic-gazebo-ros-pkgs
    ```

    ``` bash
    sudo apt install ros-galactic-ros-core ros-galactic-geometry2
    ```

    ``` bash
    sudo apt-get install ros-galactic-turtle-tf2-py ros-galactic-tf2-tools ros-galactic-tf-transformations
    ```

    ``` bash
    sudo apt install ros-galactic-robot-localization
    ```

    ``` bash
    sudo apt install ros-galactic-tf2-geometry-msgs
    ```

    ``` bash
    pip install transformations
    ```

## Dependencies

None yet

## How to setup

1. In a terminal:

    a)

    Create a workspace folder with a **PATH** of your choice. Remember/write down the **PATH** for later:

    ``` bash
    mkdir PATH
    ```

    b)

    clone the reposetory:

    ``` bash
    git clone https://github.com/kasperfg16/p6-swarm.git
    ```

    c)

    Go into the workspace folder and build the package:

    ``` bash
    cd PATH
    mkdir config launch maps meshes models params rviz worlds
    colcon build
    ```

    d)

    Add some commands to .bashrc file, so you don't have to source the project every time.

    ``` bash
    echo 'source /opt/ros/galactic/setup.bash' >> ~/.bashrc
    echo 'source ~/ros2_galactic/install/local_setup.bash' >> ~/.bashrc
    ```

    In this command remember to change **PATH**:

    ``` bash
    echo 'source PATH/install/setup.bash' >> ~/.bashrc
    ```

    Install the `gazebo_ros_pkgs` package.

    ``` bash
    sudo apt install ros-galactic-gazebo-ros-pkgs
    ```

## How to run

1. Open a new terminal

    a)

    Launch gazebo and rviz:

    ``` bash
    ros2 launch process_robot_gazebo gazebo.launch.py 
    ```

    b)

    Launch gazebo and rviz:

    ``` bash
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
    ```

2. Open a new terminal

    a)

    ``` bash
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
    ```

3. Open a new terminal

    a)

    ``` bash
    ros2 launch process_robot_navigation navigation_gazebo.launch.py
    ```

4. Open a new terminal

    Now you have to drive the robot around and a map will be drawn. Launch gazebo.launch.py to control the robot in gazebo:

    ``` bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

5. Open a new terminal

    When you have mapped the obstables in the map you save it

    ``` bash
    cd swarm_process_robot/process_robot_navigation/maps
    ```

    ``` bash
    ros2 run nav2_map_server map_saver_cli -t /global_costmap/costmap -f <map_name> --ros-args -p save_map_timeout:=10000
    ```

    Two files which describes the generated map will be created and saved

6. Open a new terminal

    ros2 launch nav2_bringup bringup_launch.py map:=/home/ubuntu/Documents/GitHub/P6-Swarm/p6-swarm/src/swarm_process_robot/process_robot_navigation/maps/swarm_map1.yaml map_type:=occupancy

    When closing close gazebo as it might still be running in the background

    ``` bash
    killall gzserver
    ```

    ros2 run nav2_bt_navigator bt_navigator --ros-args --params-file /home/ubuntu/Documents/GitHub/P6-Swarm/p6-swarm/src/swarm_process_robot/process_robot_navigation/config/navigation_gazebo.yaml

## Rules

* All files that only relates to your own pc should never be included in commits, make sure to add them to gitignore!.
* All custom environments should be added to gitignore.
* If you did not create the branch (or if it the main branch), please to not make direct commits, only pull requests.
* The code sholud follow the topolegy overview

## Links

## Related/inspiration projects

<https://automaticaddison.com/how-to-create-a-simulated-mobile-robot-in-ros-2-using-urdf/>

## Topology overview
