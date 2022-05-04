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

    e)

    follow these intructions:
    <https://github.com/kajMork/Brick_Feeder/wiki>

## How to run in simulation

1. Open a new terminal in root

    a)

    Launch gazebo, rviz and the NAV2 navigation stack:

    ``` bash
    ros2 launch process_robot_gazebo gazebo.launch.py 
    ```

2. Open a new terminal in root

    a)

    Launch gazebo, rviz and the NAV2 navigation stack:

    ``` bash
    ros2 launch process_robot_navigation navigation_gazebo.launch.py
    ```

3. Open a new terminal

    a)

    Run plotjuggler to

    ``` bash
    ros2 run plotjuggler plotjuggler
    ```

## How to create a map with single real robot

1. Run script on jetson

    SSH onto a jetson nano on a robot
    run the command:

    ``` bash
    /usr/bin/python3 ResetTeensy.py
    ```

2. Open a new terminal in root

    a)

    Launch the rviz and joint state publisher

    ``` bash
    ros2 launch process_robot_bringup bringup.launch.py 
    ```

3. Open a new terminal in root

    a)

    Launch NAV2 navigation stack:

    ``` bash
    ros2 launch process_robot_navigation navigation_real_robot.launch.py
    ```

4. Open a new terminal in root

    a)

    Launch a static transform publisher, that publishes transform between "map" and "odom" frame

    ``` bash
    ros2 run tf2_ros static_transform_publisher -2.5 -2.5 0 0 0 0 odom map
    ```

5. Open a new terminal in root

    a)

    Launch the robot commander

    ``` bash
    ros2 launch process_robot_commander robot_commander.launch.py
    ```

## How to run Nav2 on single real robot

1. Open a new terminal in root

    a)

    SSH onto the computer on the robot e.g. via the use of putty SSH client: <https://itsfoss.com/putty-linux/>

    Then run the following command
    ``` bash
    python3 Reset_Teensy
    ```

2. Open a new terminal in root

    a)

    Launch the rviz and joint state publisher

    ``` bash
    ros2 launch process_robot_bringup bringup.launch.py 
    ```

3. Open a new terminal in root

    a)

    Launch NAV2 navigation stack:

    ``` bash
    ros2 launch process_robot_navigation navigation_real_robot.launch.py
    ```

4. Open a new terminal in root

    a)

    Launch a static transform publisher, that publishes transform between "map" and "odom" frame

    ``` bash
    ros2 run tf2_ros static_transform_publisher -2.5 -2.5 0 0 0 0 odom map
    ```

5. Open a new terminal in root

    a)

    Start an agent to the ESP32

    ``` bash
    sudo docker run -it --rm --net=host microros/micro-ros-agent:galactic udp4 --port 8888 -v6
    ```

6. Open a new terminal in root

    a)

    Launch a static transform publisher, that publishes transform between "map" and "odom" frame

    ``` bash
    ros2 launch process_robot_commander robot_commander.launch.py
    ```

7. (Suggestion) Open a new terminal

    a)

    install plotjuggler:
    <https://snapcraft.io/install/plotjuggler/ubuntu>

    b)

    Run plotjuggler to visualize data. Plotjuggler FTW!!

    ``` bash
    ros2 run plotjuggler plotjuggler
    ```

## Rules

* All files that only relates to your own pc should never be included in commits, make sure to add them to gitignore!.
* All custom environments should be added to gitignore.
* If you did not create the branch (or if it the main branch), please to not make direct commits, only pull requests.
* The code sholud follow the topolegy overview

## Links

## Related/inspiration projects

<https://automaticaddison.com/how-to-create-a-simulated-mobile-robot-in-ros-2-using-urdf/>

## Topology overview
