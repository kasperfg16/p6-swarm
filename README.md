# p6-swarm

Github for 6th semester project at AAU.

Project report is found here: p6-swarm/P6_Bachelor_projekt.pdf

## Recommmendations

1. We recommend to read the project report for an understanding of the system.

2. This project is using ROS2 galactic on ubuntu 20.04. Other configurations are used at own dispare and misery

3. We recommend using dualboot via USB: <https://www.youtube.com/watch?v=cHF1ByFKtZo&t=315s>. In this way you can transfer all files between systems on the go.

## Prerequisites

1. Install python

    <https://www.python.org/downloads/>

2. Install ROS2 galaxtic:

    <https://docs.ros.org/en/galactic/Installation/Ubuntu-Development-Setup.html>

3. Install pip

    <https://www.geeksforgeeks.org/how-to-install-pip-on-windows/>

4. In a terminal

    Install dependecies, extra ros-packeages etc.

    ``` bash
    pip install -r pip install -r requirements.txt 
    ```

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

    ``` bash
    sudo apt install ros-galactic-gazebo-ros-pkgs
    ```

## How to setup

1. In a terminal:

    a)

    Create a workspace folder with a **PATH** of your choice. Remember/write down the **PATH** for later:

    ``` bash
    mkdir PATH
    ```

    b)

    Clone the reposetory:

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

    In this command remember to change **"PATH"** to the **PATH** where you cloned to reposetory to:

    ``` bash
    echo 'source PATH/install/setup.bash' >> ~/.bashrc
    ```

    e)

    Follow these intructions:
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

3. Open a new terminal in root

    a)

    Now you can click the Nav2 Goal button:

    ![plot](arrow_rviz.png)

    Then click on the map to make the robot move to a certain pose on the map:

4. Open a new terminal

    a)

    Run plotjuggler to

    ``` bash
    ros2 run plotjuggler plotjuggler
    ```

## How to run Nav2 on single real robot

1. Go to <https://github.com/kajMork/linorobot2_hardware/blob/master/README.md> and follow the instructions in the README.md

1. Go to the <https://github.com/BenMusak/docking_action_server> reposetory

1. Open a new terminal in root

    a)

    SSH onto the robot computer e.g. via the use of putty SSH client: <https://itsfoss.com/putty-linux/>

    Then run the following command

    ``` bash
    python3 Reset_Teensy
    ```

2. Open a new terminal in root

    a)

    Launch a static transform publisher, that publishes transform between "map" and "odom" frame in meters

    NOTE: The values in the command below must be chnaged to your application. The tranform is x y z roll pitch yaw

    ``` bash
    ros2 run tf2_ros static_transform_publisher -1.853 -4.045 0 0 0 0 odom map
    ```

3. Open a new terminal in root

    a)

    Launch the rviz and joint state publisher

    ``` bash
    ros2 launch process_robot_bringup bringup.launch.py 
    ```

4. Open a new terminal i<https://github.com/BenMusak/ROB_vis_aruco>

    a)

    Launch NAV2 navigation stack:

    ``` bash
    ros2 launch process_robot_navigation navigation_real_robot.launch.py
    ```

5. Follow the readme of <https://github.com/BenMusak/ROB_vis_aruco>

6. Open a new terminal in root

    a)

    Start an agent to the ESP32

    ``` bash
    sudo docker run -it --rm --net=host microros/micro-ros-agent:galactic udp4 --port 8888 -v6
    ```

7. Open a new terminal in root

    a)

    Launch a static transform publisher, that publishes transform between "map" and "odom" frame

    ``` bash
    ros2 launch process_robot_commander robot_commander.launch.py
    ```

8. (Suggestion) Open a new terminal

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
