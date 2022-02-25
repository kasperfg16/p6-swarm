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

    Launch the project:

    ``` bash
    ros2 launch omni_featter_gazebo omni_featter_gazebo.launch.py
    ```

    When closing close gazebo as it is still running in the background

    ``` bash
    killall gzserver
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
