# wandering_behavior

A fork of `move_base` with global planner capabilities removed. Kuri does not use `move_base` and did not ship with the necessary dependencies. Even if it had, `move_base` looks much different in newer releases thanks to TF2 adoption. To make the navigation component easy to develop, easy to deploy and easy to use in other contexts, this code is developed against the Melodic/Noetic releases of ROS Navigation. A Dockerfile is included for running on the robot.

## Usage

Currently, the navigator node engages when launched, navigating the robot around the environment endlessly.

### Simulation

With the `melodic-devel` version of [our Kuri code](https://github.com/hcrlab/kuri/tree/melodic-devel), launch a simulation:

    roslaunch kuri_gazebo kuri_gazebo.launch

Now launch the navigator:

    roslaunch wandering_behavior wandering_behavior.launch

### Robot

Create a new workspace to hold this package and mount it in a Docker container using the image provided in `docker/`:

    sudo docker run --rm --network host -v /home/mayfield/workspaces/melodic_ws:/workspace -it local-nav

In the container, build the workspace with `catkin_make`. Tweak the `ROS_IP` and `ROS_HOSTNAME` variables (in the entrypoint or in the shell) as needed to suit your network, then run the same launch file as used in simulation. Note that the `../env.sh` may help you set the correct networking variables, and is documented in the [readme in the executive_package](../kuri_wandering_robot/README.md). For more information, we'd recommend you read our [wiki entry on using Docker with Kuri](https://github.com/hcrlab/wiki/wiki/Robots:-Kuri:-Docker).
