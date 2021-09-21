# Kuri Wandering Robot (Executive Node)

This directory contains the central executive node and launchfile for the Kuri Wandering Robot system.

## Configuring the Node

In `kuri_wandering_robot/kuri_wandering_robot/cfg/kuri_wandering_robot_params.yaml`, change the `slackbot_url` rosparam to the url for the remote server that you are running the slackbot on (be sure to point to the port that the Flask server is running on, 8194 by default).

## Running the Node

### Docker

If you are running the code with Docker (see our [wiki entry on using Docker with Kuri](https://github.com/hcrlab/wiki/wiki/Robots:-Kuri:-Docker)), first build the Docker container. Then, run it with the following (NOTE: `my-image` will have to change, and the paths to the workspace might have to as well).

```
sudo docker run --rm --network host -v /home/mayfield/workspaces/my_workspace:/workspace -it my-image
```

Note that we ran the code on three different Kuri's and noticed the need for slightly different Dockerfiles for each. Specifically, the two changes were:
- One of our Kuri's had trouble with `apt update` and needed an up-to-date key. In order to enable that, uncomment lines 3-4 of the Dockerfile and comment line 5.
- One of our Kuri's had an issue with lsb_release between python2 and python3. To fix this, remove the comments on lines 33 and 34.

We would recommend first attempting to use the Dockerfile as-is. If you run into issues, then we would recommend you try the above changes to the Dockerfile before doing further debugging (note that you have to rebuild the image after every modification of the Dockerfile).

Then, build the workspace inside the Docker container, likely using `catkin_make`.

### Network Configuration

Note that you will have to do some network configuration to enable the nodes on the inside and outside of the Docker container to communicate with one another. The script `../env.sh` should help with this. To use it, first run `hostname -I` (either inside or outside the docker container) to determine the index of the target IP you want ROS to use (referred to as `[INDEX]` below). Then, outside the Docker container, first source your workspace (`source devel.setup.bash`) and then source env.sh (`source env.sh [INDEX] core`). Then, inside the Docker container, first source your workspace and then source env.sh as `source env.sh [INDEX]`. This should properly setup your network for the nodes running on the inside and outside of the docker container to communicate with each other. For more details, see our [wiki entry on using Docker with Kuri](https://github.com/hcrlab/wiki/wiki/Robots:-Kuri:-Docker).

### Launching the Nodes

Note that you will need to simultaneously have code running inside and outside the docker container. We recommend using [tmux](https://github.com/tmux/tmux/wiki/Installing) for this, although other valid approaches exist.

In your workspace outside the Docker container, ensure that you have pulled and built the latest files from the [kuri](https://github.com/hcrlab/kuri/tree/melodic-devel) repository, and are in the `melodic_devel` branch. (Note that this branch will still build on ROS Indigo, which is native to Kuri. It is called `melodic_devel` because it is intended to interace with `kuri_wandering_robot`, which is run in ROS Melodic.) Then, run `roslaunch kuri_launch kuri.launch use_camera:=true use_navigation:=false`. Note that this *has* to be run outside the container in order to communicate eith Kuri's hardware and sensors.

Inside the Docker container, run `roslaunch kuri_wandering_robot kuri_wandering_robot.launch`.

## How the Node Works

This node has two states: NORMAL and CHARGING:

- NORMAL toggles on the `wandering_behavior` action server and opens Kuri's eyes. The robot stays in NORMAL until it detects that it is connected to its charger and its battery is lower than `to_charge_threshold` (a configurable parameter -- see `./cfg/kuri_wandering_robot_params.yaml`). It then switches to CHARGING.
- CHARGING closes Kuri's eyes, toggles off the the `wandering_behavior`, and continually checks the battery status. If the battery is either above `charging_done_threshold` (another configurable parameter) or the robot is moved off of its charger, it transitions back to NORMAL.

(Note that in order to connect Kuri to its charger, the helper may have to hold Kuri in place, pushing it into its charger, for a few seconds. They will know Kuri is charging onces its wheels stop moving and its eyes close.)

Note that these states can be extended for domain-specific purposes. For example, in our deployment, we added three additional states related to taking images and sending them to the user.

### Communicating with the Slackbot

The robot continually checks its power level and, if it dips below one of the thresholds specified in the `battery_notification_thresholds` parameter, it sends a low battery alert to the Slackbot, which sends it to the designated low battery helpers.

Further, to illustrate the sample where_am_i help message, this code subscribes to a dummy node, `/where_am_i_help` topic. Sending an Empty message to that topic (e.g., `rostopic pub where_am_i_help std_msgs/Empty "{}"`) will lead Kuri to notify that Slackbot, which will send the message to the designated helpers. Kuri will also ping the Slackbot for updates and print any user responses to console. Note that this message is merely meant to illustrate some types of interactivity that can be achieved with remote user interactions. If users want this exact help message to be autonomous, they will have to develop the autonomous trigger for this help message and enable to robot to incorporate user responses in its decision-making.
