# Kuri Wandering Robot

This repository contains the code associated with the below paper. Please cite the paper if you use the code.

*Nanavati, A., Walker, N., Taber, L., Mavrogiannis, C., Takayama, L., Cakmak, M., & Srinivasa, S. (2022, March). Not All Who Wander Are Lost: A Localization-Free System for In-the-Wild Mobile Robot Deployments. In Proceedings of the 2022 ACM/IEEE International Conference on Human-Robot Interaction.*

The code is broadly divided into three components: motion (`wandering_behavior`), remote user interaction (`slackbot`), and the executive node that integrates the two (`kuri_wandering_robot`).

## Getting Started

### Hardware Requirements
- A Kuri robot. The decision-making and and interaction components are robot agnostic (as of now), but the navigation component is robot-specific.
- A remote computer, configured to enable a Flask app to run on it on ports 8193 and 8194 (instructions under "Slackbot" below). This computer should always be on, to recieve user responses to Slack messages.

### Running Code On-Board the Robot: Dockerfiles

Both the `wandering_behavior` and the `kuri_wandering_robot` directories use Docker containers to isolate ROS versions and ensure the appropriate dependencies are installed. The appropriate Dockerfiles are included in the directories. To install Docker on Kuri, first configure the amd64 Docker PPA following [the latest instructions](https://docs.docker.com/engine/install/ubuntu/). Then run:
```
sudo apt-get update
sudo apt-get install docker-ce
```

Images take a lot of space, so move /var/lib/docker to Kuri's microSD card and symlink the data back into place:
```
sudo service docker stop
sudo umount /var/lib/docker
sudo mv /var/lib/docker /mayfield/data/docker
sudo ln -s /mayfield/data/docker /var/lib/docker
sudo service docker start
```

To build a docker image, cd to the appropriate `docker` directory and run:
```
sudo docker build -t my-image .
```

To run a docker image, from any location run:

```
sudo docker run --rm --network host -v /home/mayfield/workspaces/my_workspace:/workspace -it my-image
```

Note that Kuri still needs its default code running outside the docker image, to communicate with its motors/sensors. Therefore, you should have one workspace that you compile outside the docker image (in ROS Indigo) with [the default kuri code (melodic-devel branch)](https://github.com/hcrlab/kuri/tree/melodic-devel). You should have another workspace that you compile within the docker image (in ROS Melodic) with this repository. Then, to run any code within a docker image on Kuri, outside the docker image run the default kuri launchfile, and inside the docker image run this code. More details can be found in the [readme in the kuri_wandering_robot directory](./kuri_wandering_robot/README.md).

NOTE: For running the overall system, use the Dockerfile in `kuri_wandering_robot`. The Dockerfile in `wandering_behavior` should chiefly be used to run that node in isolation.

### Slackbot

Refer to the [readme in the slackbot directory](./slackbot/README.md). This code must run on a remote computer, so that it is guaranteed to be always on.

### Wandering Behavior

Refer to the [readme in the wandering_behavior directory](./wandering_behavior/README.md).

### Kuri Wandering Robot

This node is the executive node; it brings together the slackbot and navigation, and decision-making aspects of this system. Refer to the [readme in the kuri_wandering_robot directory](./kuri_wandering_robot/README.md).

## Deployment Code: Robot Photographer

As is mentioned in the paper, for the four-day deployment we extended this system to a robot photographer. That version of the code can be found in [this commit](https://github.com/hcrlab/kuri_wandering_robot/tree/1888a45f0c5b896d6c79a2e8ad1174b546be2175).
