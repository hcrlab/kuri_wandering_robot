# Kuri CMM Demo (Decision-Making Node)

This directory contains the decision-making code (i.e., who to send what image(s) to), as well as well as the central executive node and launchfile for this demo. Not that the current version of this node is robot agnostic, and can be run as long as users provide it a CompressedImage stream to subscribe to.

## Configuring AWS Rekognition

We recommend users follow Steps 1 & 2 in [Getting Started with AWS Rekognition](https://docs.aws.amazon.com/rekognition/latest/dg/getting-started.html) to configure their AWS account.

## Configuring the Node

In `kuri_cmm_demo/kuri_cmm_demo/launch/kuri_cmm_demo.launch`, change the `slackbot_url` rosparam to the url for the remote server that you are running the slackbot on (be sure to point to port 8194 on that server).

In the same launchfile, set the `aws_region_name` and `aws_profile_name` to the values you configured in Step 2 of [Getting Started with AWS Rekognition](https://docs.aws.amazon.com/rekognition/latest/dg/getting-started.html).

## Running the Node

If you are running the code with Docker (see our [wiki entry on using Docker with Kuri](https://github.com/hcrlab/wiki/wiki/Robots:-Kuri:-Docker)), first build the Docker container. Then, run it with the following (NOTE: `my-image` will have to change, and the paths to the workspaces might have to as well).

```
sudo docker run --rm --network host -v /home/mayfield/workspaces/my_workspace:/workspace -v /home/mayfield/.aws:/root/.aws -it my-image
```

Then, build the workspace. Then, run `roslaunch kuri_cmm_demo kuri_cmm_demo.launch`.

In the current version of the code, starting the launchfile will lead to the following:
- Subscribing to Kuri's camera feed and detecting objects in a subsampled stream of those images.
- Using the decision-making policy to select who (if anyone) to send the images to.
- Sending those images to the Slackbot and receiving its responses.

Note that this node does not currently govern motion of the robot base. Users can either [teleoperate Kuri around](https://github.com/hcrlab/kuri/tree/master/kuri_teleop), or run `../local_coverage_navigation`.
