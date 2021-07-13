# Slackbot

This document contains a guide to setting up and running the Slackbot used in the UW+UCSC CMM Demo.

## Configuring the Remote Machine

First, you need a remote server to run the Slackbot on. We use an AWS EC2 server, configured [following the instructions here](https://www.codementor.io/@jqn/deploy-a-flask-app-on-aws-ec2-13hp1ilqy2). We deviate from those instructions in the following ways:
- We set up an Ubuntu 20.04 machine.
- In addition to the two security groups listed, add two “Custom TCP Rule”, one for port 8194 and one for post 8193, with “Source” set to “Custom” and “0.0.0.0/0”.
  - We also used ports 3000 and 3001 for the debug (non-deployment) mode. This can be beneficial if you have multiple Kuri's running at once and don't want them to interfere in each others' Slackbots, but may not be necessary otherwise.
- Instead of installing `python-pip` we installed `python3-pip`
- In all instructions that used `pip`, we used `pip3` instead.
- We installed this additional system package: `sudo apt-get install python3-opencv`
- We installed the following additional python packages: `sudo pip3 install slack_bolt`, `sudo pip3 install opencv-python`

Note that you will also likely have to configure GitHub ssh access on the remote computer. We recommend [following the instructions here](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh).

Then run the following:

```
git clone git@github.com:hcrlab/kuri_cmm_demo.git
cd kuri_cmm_demo/slackbot/cfg
cp slackbot_template.yaml slackbot.yaml
```

Before we run the app, we must populate `kuri_cmm_demo/slackbot/cfg/slackbot.yaml` with the appropriate Slack App credentials (below).

## Setting Up a Slack App

### Prerequisites

- You should have [a Slack workspace](https://slack.com/help/articles/206845317-Create-a-Slack-workspace) and a login to the workspace. (You [do not need to be](https://slack.com/help/articles/201314026-Permissions-by-role-in-Slack) an owner/administrator to create/install a Slackbot.)

### Steps

1. Go to https://api.slack.com/apps, and sign in to the account you use with the workspace you’d like to install the Slackbot in.
    - Note: after logging in, it might automatically take you to your workspace. In that case, go to https://api.slack.com/apps again.
2. Click “Create New App” (or go to https://api.slack.com/apps/new)
3. Click “From scratch”
4. Enter a name (we used KuriBot), and select the workspace you want to develop the app in.
5. You should now be on the main app configuration page. You should see a left sidebar with several options, such as “Basic Information,” “Collaborators,” etc. We will go through the important ones here. For most steps, you will have to click the green “Save Changes” button at the bottom-right before proceeding.
6. **Basic Information**: scroll down to “Display Information” and configure the robot’s name, description, and icon. We used:
    - App Name: KuriBot
    - Short Description: This bot is how Kuri communicates with you! If it wants to share pictures with you, it will do so using this bot.
    - App Icon: [KuriBot_icon.jpg](./imgs/KuriBot_icon.jpg)
7. **Collaborators**: Add any additional member of the workspace who should have edit access to the app.
8. **OAuth & Permissions**:
    - Under “Bot Token Scopes” grant the app the following. `channels:read`, `chat:write`, `files:read`, `files:write`, `im:history`, `im:read`, `reactions:read`, `users:read`
    - Under “User Token Scopes” grant the app the following. `chat:write`, `files:write`, `users:read`, `users:read.email`
    - Note: the permissions in this readme might be a superset of the needed permissions.
9. **Install App**: Click  “Install to Workplace”
10. Copy the “User OAuth Token” and the “Bot User OAuth Token”  from the “Install App” page, and the “Signing Secret” from the “Basic Information” page, and paste them into their respective fields (lines 123-125) in `kuri_cmm_demo/slackbot/scripts/create_slackbot_config.py`, and run it. Then, rename the generated file (`kuri_cmm_demo/slackbot/cfg/slackbot_autogen.yaml`) to `kuri_cmm_demo/slackbot/cfg/slackbot.yaml`
11. Run the slackbot python code, by navigating to `kuri_cmm_demo/slackbot/app` and running `python3 run.py`. Make sure `is_deployed` is set to True (line 914).
12. Go back to the app configuration page, to **Event Subscriptions**:
    - Toggle “Enable Events” on, and enter the request URL as `http://{url_to_the_remote_machine}:8193/slack/events`. (Note that for local development, you can [use an ngrok URL](https://ngrok.com/docs) instead of the aforementioned remote server.) This will send a message to the URL to verify it, which is why it is important to run the code with the correct tokens before this step.
    - Under “Subscribe to Bot Events,” add `message.im`, `reaction_added`, `reaction_removed`
13. **Interactivity & Shortcuts**: Paste the same URL as you did in “Event Subscriptions”
14. **Slash Commands**: Slash commands can be useful to debug the Slackbot's message designs. We use multiple types of slash commands, detailed below. For every slash command, the URL is the same as the one used above. Note that several of these messages are specific to the user study, and may not be used in a deployment that is not in the context of a user study.
  - `test_pre_study_message`, `test_daily_intro_0`, `test_daily_intro_1`, `test_daily_intro_2`: Introductory messages that were sent before the user study started, and at the start of each of the three days of the user study.
  - `test_get_images`: This tests the way in which the Slackbot gets images and sends them to the user. Note that you need a client running to send images to the Slackbot -- either a Kuri/robot running the `kuri_cmm_demo_node.py`, or the `stored_image_client.py`.
  - `test_end_of_day_message_0`, `test_end_of_day_message_1`, `test_end_of_day_message_2_0`, `test_end_of_day_message_2_1`: Messages that were sent at the end of days 1-3 of the user study, as well as the survey that was sent half-way through Day 2.
15. **App Home**: Check the box that says "Allow users to send Slash commands and messages from the messages tab"
16. Make sure all your changes are saved, and you should be good to go! :)

## Generating the Configuration File

For the Slackbot to work, it requires a configuration file which specifies which users it should interact with, as well as their roles.

### Getting a List of Users

Navigate to `kuri_cmm_demo/slackbot/scripts` and run `python3 get_all_users.py`. This script will read the slackbot config in `kuri_cmm_demo/slackbot/cfg/slackbot.yaml` and print out a list of all the users in your Slack workspace. Find the users who you want the Slackbot to send messages to (you can find them using the `'real_name'` field of the output), and copy/paste their `'id'` into a location you'll be able to access in the next step.

Note that if there are too many users in your Slack Workspace to easily sort through, you can specify the first name, last name, and email of the users in the `desired_users` list (line 15) and the script will only print out Slack profiles for those users.

### Generating the Configuration File

Paste the user IDs you copied above into `kuri_cmm_demo/slackbot/scripts/create_slackbot_config.py` (lines 127-134). There are two user roles: a) *Low Battery Savior*: who is the person who will get messaged when the robot is low on battery; and b) *Image Receiver*: which is the standard role of a user who receives and responds to images. There are several ways to configure the users who will be receiving images. If you want them to receive scheduled messages over multiple days, you want to configure it with a string that corresponds to the first date of scheduled messages. If you want them to receive scheduled messages on another interval, specify the epoch of the first message and the interval (num secs) between messages. Otherwise, if you don't want them to receive scheduled messages (e.g., you just want to use the slack commands specified above), then configure it with None instead of a timestamp/date. See more details in the docstring of `User.__init__` in that file.

NOTE: The current `kuri_cmm_demo/slackbot/cfg/slackbot_template.yaml` is what is outputted by `kuri_cmm_demo/slackbot/scripts/create_slackbot_config.py` as-is.

## Running the Slackbot

On the remote machine, navigate to the `kuri_cmm_demo/slackbot/app` directory and run `python3 run.py`.

### Test Client

To test the server, open another terminal session on the same computer as the server, navigate to `kuri_cmm_demo/slackbot/scripts`, and run `python3 test_client.py` (note, you can run it from a different computer as well by modifying the `server_url` in `test_client.py`). This should test the server's `get_num_users` endpoint, its `send_image` endpoint (which will send an image to user who you run the `/test_get_images` slash command on), sleep for 10 seconds to enable the user to respond to the message, and then test the `get_responses` endpoint. Note that this test requires you to manually verify that the outputs from the endpoints are as expected.

### Stored Image Client

Unlike `test_client.py`, which only runs once, `stored_image_client.py` runs a loop and keeps sending images to the Slackbor and getting updates from the Slackbot. As such, it is closer to the actual robotic system (except the images are pre-stored images, and it does not learn from the human responses).

To run the stored image client, you need a folder of jpg images to send. Then, modify `kuri_cmm_demo/slackbot/scripts/stored_image_client.py` lines 27-28 to specify the path of that folder, and the server URL that the slackbot will be running on. Then, run `python3 stored_image_client.py`. This should run the stored image client. When images are requested (either due to the `/test_get_images` slash command or based on a schedule specified in the configuration file), the client will send 5 random images from that folder.

### Robot Client

Refer to the [readme in the kuri_cmm_demo directory](./kuri_cmm_demo/README.md).
