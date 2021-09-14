# Slackbot

This document contains a guide to setting up and running the Slackbot that the wandering robot uses to ask for help.

## Configuring the Remote Machine

First, you need a remote server to run the Slackbot on. We use an AWS EC2 server, configured [following the instructions here](https://www.codementor.io/@jqn/deploy-a-flask-app-on-aws-ec2-13hp1ilqy2). We deviate from those instructions in the following ways:
- We set up an Ubuntu 20.04 machine.
- In addition to the two security groups listed, add two “Custom TCP Rule”, one for port 8194 and one for post 8193, with “Source” set to “Custom” and “0.0.0.0/0” (or whatever ports you specify in the config file).
- Instead of installing `python-pip` we installed `python3-pip`
- In all instructions that used `pip`, we used `pip3` instead.
- We installed this additional system package: `sudo apt-get install python3-opencv`
- We installed the following additional python packages: `sudo pip3 install slack_bolt`, `sudo pip3 install opencv-python`

Note that you will also likely have to configure GitHub ssh access on the remote computer. We recommend [following the instructions here](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh).

Then clone this Github repository.

Before we run the app, we must populate `kuri_wandering_robot/slackbot/cfg/slackbot.yaml` with the appropriate Slack App credentials (below).

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
10. Copy the “User OAuth Token” and the “Bot User OAuth Token”  from the “Install App” page, and the “Signing Secret” from the “Basic Information” page, and paste them into their respective fields (lines 37-39) in `kuri_wandering_robot/slackbot/scripts/create_slackbot_config.py`, and run it to write a new `kuri_wandering_robot/slackbot/cfg/slackbot.yaml` file.
11. Run the slackbot python code, by navigating to `kuri_wandering_robot/slackbot/app` and running `python3 run.py`.
12. Go back to the app configuration page, to **Event Subscriptions**:
    - Toggle “Enable Events” on, and enter the request URL as `http://{url_to_the_remote_machine}:8193/slack/events`. (Note that for local development, you can [use an ngrok URL](https://ngrok.com/docs) instead of the aforementioned remote server.) This will send a message to the URL to verify it, which is why it is important to run the code with the correct tokens before this step.
    - Under “Subscribe to Bot Events,” add `message.im`, `reaction_added`, `reaction_removed`
13. **Interactivity & Shortcuts**: Paste the same URL as you did in “Event Subscriptions”
14. Make sure all your changes are saved, and you should be good to go! :)

## Generating the Configuration File

For the Slackbot to work, it requires a configuration file which specifies which users it should interact with, as well as their roles.

### Getting a List of Users

Navigate to `kuri_wandering_robot/slackbot/scripts` and run `python3 get_all_users.py`. This script will read the slackbot config in `kuri_wandering_robot/slackbot/cfg/slackbot.yaml` and print out a list of all the users in your Slack workspace. Find the users who you want the Slackbot to send messages to (you can find them using the `'real_name'` field of the output), and copy/paste their `'id'` into a location you'll be able to access in the next step.

Note that if there are too many users in your Slack Workspace to easily sort through, you can specify the first name, last name, and email of the users in the `desired_users` list (line 15) and the script will only print out Slack profiles for those users.

### Generating the Configuration File

Paste the user IDs you copied above into `kuri_wandering_robot/slackbot/scripts/create_slackbot_config.py` (lines 44-48), and specify what type of help messages that user should receive. There are currently two types of help messages: a) *Low Battery*: which is a message that indicates the robot has low battery and asks the recipient to charge it; and b) *Where Am I*: which is a message that asks the recipient to tell the robot where it is. Note that multiple users can the same type of helper, and the Slackbot will send them all the help request. See more details in the docstring of `User.__init__` in that file.

## Running the Slackbot

On the remote machine, navigate to the `kuri_wandering_robot/slackbot/app` directory and run `python3 run.py`.

### Test Client

To test the server, open another terminal session on the same computer as the server, navigate to `kuri_wandering_robot/slackbot/scripts`, and run `python3 test_client.py` (note, you can run it from a different computer as well by modifying the `server_url` in `test_client.py`). This should test the server's `low_battery` endpoint (with and without an image), its `where_am_i` endpoint, and its `get_updates` endpoint. This test requires you to manually verify that the outputs from the endpoints are as expected (although if you receive the Slack messages and are able to interact with them as expected, then it is working).

### Robot Client

Refer to the [readme in the kuri_wandering_robot directory](./kuri_wandering_robot/README.md).
