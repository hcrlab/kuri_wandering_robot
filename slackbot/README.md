# Slackbot

This document contains a guide to setting up and running the Slackbot used in the UW+UCSC CMM Demo.

## Configuring the Remote Machine

First, you need a remote server to run the Slackbot on. We use an AWS EC2 server, configured [following the instructions here](https://www.codementor.io/@jqn/deploy-a-flask-app-on-aws-ec2-13hp1ilqy2). We deviate from those instructions in the following ways:
- We set up an Ubuntu 20.04 machine.
- In addition to the two security groups listed, add two “Custom TCP Rule”, one for port 8194 and one for post 8193, with “Source” set to “Custom” and “0.0.0.0/0”.
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
    - App Icon: [KuriBot_icon.jpg](./img/KuriBot_icon.jpg)
7. **Collaborators**: Add any additional member of the workspace who should have edit access to the app.
8. **OAuth & Permissions**:
    - Under “Bot Token Scopes” grant the app the following. `channels:read`, `chat:write`, `files:read`, `files:write`, `im:history`, `im:read`, `reactions:read`, `users:read`
    - Under “User Token Scopes” grant the app the following. `chat:write`, `files:write`
    - Note: the permissions in this readme might be a superset of the needed permissions.
9. **Install App**: Click  “Install to Workplace”
10. Copy the “User OAuth Token” and the “Bot User OAuth Token”  from the “Install App” page, and the “Signing Secret” from the “Basic Information” page, and paste them into their respective fields in `kuri_cmm_demo/slackbot/cfg/slackbot.yaml`.
11. Run the slackbot python code, by navigating to `kuri_cmm_demo/slackbot/app` and running `python3 run.py`.
12. Go back to the app configuration page, to **Event Subscriptions**:
    - Toggle “Enable Events” on, and enter the request URL as `http://{url_to_the_remote_machine}:8193/slack/events`. (Note that for local development, you can [use an ngrok URL](https://ngrok.com/docs) instead of the aforementioned remote server.) This will send a message to the URL to verify it, which is why it is important to run the code with the correct tokens before this step.
    - Under “Subscribe to Bot Events,” add `message.im`, `reaction_added`, `reaction_removed`
13. **Interactivity & Shortcuts**: Paste the same URL as you did in “Event Subscriptions”
14. Make sure all your changes are saved, and you should be good to go! :)

## Getting a List of Users

Navigate to `kuri_cmm_demo/slackbot/scripts` and run `python3 get_all_users.py`. This script should print out a list of all the users in your Slack workspace. Find the users who you want the Slackbot to send messages to (you can find them using the `'real_name'` field of the output), and copy/paste their `'id'` into the `users_list` component of `kuri_cmm_demo/slackbot/cfg/slackbot.yaml`. (NOTE: for `test_client.py` below, it would help if your ID is the first one in the list).

## Running the Slackbot

On the remote machine, navigate to the `kuri_cmm_demo/slackbot/app` directory and run `python3 run.py`.

To test it, open another terminal session on this computer, navigate to `kuri_cmm_demo/slackbot/scripts`, and run `python3 test_client.py` (note, you can run it from a remote computer as well by modifying the `server_url` in `test_client.pu`). This should test the server's `get_num_users` endpoint, its `send_image` endpoint (which will send an image to the first user in your list), sleep for 10 seconds to enable the user to respond to the message, and then test the `get_responses` endpoint. Note that this test requires you to manually verify that the outputs from the endpoints are as expected.

## Notes
- This Slackbot has only been tested with one user in the user list as of now.
- TODO: add Slash Commands to the slackbot permission configuration section. And App Home.
