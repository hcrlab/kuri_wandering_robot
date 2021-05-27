from slack_bolt import App
import yaml

# Load the Slackbot configuration
slackbot_conf_filepath = "../cfg/slackbot.yaml"
with open(slackbot_conf_filepath, 'r') as f:
    slackbot_conf = yaml.load(f, Loader=yaml.FullLoader)

# Launch the Slack App
slack_app = App(
    token=slackbot_conf['slack_bot_token'],
    signing_secret=slackbot_conf['slack_signing_secret']
)

# Request all the users
request = slack_app.client.api_call("users.list")
if request['ok']:
    print('request', request)
    for item in request['members']:
        print(item)
else:
    print("request", request)
