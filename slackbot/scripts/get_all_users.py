from slack_bolt import App
import yaml

# Load the Slackbot configuration
slackbot_conf_filepath = "../cfg/slackbot.yaml"
with open(slackbot_conf_filepath, 'r') as f:
    slackbot_conf = yaml.load(f, Loader=yaml.FullLoader)

# Launch the Slack App
slack_app = App(
    token=slackbot_conf['slack_user_token'],
    signing_secret=slackbot_conf['slack_signing_secret']
)

desired_users = [
    # ("First Name", "Last Name", "email@email.com"),
]

# Request all the users
found_user_i = []
done = False
next_cursor = ''
while not done:
    response = slack_app.client.users_list(cursor=next_cursor)
    if response['ok']:
        for item in response['members']:
            if len(desired_users) > 0:
                for i in range(len(desired_users)):
                    user = desired_users[i]
                    first_name, last_name, email = user
                    if ('real_name' in item and first_name.lower() in item['real_name'].lower() and last_name.lower() in item['real_name'].lower()) or ('email' in item['profile'] and email.lower() in item['profile']['email'].lower()):
                        print("User %s has Slack Profile %s" % (user, item))
                        found_user_i.append(i)
            else:
                print(item)
        if 'next_cursor' in response['response_metadata'] and len(response['response_metadata']['next_cursor']) > 0:
            done = False
            next_cursor = response['response_metadata']['next_cursor']
        else:
            done = True
    else:
        print("response", response)
        print("Request failed. ")
        break

for k in range(len(desired_users)):
    if k not in found_user_i:
        print("User %s not found" % str(desired_users[k]))
