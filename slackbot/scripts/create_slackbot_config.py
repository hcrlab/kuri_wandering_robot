#!/usr/bin/python3
import os
import time
import yaml

class User(object):
    def __init__(self, user_id, low_battery_helper, where_am_i_helper):
        """
        Stores user-specific data.

        :param user_id: (str) the user's Slack ID
        :param low_battery_helper: (bool) whether they should receive the "low
                                   battery" help messages
        :param where_am_i_helper: (bool) whether they should receive the
                                  "where am I" help messages
        :returns: an instance of the User class
        """
        self.user_id = user_id
        self.low_battery_helper = low_battery_helper
        self.where_am_i_helper = where_am_i_helper

    def to_dict(self):
        """
        Convert the User to a dict to be written to yaml.
        """
        return {
            self.user_id : {
                "low_battery_helper" : self.low_battery_helper,
                "where_am_i_helper" : self.where_am_i_helper,
            }
        }

if __name__ == "__main__":
    ############################################################################
    # Parameters To Set
    ############################################################################
    slack_bot_token = "TEST_SLACK_BOT_TOKEN"
    slack_user_token = "TEST_SLACK_USER_TOKEN"
    slack_signing_secret = "TEST_SLACK_SIGNING_SECRET"

    flask_port = 8194 # where the Flask server will listen for HTTP requests
    slack_port = 8193 # where the Slackbot will listen for updates to messages

    users = [
        User("TEST_USER_1", True, False),
        User("TEST_USER_2", True, True),
        User("TEST_USER_3", False, True),
    ]

    ############################################################################
    # Save the file
    ############################################################################
    # Get the filepath
    default_filepath = "../cfg/slackbot.yaml"
    filepath = input("Enter Filepath (default: %s): " % default_filepath)
    if len(filepath) == 0:
        filepath = default_filepath

    # Check if the path exists
    if os.path.isfile(filepath):
        override_str = input("Warning: Filepath %s already exists. Are you " \
        "sure you would like to override it? (Y/n) " % filepath)
        override_bool = (override_str[0] == "Y")
        if not override_bool:
            print("Will not override file. Exiting script.")
            quit()

    # Write the config file
    with open(filepath, 'w') as f:
        yaml_dict = {
            "slack_bot_token" : slack_bot_token,
            "slack_user_token" : slack_user_token,
            "slack_signing_secret" : slack_signing_secret,
            "flask_port" : flask_port,
            "slack_port" : slack_port,
            "users_list" : [user.user_id for user in users]
        }
        for user in users:
            yaml_dict.update(user.to_dict())
        yaml.dump(yaml_dict, f, sort_keys=False)

    print("Success! Wrote config file to %s" % filepath)
