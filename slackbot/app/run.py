import base64
import datetime
from flask import Flask, request
import json
import logging
import os
import random
import requests
from sent_messages_database import SentMessagesDatabase
from slack_bolt import App
from slack_templates import slack_template_1
import string
import sys
import threading
import time
import traceback
import yaml

class FlaskSlackbot(object):
    def __init__(self, slackbot_conf, flask_port=8194, slack_port=8193,
        sent_messages_database_filepath="../cfg/sent_messages_database.pkl",
        database_save_interval=1):
        """
        Create the FlaskSlackbot.

        Inputs:
        - slackbot_conf is a dictionary containing the keys 'users_list',
          'slack_bot_token', 'slack_user_token', and 'slack_signing_secret'.
          'users_list' is a list of strings, and the others are strings.
        - flask_port is the port the Flask server will be launched at
        - slack_port is the port the Slack app will be launched at
        """
        # Create and configure the Flask App
        self.flask_app = Flask("slackbot")
        self.flask_port = flask_port
        self.flask_app.debug = False
        self.flask_app.add_url_rule('/send_image',
            view_func=self.send_image, methods=['POST'])
        self.flask_app.add_url_rule('/get_num_users',
            view_func=self.get_num_users, methods=['GET'])
        self.flask_app.add_url_rule('/get_responses',
            view_func=self.get_responses, methods=['POST'])

        # Create the Slack app
        self.slack_user_token = slackbot_conf['slack_user_token']
        self.slack_app = App(
            token=slackbot_conf['slack_bot_token'],
            signing_secret=slackbot_conf['slack_signing_secret']
        )
        self.slack_port = slack_port
        self.slack_app.action("action_id_check_mark")(self.action_button_check_mark)
        self.slack_app.action("action_id_x")(self.action_button_x)

        # Store the Slack users
        self.users = slackbot_conf['users_list']

        # Keep track of the images that are sent
        self.sent_messages_database_filepath = sent_messages_database_filepath
        self.sent_messages_database = SentMessagesDatabase.load(
            sent_messages_database_filepath)
        self.database_save_interval = database_save_interval
        self.database_updates_since_last_save = 0

    def database_updated(self):
        """
        Called everytime the database is updated. Saves the database every
        self.database_save_interval updates
        """
        self.database_updates_since_last_save += 1
        if self.database_updates_since_last_save % self.database_save_interval == 0:
            self.sent_messages_database.save(self.sent_messages_database_filepath)

    def get_image_url(self, image, message_id):
        """
        Uploads the image to Slack and gets a public URL for it.

        Input:
        - image bytes

        Output:
        - if the image upload is succesful, return a public url (string) that
          points to the image
        - else, return None
        """
        filename = message_id+".jpg"

        # Upload the file
        response = self.slack_app.client.files_upload(
            content=image,
            token=self.slack_user_token,
            filename=filename,
        )
        if not response["ok"]:
            logging.info("Error uploading file %s" % response)
            return
        file_id = response["file"]["id"]

        # Make it public
        response = self.slack_app.client.files_sharedPublicURL(
            file=file_id,
            token=self.slack_user_token,
        )
        if not response["ok"]:
            logging.info("Error making file public %s" % response)
            return
        permalink_public = response["file"]["permalink_public"]
        team_id, _, pub_secret = os.path.split(permalink_public)[1].split("-")

        direct_link = "https://files.slack.com/files-pri/" + team_id + "-" + file_id + "/" + filename + "?pub_secret=" + pub_secret
        return direct_link

    def send_image(self):
        """
        POST method at the endpoint /send_image

        Expects a json payload with keys:
        - 'image' with the ascii-decoding of the base64-encoded bytes
        - 'users' with a list of non-repeating user_i values (ints) that
          index into self.users

        Returns a json payload with keys:
        - 'num_sent' with the num images sent (<= len(users))
        - 'message_id', which this image will be referred to from hereon out.
        """
        # Decode the request
        image = base64.decodebytes(request.json['image'].encode('ascii'))
        users = request.json['users']

        message_id = self.sent_messages_database.get_new_message_id()
        self.database_updated()

        # Get a URL to the image
        direct_link = self.get_image_url(image, message_id)

        # Send the message
        sent_users = set()
        if direct_link is not None:
            for user_i in users:
                user_i = int(user_i)
                # Ensure user_i values are not repeated
                if user_i in sent_users: continue
                sent_users.add(user_i)

                # Get the message
                user_id = self.users[user_i]
                payload = slack_template_1(user_id, direct_link)

                # Send the meessage
                response = self.slack_app.client.chat_postMessage(**payload)
                if not response["ok"]:
                    logging.info("Error sending file to user %s %s" % (user_id, response))
                    continue
                ts = response["message"]["ts"]

                # Store a reference to the message
                self.sent_messages_database.add_sent_message(message_id, user_id, ts)
                self.database_updated()

        response = self.flask_app.response_class(
            response=json.dumps({'num_sent': len(sent_users), 'message_id' : message_id}),
            status=200,
            mimetype='application/json'
        )
        return response

    def get_num_users(self):
        """
        GET method at the endpoint /get_num_users
        Returns a json payload with keys:
        - 'num_users' with the number of users

        """
        response = self.flask_app.response_class(
            response=json.dumps({'num_users': len(self.users)}),
            status=200,
            mimetype='application/json'
        )
        return response

    def get_responses(self):
        """
        POST method at the endpoint /get_responses
        Expects a json payload with keys:
        - 'message_ids_and_user_ids' a dictionary with message_id keys and a
          a list of requested users as the values.

        Returns a json payload with keys:
        - 'message_id_to_user_reactions' a dict with message_id keys and
          a list of (user_i, reaction) for the users who have reacted, where
          user_i is the index of user_id in self.users

        """
        message_ids_and_user_ids = request.json['message_ids_and_user_ids']

        message_id_to_user_id_reaction = self.sent_messages_database.get_reactions(message_ids_and_user_ids)

        # Convert user_id to client-facing user_i, and filter to only include
        # the requested users
        message_id_to_user_i_reaction = {}
        for message_id in message_id_to_user_id_reaction:
            message_id_to_user_i_reaction[message_id] = []
            for (user_id, reaction) in message_id_to_user_id_reaction[message_id]:
                user = self.users.index(user_id)
                if user in message_ids_and_user_ids[message_id]:
                    message_id_to_user_i_reaction[message_id].append((user, reaction))
        response = self.flask_app.response_class(
            response=json.dumps({'message_id_to_user_reactions':message_id_to_user_i_reaction}),
            status=200,
            mimetype='application/json'
        )
        return response

    def action_button_check_mark(self, body, ack, say):
        """
        Bolt App callback for when the user clicks the :check_mark: button on
        a message
        """
        # Acknowledge the action
        ack()
        self.recv_reaction(body, 1)

    def action_button_x(self, body, ack, say):
        """
        Bolt App callback for when the user clicks the :x: button on a message
        """
        # Acknowledge the action
        ack()
        self.recv_reaction(body, 0)

    def get_response_json(self, message_id, user_id, reaction):
        """
        Returns the json payload (as a dict) of a human response of reaction to
        image message_id
        """
        user_i = self.users.index(user_id)
        return {'message_id':message_id, 'user':user_i, 'reaction':reaction}

    def recv_reaction(self, body, reaction, num_tries=3):
        """
        Stores the user's reaction.
        """
        # Get the user_id and ts
        user_id = body["user"]["id"]
        ts = body["container"]["message_ts"]
        logging.info('Got reaction %s from user %s for message at ts %s' % (reaction, user_id, ts))

        # Increment the appropriate values
        self.sent_messages_database.add_reaction(user_id, ts, reaction)
        self.database_updated()

    def start(self):
        """
        Start the FlastSlackbot
        """
        # Launch the Flask app in another thread
        self.flask_thread = threading.Thread(
            target=self.flask_app.run,
            kwargs={"host":'0.0.0.0', "port":self.flask_port, "threaded":True},
        )
        self.flask_thread.daemon = True
        self.flask_thread.start()

        # Launch the Slack app in the main thread
        self.slack_app.start(port=self.slack_port)

if __name__ == '__main__':
    # Create and configure the logger
    log_filepath = "../logs/log.txt"
    log_formatter = logging.Formatter("%(asctime)s [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s")
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.NOTSET)
    file_handler = logging.FileHandler(log_filepath)
    file_handler.setFormatter(log_formatter)
    file_handler.setLevel(logging.NOTSET)
    root_logger.addHandler(file_handler)
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(log_formatter)
    console_handler.setLevel(logging.INFO)
    root_logger.addHandler(console_handler)

    # Load the Slackbot configuration
    slackbot_conf_filepath = "../cfg/slackbot.yaml"
    with open(slackbot_conf_filepath, 'r') as f:
        slackbot_conf = yaml.load(f, Loader=yaml.FullLoader)

    # Create and start the FlaskSlackbot
    server = FlaskSlackbot(slackbot_conf)
    server.start()
