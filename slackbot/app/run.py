import base64
import cv2 # remove after I finish debugging images
import datetime
from flask import Flask, request
import hashlib
import json
import logging
import numpy as np # remove after I finish debugging images
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
        self.flask_app.add_url_rule('/send_images',
            view_func=self.send_images, methods=['POST'])
        self.flask_app.add_url_rule('/get_num_users',
            view_func=self.get_num_users, methods=['GET'])
        self.flask_app.add_url_rule('/get_updates',
            view_func=self.get_updates, methods=['POST'])

        # Create the Slack app
        self.slack_user_token = slackbot_conf['slack_user_token']
        self.slack_app = App(
            token=slackbot_conf['slack_bot_token'],
            signing_secret=slackbot_conf['slack_signing_secret']
        )
        self.slack_port = slack_port
        self.slack_app.action("action_id_check_mark")(self.action_button_check_mark)
        self.slack_app.action("action_id_x")(self.action_button_x)
        self.slack_app.command("/test_get_images")(self.test_get_images)

        # Store the Slack users
        self.users = slackbot_conf['users_list']

        # Keep track of the images that are sent
        self.sent_messages_database_filepath = sent_messages_database_filepath
        self.sent_messages_database = SentMessagesDatabase.load(
            sent_messages_database_filepath)
        self.database_save_interval = database_save_interval
        self.database_updates_since_last_save = 0

    def database_updated(self, num_updates=1):
        """
        Called everytime the database is updated. Saves the database every
        self.database_save_interval updates
        """
        self.database_updates_since_last_save += num_updates
        if self.database_updates_since_last_save >= self.database_save_interval:
            self.sent_messages_database.save(self.sent_messages_database_filepath)
            self.database_updates_since_last_save = 0

    def get_image_urls(self, images_bytes, image_ids):
        """
        Uploads the image to Slack and gets a public URL for it.

        Input:
        - images_bytes is a list of the bytes in the images to be uploaded.
        - image_ids is a list corresponding to the ids for the images

        Output:
        - A same-length list, where each entry is either an image_url or None if
          uploading the image failed.
        """
        image_urls = []
        for i in range(len(image_ids)):
            image_bytes = images_bytes[i]
            image_id = image_ids[i]
            # print("image_bytes", image_bytes, "i", i)

            # Get the image_url, generating a new one if we have not already
            # uploaded this image.
            image_url = self.sent_messages_database.get_image_url(image_id)
            if image_url is None:
                filename = image_id+".jpg"

                # Upload the file
                response = self.slack_app.client.files_upload(
                    content=image_bytes,
                    token=self.slack_user_token,
                    filename=filename,
                )
                if not response["ok"]:
                    logging.info("Error uploading file %s" % response)
                    image_urls.append(None)
                    continue
                file_id = response["file"]["id"]

                # Make it public
                response = self.slack_app.client.files_sharedPublicURL(
                    file=file_id,
                    token=self.slack_user_token,
                )
                if not response["ok"]:
                    logging.info("Error making file public %s" % response)
                    image_urls.append(None)
                    continue
                permalink_public = response["file"]["permalink_public"]
                team_id, _, pub_secret = os.path.split(permalink_public)[1].split("-")

                image_url = "https://files.slack.com/files-pri/" + team_id + "-" + file_id + "/" + filename + "?pub_secret=" + pub_secret
            image_urls.append(image_url)

        return image_urls

    # def get_image_url(self, image, message_id):
    #     """
    #     Uploads the image to Slack and gets a public URL for it.
    #
    #     Input:
    #     - image bytes
    #
    #     Output:
    #     - if the image upload is succesful, return a public url (string) that
    #       points to the image
    #     - else, return None
    #     """
    #     filename = message_id+".jpg"
    #
    #     # Upload the file
    #     response = self.slack_app.client.files_upload(
    #         content=image,
    #         token=self.slack_user_token,
    #         filename=filename,
    #     )
    #     if not response["ok"]:
    #         logging.info("Error uploading file %s" % response)
    #         return
    #     file_id = response["file"]["id"]
    #
    #     # Make it public
    #     response = self.slack_app.client.files_sharedPublicURL(
    #         file=file_id,
    #         token=self.slack_user_token,
    #     )
    #     if not response["ok"]:
    #         logging.info("Error making file public %s" % response)
    #         return
    #     permalink_public = response["file"]["permalink_public"]
    #     team_id, _, pub_secret = os.path.split(permalink_public)[1].split("-")
    #
    #     direct_link = "https://files.slack.com/files-pri/" + team_id + "-" + file_id + "/" + filename + "?pub_secret=" + pub_secret
    #     return direct_link

    def get_image_ids(self, images_bytes):
        """
        Takes in a list of the bytes of images. Returns a list of corresponding
        image_ids. The image_ids will be hashes of the bytes of the images. That
        way, if the same image was already uploaded to Slack, the Slackbot can
        return that image's image_id as opposed to returning a new image_id.
        """
        image_ids = []
        for image_bytes in images_bytes:
            # image_id = str(hash(image_bytes))
            image_id = hashlib.sha1(image_bytes).hexdigest()
            image_ids.append(image_id)
        return image_ids

    def send_image_to_slack(self, image_id, direct_link, user_id, image_description):
        """
        Actually sends the Slack message. Returns a boolean indicating whether
        the send was succesful or not.
        """
        payload = slack_template_1(user_id, direct_link, image_description)

        # Send the message
        response = self.slack_app.client.chat_postMessage(**payload)
        if not response["ok"]:
            logging.info("Error sending file to user %s %s" % (user_id, response))
            return False
        ts = response["message"]["ts"]

        # Store a reference to the message
        self.sent_messages_database.add_sent_message(image_id, user_id, ts)
        self.database_updated()
        return True

    def send_images(self):
        """
        POST method at the endpoint /send_images

        Expects a json payload with keys:
        - 'images' is a list of the ascii-decoding of the base64-encoded bytes
          of the images
        - 'user' is a user_i value (int) that index into self.users

        Returns a json payload with keys:
        - 'image_ids' with the image_id corresponding to each image.
        - 'first_send_image_state' with a boolean indicating whether the first
          sent image succeeded
        """

        # Decode the request
        images_bytes = []
        # print("request.json['images']", request.json['images'], len(request.json['images']))
        for i in range(len(request.json['images'])):
            image = request.json['images'][i]
            image_bytes = base64.decodebytes(image.encode('ascii')) # image.encode("utf-8") #
            images_bytes.append(image_bytes)
        user = request.json['user']

        image_ids = self.get_image_ids(images_bytes)

        # Get a URL to the image, and remove failed URLs
        image_ids_to_return = []
        image_urls = self.get_image_urls(images_bytes, image_ids)
        print("image_ids", image_ids, "image_urls", image_urls)
        i = 0
        while i < len(image_ids):
            image_id = image_ids[i]
            image_url = image_urls[i]
            if image_url is None:
                image_ids.pop(i)
                image_urls.pop(i)
                image_ids_to_return.append(False)
            else:
                i += 1
                image_ids_to_return.append(image_id)
        self.sent_messages_database.add_image_urls(image_ids, image_urls)
        self.database_updated(len(image_ids))

        if 'image_descriptions' in request.json:
            image_descriptions = request.json['image_descriptions']
        else:
            image_descriptions = [None for _ in range(len(image_ids))]

        # Send the first message
        image_id = image_ids.pop(0)
        direct_link = image_urls.pop(0)
        image_description = image_descriptions.pop(0)
        user_id = self.users[user]
        send_result = self.send_image_to_slack(image_id, direct_link, user_id, image_description)

        # Update the images to send
        self.sent_messages_database.set_remaining_images_to_send(user_id, image_ids, image_urls, image_descriptions)

        response = self.flask_app.response_class(
            response=json.dumps({'image_ids':image_ids_to_return, 'first_send_image_state':send_result}),
            status=200,
            mimetype='application/json'
        )
        return response

    # def send_image(self):
    #     """
    #     POST method at the endpoint /send_image
    #
    #     Expects a json payload with keys:
    #     - 'image' with the ascii-decoding of the base64-encoded bytes
    #     - 'users' with a list of non-repeating user_i values (ints) that
    #       index into self.users
    #
    #     Returns a json payload with keys:
    #     - 'num_sent' with the num images sent (<= len(users))
    #     - 'message_id', which this image will be referred to from hereon out.
    #     """
    #     # Decode the request
    #     image = base64.decodebytes(request.json['image'].encode('ascii'))
    #     users = request.json['users']
    #
    #     message_id = self.sent_messages_database.get_new_message_id()
    #     self.database_updated()
    #
    #     # Get a URL to the image
    #     direct_link = self.get_image_url(image, message_id)
    #
    #     # Send the message
    #     sent_users = set()
    #     if direct_link is not None:
    #         for user_i in users:
    #             user_i = int(user_i)
    #             # Ensure user_i values are not repeated
    #             if user_i in sent_users: continue
    #             sent_users.add(user_i)
    #
    #             # Get the message
    #             user_id = self.users[user_i]
    #             payload = slack_template_1(user_id, direct_link)
    #
    #             # Send the meessage
    #             response = self.slack_app.client.chat_postMessage(**payload)
    #             if not response["ok"]:
    #                 logging.info("Error sending file to user %s %s" % (user_id, response))
    #                 continue
    #             ts = response["message"]["ts"]
    #
    #             # Store a reference to the message
    #             self.sent_messages_database.add_sent_message(message_id, user_id, ts)
    #             self.database_updated()
    #
    #     response = self.flask_app.response_class(
    #         response=json.dumps({'num_sent': len(sent_users), 'message_id' : message_id}),
    #         status=200,
    #         mimetype='application/json'
    #     )
    #     return response

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

    def get_updates(self):
        """
        POST method at the endpoint /get_updates
        Expects a json payload with keys:
        - 'image_ids_and_users' a dictionary with image_id keys and a
          a list of requested users as the values.

        Returns a json payload with keys:
        - 'image_id_to_user_reactions' a dict with image_id keys and
          a list of (user_i, reaction) for the users who have reacted, where
          user_i is the index of user_id in self.users
        - 'time_to_send' a dict with keys being user_i and the number of seconds
          before sending them new images.

        """
        image_ids_and_users = request.json['image_ids_and_users']

        # Convert the user from the robot's perspective to the Slack user_id
        image_ids_and_user_ids = {}
        for image_id in image_ids_and_users:
            image_ids_and_user_ids[image_id] = []
            for user in image_ids_and_users[image_id]:
                if user >= len(self.users):
                    logging.info("Recieved user %s which is greater than num users %d" % (user, len(self.users)))
                    continue
                user_id = self.users[user]
                image_ids_and_user_ids[image_id].append(user_id)

        image_id_to_user_id_reaction = self.sent_messages_database.get_reactions(image_ids_and_user_ids)

        # Convert user_id to client-facing user_i, and filter to only include
        # the requested users
        image_id_to_user_reaction = {}
        for image_id in image_id_to_user_id_reaction:
            image_id_to_user_reaction[image_id] = []
            for (user_id, reaction) in image_id_to_user_id_reaction[image_id]:
                user = self.users.index(user_id)
                image_id_to_user_reaction[image_id].append((user, reaction))

        # Get the time to send per user
        user_id_to_next_image_send_time = self.sent_messages_database.get_user_time_to_send()
        time_to_send = {}
        curr_time = time.time()
        for user_id in user_id_to_next_image_send_time:
            user = self.users.index(user_id)
            time_to_send[user] = user_id_to_next_image_send_time[user_id] - curr_time

        response = self.flask_app.response_class(
            response=json.dumps({'image_id_to_user_reactions':image_id_to_user_reaction, 'time_to_send':time_to_send}),
            status=200,
            mimetype='application/json'
        )
        return response
        #I think add condition to this as well?
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

    # def get_response_json(self, message_id, user_id, reaction):
    #     """
    #     Returns the json payload (as a dict) of a human response of reaction to
    #     image message_id
    #     """
    #     user_i = self.users.index(user_id)
    #     return {'message_id':message_id, 'user':user_i, 'reaction':reaction}

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

        # Send the next image
        image_id, direct_link, image_description = self.sent_messages_database.get_next_image_to_send(user_id)
        self.database_updated()
        if image_id is not None:
            self.send_image_to_slack(image_id, direct_link, user_id, image_description)


    def test_get_images(self, ack, say, command, event, respond):
        """
        Bolt App callback for when the user types /test_get_images into the
        app. This will store that the user requested images. Then, the next
        time the robot pulls updates from the Flask App, it will indicate to
        the robot to send images. The robot will then send images to the
        Flask app to be displayed.
        """
        # Acknowledge the command
        ack()
        print("test_get_images", ack, say, command, event, respond)
        # Store that the user requested images.
        user_id = command["user_id"]
        self.sent_messages_database.add_user_send_image_time(user_id, time.time())
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
