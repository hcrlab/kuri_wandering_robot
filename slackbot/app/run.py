import base64
import csv
import cv2
import datetime
from flask import Flask, request
import hashlib
import json
import logging
import numpy as np
import os
import pprint
import random
import re
import requests
from sent_messages_database import SentMessagesDatabase
from slack_bolt import App
import slack_templates
import string
import sys
import threading
import time
import traceback
import yaml

class FlaskSlackbot(object):
    def __init__(self, slackbot_conf, data_base_dir="../data/", save_csvs=False,
        save_images=False):
        """
        Create the FlaskSlackbot.

        :param slackbot_conf: (dict) the loaded yaml that was generated by
                              create_slackbot_config.py
        :param data_base_dir: (str) the directory to store data logs in
        :param save_csvs: (bool) whether or not to store CSVs logging sent
                          messages and user responses
        :param save_images: (bool) whether or not to store sent images as JPGs
        :returns: an instance of the FlaskSlackbot class
        """
        # Create the folder to store data in
        self.data_base_dir = data_base_dir
        if not os.path.isdir(self.data_base_dir):
            os.makedirs(self.data_base_dir)

        # Configure CSV storage
        self.save_csvs = save_csvs
        if self.save_csvs:
            # Each variable below contains a filename and CSV header row
            self.sent_messages_csv = ("sent_messages.csv", ["Time", "Slackbot User ID", "Message Timestamp", "Image ID", "Image URL", "Image Filepath"])
            self.responses_csv = ("reactions.csv", ["Time", "Slackbot User ID", "Message Timestamp", "Image URL", "Button Click Response", "Open-Ended Question", "Open-Ended Response"])
            for csv_filename, csv_header in [self.sent_messages_csv, self.responses_csv]:
                csv_filepath = os.path.join(self.data_base_dir, csv_filename)
                if not os.path.exists(csv_filepath):
                    with open(csv_filepath, "w") as f:
                        csv_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                        csv_writer.writerow(csv_header)

        # Confirgure Image storage
        self.save_images = save_images
        if self.save_images:
            self.images_base_dir = os.path.join(self.data_base_dir, "images/")
            if not os.path.isdir(self.images_base_dir):
                os.makedirs(self.images_base_dir)

        # Keep track of the messages that are sent
        self.sent_messages_database_filepath = os.path.join(self.data_base_dir, "sent_messages_database.pkl")
        self.sent_messages_database = SentMessagesDatabase.load(
            self.sent_messages_database_filepath)
        self.database_save_interval = 1
        self.database_updates_since_last_save = 0

        # Store the Slack users
        self.users = slackbot_conf['users_list']
        self.low_battery_helpers = []
        self.where_am_i_helpers = []
        for user in self.users:
            if slackbot_conf[user]['low_battery_helper']:
                self.low_battery_helpers.append(user)
            if slackbot_conf[user]['where_am_i_helper']:
                self.where_am_i_helpers.append(user)

        # Create and configure the Flask App
        self.flask_app = Flask("slackbot")
        self.flask_port = slackbot_conf['flask_port']
        self.flask_app.debug = False
        self.flask_app.add_url_rule('/low_battery',
            view_func=self.low_battery, methods=['POST'])
        self.flask_app.add_url_rule('/where_am_i',
            view_func=self.where_am_i, methods=['POST'])
        self.flask_app.add_url_rule('/get_updates',
            view_func=self.get_updates, methods=['POST'])

        # Create the Slack app
        self.slack_user_token = slackbot_conf['slack_user_token']
        self.slack_app = App(
            token=slackbot_conf['slack_bot_token'],
            signing_secret=slackbot_conf['slack_signing_secret']
        )
        self.slack_port = slackbot_conf['slack_port']
        self.slack_app.action(re.compile("button_click_[*]*"))(self.button_click)
        self.slack_app.action("submit_input")(self.submit_input)

    def database_updated(self, num_updates=1):
        """
        Called everytime the database is updated. Saves the database every
        self.database_save_interval updates
        """
        self.database_updates_since_last_save += num_updates
        if self.database_updates_since_last_save >= self.database_save_interval:
            self.sent_messages_database.save(self.sent_messages_database_filepath)
            self.database_updates_since_last_save = 0

    def low_battery(self):
        """
        POST method at the endpoint /low_battery. Sends a message notifying the
        recipient of the robot's battery percentage.

        :param battery_pct: (str) part of the input json payload, indicating
                            the robot's current battery percentage
        :returns: a json payload with `success` indicating whether the message
                  was sent to at least one user, and `message_id` as a unique
                  identifier to this help request.
        """
        # Decode the request
        battery_pct = int(request.json['battery_pct'])
        image_url = None
        csv_metadata = []
        if 'image' in request.json:
            # Decode the Image
            image = request.json['image']
            image_bytes = base64.decodebytes(image.encode('ascii'))

            # Get an image_id
            image_id = self.get_image_id(image_bytes)
            image_filepath = ""
            if self.save_images:
                image_filepath = self.save_image(image_bytes, image_id)

            # Get a URL to the image
            image_url = self.get_image_url(image_bytes, image_id)
            logging.info("image_id %s image_url %s" % (image_id, image_url))
            self.sent_messages_database.add_image_url(image_id, image_url)
            self.database_updated()
            csv_metadata = [image_id, image_url, image_filepath]

        # Get Message ID
        message_id = self.sent_messages_database.get_message_id()

        # Send it to the helpers
        success = False
        for user_id in self.low_battery_helpers:
            payload = slack_templates.low_battery_template(user_id, battery_pct, image_url)
            ts = self.send_message_to_slack(payload, message_id, False, csv_metadata)
            if ts is not None:
                success = True

        # Send the result as a JSON payload
        response = self.flask_app.response_class(
            response=json.dumps({'success':success, 'message_id':message_id}),
            status=200,
            mimetype='application/json'
        )
        return response

    def send_message_to_slack(self, payload, message_id,
        can_users_respond=False, csv_metadata=[]):
        """
        Sends the message specified in payload to Slack.

        :param payload: (dict) the message metadata and blocks to send
        :param message_id: (int) the message ID
        :param can_users_respond: (bool) indicates users can respond or interact
                                   with the message.
        :param csv_metadata: (list) data to append to the csv row, in addition
                             to the time, user ID, and message timestamp.
        :returns: a timestamp if the message was succesfully sent, else None
        """
        user_id = payload["channel"]

        try:
            response = self.slack_app.client.chat_postMessage(**payload)
            if not response["ok"]:
                logging.info("Error sending file to user %s: %s" % (user_id, response))
            else:
                ts = response["message"]["ts"]

                self.sent_messages_database.add_sent_message(user_id, ts, message_id, can_users_respond)
                self.database_updated()

                # Save it to the CSV
                if self.save_csvs:
                    if len(csv_metadata) == 0:
                        csv_metadata = ["" for i in range(3, len(self.sent_messages_csv[1]))]
                    csv_filepath = os.path.join(self.data_base_dir, self.sent_messages_csv[0])
                    with open(csv_filepath, "a") as f:
                        csv_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                        csv_writer.writerow([time.time(), user_id, ts]+csv_metadata)

                return ts
        except Exception as e:
            logging.info("Error sending file to user %s" % (user_id))
            if "response" in locals():
                logging.info("Response text %s." % response.text)
            logging.info(traceback.format_exc())
            logging.info("Error %s." % e)

    def where_am_i(self):
        """
        POST method at the endpoint /where_am_i. Sends a Slack message with
        the given image, and shows the user buttons correpsonding to where the
        robot thinks it is. The user either clicks one of those buttons or
        Other. If Other, the robot asks the user to type in where it is.

        :param image: (str) the ascii-decoding of the base64-encoded bytes of
                      the image of the robot's current view
        :param options: (list of str) the human-readable locations the robot
                        thinks it might be in.
        :returns: a json payload with `success` indicating whether the message
                  was sent to at least one user, and `message_id` as a unique
                  identifier to this help request.
        """
        # Decode the request
        image = request.json['image']
        image_bytes = base64.decodebytes(image.encode('ascii'))
        options = request.json['options']

        # Get an image_id
        image_id = self.get_image_id(image_bytes)
        image_filepath = ""
        if self.save_images:
            image_filepath = self.save_image(image_bytes, image_id)

        # Get a URL to the image
        image_url = self.get_image_url(image_bytes, image_id)
        logging.info("image_id %s image_url %s" % (image_id, image_url))
        self.sent_messages_database.add_image_url(image_id, image_url)
        self.database_updated()

        # Get Message ID
        message_id = self.sent_messages_database.get_message_id()

        # Send it to the helpers
        success = False
        if image_url is not None:
            for user_id in self.where_am_i_helpers:
                payload = slack_templates.where_am_i_template(user_id, image_url, options)
                ts = self.send_message_to_slack(payload, message_id, True, [image_id, image_url, image_filepath])
                if ts is not None:
                    success = True

        # Send the result as a JSON payload
        response = self.flask_app.response_class(
            response=json.dumps({'success':success, 'message_id':message_id}),
            status=200,
            mimetype='application/json'
        )
        return response

    def get_image_id(self, image_bytes):
        """
        Returns the Sha1 hash of the image bytes as the image ID.

        :param image_bytes: (bytes) the image bytes
        :returns: (str) the Sha1 hash of the image as an image ID
        """
        image_id = hashlib.sha1(image_bytes).hexdigest()
        return image_id

    def save_image(self, image_bytes, image_id):
        """
        Saves the image to file

        :param image_bytes: (bytes) the image bytes
        :param image_id: (str) the image ID
        :returns: (str) the filepath to the image
        """
        image_cv2 = cv2.imdecode(np.fromstring(image_bytes, np.uint8), cv2.IMREAD_COLOR)
        image_filepath = os.path.join(self.images_base_dir, str(image_id)+".jpg")
        cv2.imwrite(image_filepath, image_cv2)
        return image_filepath

    def get_image_url(self, image_bytes, image_id):
        """
        Uploads the image to Slack and gets a public URL for it.

        :param image_bytes: (bytes) the image bytes
        :param image_id: (str) the image ID
        :returns: (str) the public URL to the image
        """
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
                return None
            file_id = response["file"]["id"]

            # Make it public
            response = self.slack_app.client.files_sharedPublicURL(
                file=file_id,
                token=self.slack_user_token,
            )
            if not response["ok"]:
                logging.info("Error making file public %s" % response)
                return None
            permalink_public = response["file"]["permalink_public"]
            team_id, _, pub_secret = os.path.split(permalink_public)[1].split("-")

            image_url = "https://files.slack.com/files-pri/" + team_id + "-" + file_id + "/" + filename + "?pub_secret=" + pub_secret
        return image_url

    def get_updates(self):
        """
        POST method at the endpoint /get_updates

        :param message_ids_and_action_ts: (dict: str -> float) an optional
            parameter that specifies the message_ids that you want updates for,
            and action_ts that you want updates after (to prevent sending
            updates you already have).

        :returns: a json payload with `message_id_to_responses` which is a
                  dict mapping from message IDs to a list of user responses,
                  ordered chronologically. where each user responses contains a
                  timestamp and a responses.
        """
        # Decode the request
        message_ids_and_action_ts = None
        if 'message_ids_and_action_ts' in request.json:
            message_ids_and_action_ts = request.json['message_ids_and_action_ts']

        # Get the user responses
        message_id_to_responses = self.sent_messages_database.get_responses(message_ids_and_action_ts)

        # Send the result as a JSON payload
        response = self.flask_app.response_class(
            response=json.dumps({'message_id_to_responses':message_id_to_responses}),
            status=200,
            mimetype='application/json'
        )
        return response

    def button_click(self, body, ack, respond):
        """
        Bolt App action callback for when the user clicks a button.

        :param body: (dict) the body of the json payload, including details
                     about the message and the button clicked
        :param ack: (function) a function to acknowledge receipt of this action
                    to Slack.
        :param respond: (function) a function to respond by updating the message
        :returns: None
        """
        # Acknowledge the action
        ack()

        # Determine what button was clicked on what message
        user_id, ts, image_url, button_label, action_ts = slack_templates.get_button_click_metadata(body)
        logging.info('Got button click %s from user %s for message at ts %s' % (button_label, user_id, ts))

        # Add the button click to the database
        self.sent_messages_database.add_response(user_id, ts, button_label, action_ts)
        self.database_updated()

        # Save it to the CSV
        if self.save_csvs:
            csv_filepath = os.path.join(self.data_base_dir, self.responses_csv[0])
            with open(csv_filepath, "a") as f:
                csv_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                csv_writer.writerow([time.time(), user_id, ts, image_url, button_label, "", ""])

        # Update the message
        response = slack_templates.got_button_click(body, user_id, button_label)
        respond(response)

    def submit_input(self, body, ack, respond):
        """
        Bolt App action callback for when the user submits an open-ended
        response.

        :param body: (dict) the body of the json payload, including details
                     about the message and the button clicked
        :param ack: (function) a function to acknowledge receipt of this action
                    to Slack.
        :param respond: (function) a function to respond by updating the message
        :returns: None
        """
        # Acknowledge the action
        ack()

        # Determine what response was given on what message
        user_id, ts, image_url, question, answer, action_ts = slack_templates.get_submit_input_metadata(body)
        logging.info('Got submit input %s from user %s for message at ts %s' % (answer, user_id, ts))

        # Add the button click to the database
        self.sent_messages_database.add_response(user_id, ts, answer, action_ts)
        self.database_updated()

        # Save it to the CSV
        if self.save_csvs:
            csv_filepath = os.path.join(self.data_base_dir, self.responses_csv[0])
            with open(csv_filepath, "a") as f:
                csv_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                csv_writer.writerow([time.time(), user_id, ts, image_url, "", question, answer])

        # Update the message
        response = slack_templates.got_submit_input(body, user_id, answer)
        respond(response)

    def start(self):
        """
        Start the FlaskSlackbot
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
    server = FlaskSlackbot(slackbot_conf)#, save_csvs=True, save_images=True)
    server.start()
