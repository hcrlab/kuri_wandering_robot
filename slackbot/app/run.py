import base64
import csv
import cv2 # remove after I finish debugging images
import datetime
from flask import Flask, request
import hashlib
import json
import logging
import numpy as np # remove after I finish debugging images
import os
import pprint
import random
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
    def __init__(self, slackbot_conf, flask_port=8194, slack_port=8193,
        sent_messages_database_filepath="../cfg/sent_messages_database.pkl",
        database_save_interval=1, data_base_dir="/home/ubuntu/kuri_cmm_demo/data/"):
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
        self.slack_app.action("confirm_input")(self.confirm_input)
        self.slack_app.command("/test_get_images_2")(self.test_get_images)
        self.slack_app.command("/test_end_of_day_message_0")(self.test_end_of_day_message_0)
        self.slack_app.command("/test_end_of_day_message_1")(self.test_end_of_day_message_1)
        self.slack_app.command("/test_end_of_day_message_2")(self.test_end_of_day_message_2)
        self.slack_app.command("/send_daily_intro_message_0")(self.send_daily_intro_message_0)
        self.slack_app.command("/send_daily_intro_message_1")(self.send_daily_intro_message_1)
        self.slack_app.command("/send_daily_intro_message_2")(self.send_daily_intro_message_2)
        self.slack_app.command("/test_pre_study_message")(self.test_pre_study_message)

        # Configure data storage
        self.data_base_dir = data_base_dir
        if not os.path.isdir(self.data_base_dir):
            os.makedirs(self.data_base_dir)
        # Each variable below contains the filename and the header row
        self.received_images_csv = ("received_images.csv", ["Time", "Filepath", "Image ID", "Image URL", "Slackbot User ID", "Robot User ID"])
        self.sent_images_csv = ("sent_images.csv", ["Time", "Image URL", "Slackbot User ID", "Message Timestamp"])
        self.reactions_csv = ("reactions.csv", ["Time", "Image URL", "Slackbot User ID", "Message Timestamp", "Reaction"])
        self.followups_csv = ("followups.csv", ["Time", "Image URL", "Slackbot User ID", "Message Timestamp", "Question", "Response"])
        self.survey_csv = ("survey.csv", ["Time", "Survey Random ID", "Survey URL", "Day", "Slackbot User ID", "Message Timestamp"])
        for csv_filename, csv_header in [self.received_images_csv, self.sent_images_csv, self.reactions_csv, self.followups_csv, self.survey_csv]:
            csv_filepath = os.path.join(self.data_base_dir, csv_filename)
            if not os.path.exists(csv_filepath):
                with open(csv_filepath, "w") as f:
                    csv_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    csv_writer.writerow(csv_header)
        self.images_base_dir = os.path.join(self.data_base_dir, "images/")
        if not os.path.isdir(self.images_base_dir):
            os.makedirs(self.images_base_dir)

        # Keep track of the images that are sent
        self.sent_messages_database_filepath = sent_messages_database_filepath
        self.sent_messages_database = SentMessagesDatabase.load(
            sent_messages_database_filepath)
        self.database_save_interval = database_save_interval
        self.database_updates_since_last_save = 0

        # Store the Slack users
        self.users = slackbot_conf['users_list']
        self.users_to_configuration = {}
        for user in self.users:
            if user not in slackbot_conf:
                logging.warn("Error, user %s should have a configuration but does not. Skipping user." % user)
                continue
            self.users_to_configuration[user] = slackbot_conf[user]
        self.user_id_to_next_image_day = {}
        self.user_id_to_next_image_i = {}
        self.send_scheduled_messages()

    def send_scheduled_messages(self):
        for user_id in self.users_to_configuration:
            # Send all the scheduled messages
            if 'pre_study_message_timestamp' in self.users_to_configuration[user_id]:
                pre_study_message_timestamp = self.users_to_configuration[user_id]['pre_study_message_timestamp']
                if not self.sent_messages_database.was_scheduled_message_sent(user_id, pre_study_message_timestamp):
                    sent = self.send_pre_study_message(user_id, pre_study_message_timestamp)
                    if sent:
                        self.sent_messages_database.scheduled_message_was_sent(user_id, pre_study_message_timestamp)
                        self.database_updated()

            if 'daily_schedule' in self.users_to_configuration[user_id]:
                for day in range(len(self.users_to_configuration[user_id]['daily_schedule'])):
                    daily_schedule = self.users_to_configuration[user_id]['daily_schedule'][day]

                    intro_message_timestamp = daily_schedule['intro_message_timestamp']
                    if not self.sent_messages_database.was_scheduled_message_sent(user_id, intro_message_timestamp):
                        sent = self.send_daily_intro_message(user_id, day, intro_message_timestamp)
                        if sent:
                            self.sent_messages_database.scheduled_message_was_sent(user_id, intro_message_timestamp)
                            self.database_updated()

                    survey_timestamp = daily_schedule['survey_timestamp']
                    if not self.sent_messages_database.was_scheduled_message_sent(user_id, survey_timestamp):
                        sent = self.send_end_of_day_message(user_id, day, survey_timestamp)
                        if sent:
                            self.sent_messages_database.scheduled_message_was_sent(user_id, survey_timestamp)
                            self.database_updated()

            # Initialize the image send times to the first image send timestamp in the
            # future. Assumes timestamps across days and images are monotonically increasing
            if 'daily_schedule' in self.users_to_configuration[user_id]:
                for day in range(len(self.users_to_configuration[user_id]['daily_schedule'])):
                    daily_schedule = self.users_to_configuration[user_id]['daily_schedule'][day]
                    for image_i in range(len(daily_schedule['images_sent_timestamps'])):
                        next_image_timestamp = daily_schedule['images_sent_timestamps'][image_i]
                        if time.time() < next_image_timestamp:
                            self.user_id_to_next_image_day[user_id] = day
                            self.user_id_to_next_image_i[user_id] = image_i
                            self.sent_messages_database.add_user_send_image_time(user_id, next_image_timestamp)
                            self.database_updated()
                            break
                    if user_id in self.user_id_to_next_image_day:
                        break
                # If all image send times are in the past
                if user_id not in self.user_id_to_next_image_day:
                    self.user_id_to_next_image_day[user_id] = None
                    self.user_id_to_next_image_i[user_id] = None
                    self.sent_messages_database.add_user_send_image_time(user_id, None)
                    self.database_updated()


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

    def send_images_to_slack(self, image_ids, image_urls, user_id, image_descriptions):
        """
        Actually sends the Slack messages. Returns a boolean indicating whether
        the send was succesful or not.
        """
        n_images = min(len(image_ids), len(image_urls))
        n_successes = 0

        for i in range(-1, n_images):
            if i == -1:
                image_url = ""
                payload = slack_templates.post_images_intro(user_id, n_images)
            else:
                image_url = image_urls[i]
                payload = slack_templates.post_image(user_id, image_url, image_descriptions[i], i, n_images)

            # Send the message
            response = self.slack_app.client.chat_postMessage(**payload)
            if not response["ok"]:
                logging.info("Error sending file to user %s: %s" % (user_id, response))
                continue
            ts = response["message"]["ts"]

            # Store a reference to the message
            if i > -1:
                self.sent_messages_database.add_sent_message(image_ids[i], user_id, ts)
                self.database_updated()

            # Save it in the CSV
            csv_filepath = os.path.join(self.data_base_dir, self.sent_images_csv[0])
            with open(csv_filepath, "a") as f:
                csv_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                csv_writer.writerow([time.time(), image_url, user_id, ts])
            n_successes += 1

        # Update the next time to send
        if user_id in self.user_id_to_next_image_day:
            while self.user_id_to_next_image_day[user_id] is not None and time.time() > self.users_to_configuration[user_id]['daily_schedule'][self.user_id_to_next_image_day[user_id]]['images_sent_timestamps'][self.user_id_to_next_image_i[user_id]]:
                self.user_id_to_next_image_i[user_id] += 1
                if self.user_id_to_next_image_i[user_id] >= len(self.users_to_configuration[user_id]['daily_schedule'][self.user_id_to_next_image_day[user_id]]['images_sent_timestamps']):
                    self.user_id_to_next_image_i[user_id] = 0
                    self.user_id_to_next_image_day[user_id] += 1
                    if self.user_id_to_next_image_day[user_id] >= len(self.users_to_configuration[user_id]['daily_schedule']):
                        self.user_id_to_next_image_day[user_id] = None
                        self.user_id_to_next_image_i[user_id] = None
            if self.user_id_to_next_image_day[user_id] is None:
                next_image_timestamp = None
            else:
                next_image_timestamp = self.users_to_configuration[user_id]['daily_schedule'][self.user_id_to_next_image_day[user_id]]['images_sent_timestamps'][self.user_id_to_next_image_i[user_id]]
            self.sent_messages_database.add_user_send_image_time(user_id, next_image_timestamp)
            self.database_updated()

        return n_successes

    def save_images(self, images_bytes, image_ids):
        """
        Saves the images
        """
        image_filepaths = []
        for i in range(len(images_bytes)):
            image_bytes = images_bytes[i]
            image_id = image_ids[i]
            image_cv2 = cv2.imdecode(np.fromstring(image_bytes, np.uint8), cv2.IMREAD_COLOR)
            filepath = os.path.join(self.images_base_dir, str(image_id)+".jpg")
            cv2.imwrite(filepath, image_cv2)
            image_filepaths.append(filepath)
        return image_filepaths

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
        for i in range(len(request.json['images'])):
            image = request.json['images'][i]
            image_bytes = base64.decodebytes(image.encode('ascii')) # image.encode("utf-8") #
            images_bytes.append(image_bytes)
        user = request.json['user']
        user_id = self.users[user]

        image_ids = self.get_image_ids(images_bytes)
        image_filepaths = self.save_images(images_bytes, image_ids)

        # Get a URL to the image, and remove failed URLs
        image_ids_to_return = []
        image_urls = self.get_image_urls(images_bytes, image_ids)
        logging.info("image_ids %s image_urls %s" % (image_ids, image_urls))

        # Save it in the CSV
        csv_filepath = os.path.join(self.data_base_dir, self.received_images_csv[0])
        with open(csv_filepath, "a") as f:
            csv_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            for i in range(len(image_ids)):
                image_id = image_ids[i]
                image_url = image_urls[i]
                image_filepath = image_filepaths[i]
                csv_writer.writerow([time.time(), image_filepath, image_id, "" if image_url is None else image_url, user_id, user])

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

        if 'image_descriptions' in request.json and len(request.json['image_descriptions']) == len(image_ids):
            image_descriptions = request.json['image_descriptions']
        else:
            image_descriptions = [None for _ in range(len(image_ids))]

        n_successes = self.send_images_to_slack(image_ids, image_urls, user_id, image_descriptions)

        response = self.flask_app.response_class(
            response=json.dumps({'image_ids':image_ids_to_return, 'n_successes':n_successes}),
            status=200,
            mimetype='application/json'
        )
        return response

    def get_num_users(self):
        """
        GET method at the endpoint /get_num_users
        Returns a json payload with keys:
        - 'num_users' with the number of users
        - 'user_to_learning_condition' where the keys are ints in [0, num_users)
          and the values are 0 if that user is not in the learning condition and
          1 otherwise

        """
        user_to_learning_condition = {}
        for i in range(len(self.users)):
            user_id = self.users[i]
            learning_condition = self.users_to_configuration[user_id]['learning_condition']
            user_to_learning_condition[i] = learning_condition
        response = self.flask_app.response_class(
            response=json.dumps({
                'num_users': len(self.users),
                'user_to_learning_condition': user_to_learning_condition,
            }),
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
    def action_button_check_mark(self, body, ack, say, action, respond):
        """
        Bolt App callback for when the user clicks the :check_mark: button on
        a message
        """
        # Acknowledge the action
        ack()
        self.recv_reaction(body, respond, 1)

    def action_button_x(self, body, ack, say, action, respond):
        """
        Bolt App callback for when the user clicks the :x: button on a message
        """
        # Acknowledge the action
        ack()
        self.recv_reaction(body, respond, 0)

    def body_to_image_url(self, body):
        """
        Given a Slackbot message body, return the first image_url in the
        message if it exists, else None.
        TODO: this can be hardcoded once we fix a particular template message
        """
        if "message" not in body: return None
        if "blocks" not in body["message"]: return None
        for block in body["message"]["blocks"]:
            if "type" not in block: continue
            if block["type"] == "image":
                if "image_url" not in block: continue
                return block["image_url"]
        return None

    def recv_reaction(self, body, respond, reaction):
        """
        Stores the user's reaction.
        """
        # Get the user_id and ts
        user_id = body["user"]["id"]
        ts = body["container"]["message_ts"]
        image_url = self.body_to_image_url(body)

        # Send the followup question
        response = slack_templates.action_button_check_mark_or_x(body, user_id, self.users_to_configuration[user_id]['expression_of_curiosity_condition'], reaction)
        respond(response)
        # response = self.slack_app.client.chat_update(channel=body["container"]["channel_id"], ts=ts, **response)
        # if not response["ok"]:
        #     logging.info("Error sending file to user %s: %s" % (user_id, response))

        # Save it in the CSV
        csv_filepath = os.path.join(self.data_base_dir, self.reactions_csv[0])
        with open(csv_filepath, "a") as f:
            csv_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            csv_writer.writerow([time.time(), image_url, user_id, ts, reaction])

        if user_id in self.users:
            logging.info('Got reaction %s from user %s for message at ts %s' % (reaction, user_id, ts))

            # Increment the appropriate values
            self.sent_messages_database.add_reaction(user_id, ts, reaction)
            self.database_updated()
        else:
            logging.info("Got button click reaction from user_id %s not in self.users, ignoring" % user_id)

    def body_to_question_answer(self, body):
        """
        Given a Slackbot message body, return the first input field's question
        and answer if it exists, else None.
        TODO: this can be hardcoded once we fix a particular template message
        TODO: this is horrible style, TBH I should use a try/except block to
        make this more readable. Same with body_to_image_url
        """
        if "message" not in body: return None, None
        if "blocks" not in body["message"]: return None, None
        for block in body["message"]["blocks"]:
            if "type" not in block: continue
            if block["type"] == "input":
                if "block_id" not in block: continue
                block_id = block["block_id"]
                if "label" not in block or "text" not in block["label"]:
                    continue
                question = block["label"]["text"]
                if "state" not in body or "values" not in body["state"] or block_id not in body["state"]["values"] or "plain_input" not in body["state"]["values"][block_id] or "value" not in body["state"]["values"][block_id]["plain_input"]:
                    continue
                answer = body["state"]["values"][block_id]["plain_input"]["value"]
                return question, answer
        return None, None

    def confirm_input(self, body, ack, say, action, respond):
        """
        Bolt App callback for when the user clicks the confirm button on their input
        """
        ack()
        # Get the user_id and ts
        user_id = body["user"]["id"]
        ts = body["container"]["message_ts"]
        image_url = self.body_to_image_url(body)

        question, answer = self.body_to_question_answer(body)
        if question is None: question = ""
        if answer is None: answer = ""

        # Update the blocks
        response = slack_templates.confirm_input_template(body, user_id, question, answer)
        # respond(response)
        response = self.slack_app.client.chat_update(channel=body["container"]["channel_id"], ts=ts, **response)
        if not response["ok"]:
            logging.info("Error sending file to user %s: %s" % (user_id, response))

        # Save it in the CSV
        csv_filepath = os.path.join(self.data_base_dir, self.followups_csv[0])
        with open(csv_filepath, "a") as f:
            csv_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            csv_writer.writerow([time.time(), image_url, user_id, ts, question, answer])

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
        logging.info("test_get_images")
        # Store that the user requested images.
        user_id = command["user_id"]
        if user_id in self.users:
            self.sent_messages_database.add_user_send_image_time(user_id, time.time())
            self.database_updated()
        else:
            logging.info("Got /test_get_images from user_id %s not in self.users, ignoring" % user_id)

    def test_end_of_day_message_0(self, ack, say, command, event, respond):
        """
        Bolt App callback for when the user types /test_end_of_day_message_0 into the
        app.
        """
        # Acknowledge the command
        ack()
        logging.info("test_end_of_day_message_0")
        day = 0
        user_id = command["user_id"]
        self.send_end_of_day_message(user_id, day)

    def test_end_of_day_message_1(self, ack, say, command, event, respond):
        """
        Bolt App callback for when the user types /test_end_of_day_message_1 into the
        app.
        """
        # Acknowledge the command
        ack()
        logging.info("test_end_of_day_message_1")
        day = 1
        user_id = command["user_id"]
        self.send_end_of_day_message(user_id, day)

    def test_end_of_day_message_2(self, ack, say, command, event, respond):
        """
        Bolt App callback for when the user types /test_end_of_day_message_2 into the
        app.
        """
        # Acknowledge the command
        ack()
        logging.info("test_end_of_day_message_2")
        day = 2
        user_id = command["user_id"]
        self.send_end_of_day_message(user_id, day)

    def send_end_of_day_message(self, user_id, day, timestamp=None):
        # Assign a unique random ID
        sent = False
        if user_id in self.users:
            random_id = self.sent_messages_database.get_random_id()
            payload, survey_url = slack_templates.survey_template(user_id, random_id, day)

            self.sent_messages_database.add_random_id(user_id, random_id, survey_url)
            self.database_updated()

            # Send the message
            if timestamp is None or timestamp <= time.time() + 10:
                response = self.slack_app.client.chat_postMessage(**payload)
                if not response["ok"]:
                    logging.info("Error sending survey message to user %s: %s, timestamp %s time.time() %s" % (user_id, response, timestamp, time.time()))
                    return sent
                ts = response["message"]["ts"]
                self.sent_messages_database.add_sent_survey(user_id, ts, survey_url)
                self.database_updated()
                sent = True
            else:
                response = self.slack_app.client.chat_scheduleMessage(post_at=timestamp, **payload)
                if not response["ok"]:
                    logging.info("Error schedule-sending daily intro message to user %s: %s" % (user_id, response))
                    return sent
                ts = timestamp
                sent = True

            # Save it in the CSV
            csv_filepath = os.path.join(self.data_base_dir, self.survey_csv[0])
            with open(csv_filepath, "a") as f:
                csv_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                csv_writer.writerow([time.time(), random_id, survey_url, day, user_id, ts])
        else:
            logging.info("Got /test_end_of_day_message from user_id %s not in self.users, ignoring" % user_id)
        return sent

    def send_daily_intro_message_0(self, ack, say, command, event, respond):
        """
        Bolt App callback for when the user types /send_daily_intro_message_0 into the
        app.
        """
        # Acknowledge the command
        ack()
        logging.info("send_daily_intro_message_0")
        day = 0
        user_id = command["user_id"]
        self.send_daily_intro_message(user_id, day)

    def send_daily_intro_message_1(self, ack, say, command, event, respond):
        """
        Bolt App callback for when the user types /send_daily_intro_message_1 into the
        app.
        """
        # Acknowledge the command
        ack()
        logging.info("send_daily_intro_message_1")
        day = 1
        user_id = command["user_id"]
        self.send_daily_intro_message(user_id, day)

    def send_daily_intro_message_2(self, ack, say, command, event, respond):
        """
        Bolt App callback for when the user types /send_daily_intro_message_2 into the
        app.
        """
        # Acknowledge the command
        ack()
        logging.info("send_daily_intro_message_2")
        day = 2
        user_id = command["user_id"]
        self.send_daily_intro_message(user_id, day)

    def send_daily_intro_message(self, user_id, day, timestamp=None):
        if user_id in self.users:
            payload = slack_templates.intro_template(user_id, day)

            # Send the message
            if timestamp is None or timestamp <= time.time() + 10:
                response = self.slack_app.client.chat_postMessage(**payload)
                if not response["ok"]:
                    logging.info("Error sending daily intro message to user %s: %s, timestamp %s time.time() %s" % (user_id, response, timestamp, time.time()))
                    return False
                ts = response["message"]["ts"]
                self.sent_messages_database.add_intro_message(user_id, ts)
                self.database_updated()
                return True
            else:
                response = self.slack_app.client.chat_scheduleMessage(post_at=timestamp, **payload)
                if not response["ok"]:
                    logging.info("Error schedule-sending daily intro message to user %s: %s" % (user_id, response))
                    return False
                return True
        else:
            logging.info("Got /send_daily_intro_message from user_id %s not in self.users, ignoring" % user_id)
        return False

    def send_pre_study_message(self, user_id, timestamp=None):
        """
        if timestamp is None, send immediately. Else, schedule.
        """
        if user_id in self.users:
            payload = slack_templates.pre_study_template(user_id)

            # Send the message
            if timestamp is None or timestamp <= time.time() + 10:
                response = self.slack_app.client.chat_postMessage(**payload)
                if not response["ok"]:
                    logging.info("Error sending pre-study message to user %s: %s, timestamp %s time.time() %s" % (user_id, response, timestamp, time.time()))
                    return False
                ts = response["message"]["ts"]
                self.sent_messages_database.add_pre_study_message(user_id, ts)
                self.database_updated()
                return True
            else:
                response = self.slack_app.client.chat_scheduleMessage(post_at=timestamp, **payload)
                if not response["ok"]:
                    logging.info("Error schedule-sending pre-study message to user %s: %s" % (user_id, response))
                    return False
                return True
        else:
            logging.info("Got send_pre_study_message from user_id %s not in self.users, ignoring" % user_id)
        return False

    def test_pre_study_message(self, ack, say, command, event, respond):
        """
        Bolt App callback for when the user types /test_pre_study_message into the
        app.
        """
        # Acknowledge the command
        ack()
        logging.info("test_pre_study_message")

        user_id = command["user_id"]
        self.send_pre_study_message(user_id)

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
    server = FlaskSlackbot(slackbot_conf)
    server.start()
