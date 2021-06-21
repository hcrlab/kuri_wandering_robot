import requests, json, base64, time, threading, sys, cv2
import numpy as np
from flask import Flask, render_template, send_file, request, redirect

# Configuration Variables
server_url = "http://localhost:8194" # the URL the Slackbot server is running on

# test get_num_users
print("Testing get_num_users...", end='')
res = requests.get(server_url+'/get_num_users')
print("got %d users. Is this the response you were expecting?" % res.json()["num_users"])

# Sleep
sleep_secs = 15
print("Sleeping for %d seconds. In this time, go to Slack, and direct message KuriBot /test_get_images" % sleep_secs)
time.sleep(sleep_secs)

# Test get_updates
print("Testing get_updates...", end='')
dict_to_send = {'image_ids_and_users' : {}}
res = requests.post(server_url+'/get_updates', json=dict_to_send)
print("Got time_to_send %s, where the number is the remaining seconds before sending images. Is this the response you were expecting?" % res.json()["time_to_send"])

# Load the image content
print("Loading image...", end='')
image_filepaths = ["../imgs/test_image_%d.jpg" % i for i in range(2)]
image_contents = []
for image_filepath in image_filepaths:
    with open(image_filepath, "rb") as f:
        raw_bytes = f.read()
        image_contents.append(raw_bytes)
print("done!")

# Test send_images
user_to_send_to = 0
print("Testing send_images to user %s..." % user_to_send_to, end='')
images_to_send = []
for image_content in image_contents:
    images_to_send.append(base64.encodebytes(image_content).decode('ascii'))
dict_to_send = {'images':images_to_send, 'user':user_to_send_to}
res = requests.post(server_url+'/send_images', json=dict_to_send)
image_ids = res.json()["image_ids"]
assert(len(image_filepaths) == len(image_ids))
print("sent to all users. Got image_ids %s. Is this the response you were expecting?" % image_ids)

# Sleep
sleep_secs = 15
print("Sleeping for %d seconds. In this time, go to Slack, and react to the first image. After that, you should receive a second image. You can decide how/whether to react to it." % sleep_secs)
time.sleep(sleep_secs)

# Test get_updates
print("Testing get_updates for image_ids %s..." % image_ids, end='')
image_ids_and_users = {image_id:[user_to_send_to] for image_id in image_ids}
res = requests.post(server_url+'/get_updates', json={'image_ids_and_users':image_ids_and_users})
image_id_to_user_reactions = res.json()["image_id_to_user_reactions"]
print("Got image_id_to_user_reactions %s. The key is the image_id, the first element in the list is the user_id, and the second element in the list is their reaction. Is this what you were expecting?" % image_id_to_user_reactions)
print("Done!")
