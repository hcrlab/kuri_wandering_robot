import requests, json, base64, time, threading, sys, cv2
import numpy as np
from flask import Flask, render_template, send_file, request, redirect

# Configuration Variables
server_url = "http://localhost:8194" # the URL the Slackbot server is running on

# Test low_battery without an image
print("Testing /low_battery without an image...", end='')
dict_to_send = {'battery_pct':25}
res = requests.post(server_url+'/low_battery', json=dict_to_send)
print("Got response JSON from /low_battery: %s" % res.json())

# Load an image
with open("../imgs/test_image_0.jpg", "rb") as f:
    image_content = f.read()
    image_to_send = base64.encodebytes(image_content).decode('ascii')

# Test low_battery with an image
print("Testing /low_battery with an image...", end='')
dict_to_send = {'battery_pct':15, 'image':image_to_send}
res = requests.post(server_url+'/low_battery', json=dict_to_send)
print("Got response JSON from /low_battery: %s" % res.json())

# Load another image
with open("../imgs/test_image_1.jpg", "rb") as f:
    image_content = f.read()
    image_to_send = base64.encodebytes(image_content).decode('ascii')

# Test where_am_i
print("Testing /where_am_i...", end='')
dict_to_send = {'image':image_to_send, 'options':['Lounge', "Office#252", "200 Corridoor", "Atrium"]}
res = requests.post(server_url+'/where_am_i', json=dict_to_send)
print("Got response JSON from /where_am_i: %s" % res.json())
where_am_i_message_id = res.json()['message_id']

# Sleep to enable the user to respond
print("Sleeping for 15 seconds. In this time, respond to the Slack message.")
sleep_secs = 15
time.sleep(sleep_secs)

# Test get_updates without message_ids
print("Testing get_updates without message_ids...", end='')
dict_to_send = {}
res = requests.post(server_url+'/get_updates', json=dict_to_send)
print("Got response JSON from /get_updates: %s" % res.json())

# Test get_updates with message_ids and a timestamp of 0
print("Testing get_updates with a message_id and timestamp of 0...", end='')
dict_to_send = {'message_ids_and_action_ts' : {where_am_i_message_id:0}}
res = requests.post(server_url+'/get_updates', json=dict_to_send)
print("Got response JSON from /get_updates: %s" % res.json())

if where_am_i_message_id in res.json()['message_id_to_responses']:
    where_am_i_latest_action_ts = float(res.json()['message_id_to_responses'][where_am_i_message_id][-1][0])

    # Sleep to enable the user to respond
    print("Sleeping for 5 seconds. In this time, respond to the Slack message if applicable (e.g., you clicked 'Other').")
    sleep_secs = 5
    time.sleep(sleep_secs)

    # Test get_updates with message_ids and a timestamp of where_am_i_latest_action_ts
    print("Testing get_updates with a message_id and timestamp of %f..." % where_am_i_latest_action_ts, end='')
    dict_to_send = {'message_ids_and_action_ts' : {where_am_i_message_id : where_am_i_latest_action_ts}}
    res = requests.post(server_url+'/get_updates', json=dict_to_send)
    print("Got response JSON from /get_updates: %s" % res.json())
