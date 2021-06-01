import requests, json, base64, time, threading, sys
from flask import Flask, render_template, send_file, request, redirect

# Configuration Variables
server_url = "http://localhost:8194" # the URL the Slackbot server is running on

# test get_num_users
print("Testing get_num_users...", end='')
res = requests.get(server_url+'/get_num_users')
print("got %d users. Is this the response you were expecting?" % res.json()["num_users"])

# Load the image content
print("Loading image...", end='')
filepath = "../imgs/test_image.jpg"
with open(filepath, "rb") as f:
    content = f.read()
print("done!")

# test send_image
users_to_send_to = [0]
print("Testing send_image to users %s..." % users_to_send_to, end='')
dictToSend = {'image':base64.encodebytes(content).decode('ascii'), 'users':users_to_send_to}
res = requests.post(server_url+'/send_image', json=dictToSend)
assert(len(users_to_send_to) == res.json()["num_sent"])
message_id = res.json()["message_id"]
print("sent to all users. Got message_id %s. Is this the response you were expecting?" % message_id)

# Sleep
sleep_secs = 10
print("Sleeping for %d seconds waiting for a reaction..." % sleep_secs)
time.sleep(sleep_secs)

# test get_responses
print("Testing get_responses for message_id %s..." % message_id, end='')
message_ids_and_user_ids = {message_id:[0]}
res = requests.post(server_url+'/get_responses', json={'message_ids_and_user_ids':message_ids_and_user_ids})
message_id_to_user_reactions_from_server = res.json()["message_id_to_user_reactions"]
assert(len(message_ids_and_user_ids) == len(message_id_to_user_reactions_from_server))
print("Got message_id_to_user_reactions_from_server %s. The key is the message id, the first element in the list is the user id, and the second element in the list is their reaction. Is this what you were expecting?" % message_id_to_user_reactions_from_server)
print("Done!")
