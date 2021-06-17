import requests, json, base64, time, threading, sys, cv2
import numpy as np
from flask import Flask, render_template, send_file, request, redirect

# Configuration Variables
server_url = "http://localhost:8194" # the URL the Slackbot server is running on

# # test get_num_users
# print("Testing get_num_users...", end='')
# res = requests.get(server_url+'/get_num_users')
# print("got %d users. Is this the response you were expecting?" % res.json()["num_users"])
#
# # Sleep
# sleep_secs = 15
# print("Sleeping for %d seconds. In this time, go to Slack, and direct message KuriBot /test_get_images" % sleep_secs)
# time.sleep(sleep_secs)
#
# # Test get_updates
# print("Testing get_updates...", end='')
# dict_to_send = {'image_ids_and_users' : {}}
# res = requests.post(server_url+'/get_updates', json=dict_to_send)
# print("Got time_to_send %s, where the number is the remaining seconds before sending images. Is this the response you were expecting?" % res.json()["time_to_send"])

# Load the image content
print("Loading image...", end='')
image_filepaths = ["../imgs/test_image_%d.jpg" % i for i in range(2)]
image_contents = []
for image_filepath in image_filepaths:
    with open(image_filepath, "rb") as f:
        raw_bytes = f.read()
        image_contents.append(raw_bytes)

#         encoded_bytes = base64.encodebytes(raw_bytes).decode('ascii')
#         decoded_bytes = base64.decodebytes(encoded_bytes.encode('ascii'))
#         print("raw_bytes == decoded_bytes", raw_bytes == decoded_bytes)
#
#         image_array = np.fromstring(raw_bytes, np.uint8)
#         img_cv2 = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
#         cv2.imshow("test_window", img_cv2)
#         cv2.waitKey(0)
#
#         image_array = np.fromstring(decoded_bytes, np.uint8)
#         img_cv2 = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
#         cv2.imshow("test_window", img_cv2)
#         cv2.waitKey(0)
# cv2.destroyAllWindows()
print("done!")

# Test send_images
user_to_send_to = 0
print("Testing send_images to user %s..." % user_to_send_to, end='')
images_to_send = []
for image_content in image_contents:
    # print("type(image_content)", type(image_content))
    # print("type(base64.encodebytes(image_content))", type(base64.encodebytes(image_content)))
    # print("type(base64.encodebytes(image_content)).decode('ascii')", type(base64.encodebytes(image_content).decode('ascii')))
    images_to_send.append(base64.encodebytes(image_content).decode('ascii'))
    # images_to_send.append(image_content.decode('utf-8'))
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
