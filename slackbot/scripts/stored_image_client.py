"""
This client first loads images stored at a specified filepath. It then listens
for updates from the server. When it gets a request for an update, it sends 5
random stored photos. It keeps doing this until it is terminated.
"""
import base64
import cv2
import numpy as np
import os
import random
import requests
import time

def load_images(folder_path):
    """
    Loads all the jpg images in folder_path as cv2 images
    """
    retval = {}
    for filename in os.listdir(folder_path):
        img_id, ext = os.path.splitext(filename)
        if ext.lower() in [".jpg", ".jpeg"]:#, ".png"]:
            retval[img_id] = cv2.imread(os.path.join(folder_path, filename))
    return retval

if __name__ == "__main__":
    # NEED TO CHANGE
    folder_path = "/Users/amaln/Documents/HCRLab/kuri_photography_cmm_demo/20210419_Kuri_Moving_Around_UW_CSE2/subsampling_policy_1/"
    server_url = "http://ec2-34-222-57-139.us-west-2.compute.amazonaws.com:8194"

    n_images = 5 # Num images to send

    # Load the images
    print("Loading images")
    images = load_images(folder_path)
    print("Loaded %d Images" % len(images))

    # Get the number of users
    res = requests.get(server_url+'/get_num_users')
    n_users = res.json()["num_users"]
    print("Established connection with server. Got %d users" % n_users)

    # Run the loop
    target_loop_duration = 10.0 # secs
    user_to_stored_images = {i : list(images.keys()) for i in range(n_users)}
    user_to_sent_images = {i : list() for i in range(n_users)}
    image_id_to_slackbot_image_id = {}
    while True:
        loop_start_time = time.time()

        # Get updates for all users
        image_ids_and_users = {}
        for user in user_to_sent_images:
            for image_id in user_to_sent_images[user]:
                slackbot_image_id = image_id_to_slackbot_image_id[image_id]
                if slackbot_image_id not in image_ids_and_users:
                    image_ids_and_users[slackbot_image_id] = []
                image_ids_and_users[slackbot_image_id].append(user)
        res = requests.post(server_url+'/get_updates', json={'image_ids_and_users':image_ids_and_users})
        time_to_send = res.json()["time_to_send"]
        print("%f: Got update %s" % (time.time(), res.json()))

        # For the users that need images, send them random unsent images
        for user in time_to_send:
            if time_to_send[user] < target_loop_duration:
                user = int(user)
                image_ids_to_send = []
                images_to_send = []
                for _ in range(n_images):
                    i = random.randint(0, len(user_to_stored_images[user])-1)
                    image_id = user_to_stored_images[user].pop(i)
                    image_ids_to_send.append(image_id)
                    images_to_send.append(base64.encodebytes(bytearray(np.array(cv2.imencode('.jpg', images[image_id])[1]).tostring())).decode('ascii'))
                    user_to_sent_images[user].append(image_id)
                dict_to_send = {'images':images_to_send, 'user':user}
                res = requests.post(server_url+'/send_images', json=dict_to_send)
                slackbot_image_ids = res.json()["image_ids"]
                for i in range(len(slackbot_image_ids)):
                    image_id = image_ids_to_send[i]
                    slackbot_image_id = slackbot_image_ids[i]
                    image_id_to_slackbot_image_id[image_id] = slackbot_image_id

        loop_duration = time.time() - loop_start_time
        if loop_duration < target_loop_duration:
            time.sleep(target_loop_duration - loop_duration)
