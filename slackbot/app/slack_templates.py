import copy
import logging
import pprint
import random

# See Slack's Block Kit Builder for more: https://app.slack.com/block-kit-builder/

def low_battery_template(user_id, battery_pct, image_url=None):
    payload = {
        "ts": "",
        "channel": user_id,
        "username": "kuribot",
        "icon_emoji": ":robot_face:",
        "text": "Hi, it's Kuribot! I am at low battery. Can you help put me on my charger?",
        "blocks": [
    	{
    		"type": "section",
    		"text": {
    			"type": "mrkdwn",
    			"text": ":rotating_light: :warning: My battery is low (*" + str(battery_pct) + "%*)!! " + ("Here is a picture from where I am. " if image_url is not None else "") + "Please come and put me on my charger! :warning: :rotating_light: "
    	           }
        }
        ]
    }
    if image_url is not None:
        payload["blocks"].insert(0, {
    		"type": "image",
    		"image_url": image_url,
    		"alt_text": "This is an image of what Kuribot sees.",
    	})
    return payload

def where_am_i_template(user_id, image_url, options):
    payload = {
        "ts": "",
        "channel": user_id,
        "username": "kuribot",
        "icon_emoji": ":robot_face:",
        "text": "Hi, it's Kuribot! I'm lost. Can you help tell me where I am?",
        "blocks": [
        {
    		"type": "image",
    		"image_url": image_url,
    		"alt_text": "This is an image of what Kuribot sees.",
    	},
        {
    		"type": "section",
    		"text": {
    			"type": "mrkdwn",
    			"text": "I'm lost. :disappointed: This is an image of what I see. :eyes: Can you please tell me where I am?:round_pushpin::world_map:"
    		}
    	},
    	{
    		"type": "actions",
    		"elements": [
    			{
    				"type": "button",
    				"text": {
    					"type": "plain_text",
    					"text": button_label
    				},
    				"value": button_label,
    				"action_id": "button_click_"+button_label.lower()
    			} for button_label in options+["Other"]
    		]
		},
	   ],
    }
    return payload


def got_button_click(body, user_id, button_label):
    blocks = [
        {
            "type": "section",
            "text": {
                "type": "mrkdwn",
                "text": "Thanks! You chose "+ button_label + "."
            }
        },
    ]
    if button_label == "Other":
        blocks.append(
            {
                "type": "input",
                "label":{
                    "type": "plain_text",
                    "text": "Can you please type in the name of the location I'm in or near?",
                    "emoji": True
                },
                "element":{
                    "type": "plain_text_input",
                    "action_id": "plain_input",
                    "multiline": True
                },
            }
        )
        blocks.append(
            {
                "type": "actions",
        		"elements": [
        			{
        				"type": "button",
        				"text": {
        					"type": "plain_text",
        					"text": "Submit"
        				},
                        "style": "primary",
        				"value": "submit",
        				"action_id": "submit_input"
        			}
                ]
            }
        )

    response={
        "blocks" : blocks
    }

    if "message" in body and "blocks" in body["message"]:
        # Add the same first two blocks as the original message
        response["blocks"].insert(0, copy.deepcopy(body["message"]["blocks"][0]))
        response["blocks"].insert(1, copy.deepcopy(body["message"]["blocks"][1]))
        # Remove any elements that were added after sending the message, otherwise
        # it will not render.
        del response["blocks"][0]['block_id']
        del response["blocks"][0]['fallback']
        del response["blocks"][0]['image_bytes']
        del response["blocks"][0]['image_height']
        del response["blocks"][0]['image_width']
        del response["blocks"][1]['block_id']

    return response

def got_submit_input(body, user_id, answer):
    response = {
        "blocks" : []
    }

    if "message" in body and "blocks" in body["message"]:
        # Add the same first three blocks as the original message
        response["blocks"].insert(0, copy.deepcopy(body["message"]["blocks"][0]))
        response["blocks"].insert(1, copy.deepcopy(body["message"]["blocks"][1]))
        response["blocks"].insert(2, copy.deepcopy(body["message"]["blocks"][2]))
        # Remove any elements that were added after sending the message, otherwise
        # it will not render.
        del response["blocks"][0]['block_id']
        del response["blocks"][0]['fallback']
        del response["blocks"][0]['image_bytes']
        del response["blocks"][0]['image_height']
        del response["blocks"][0]['image_width']
        del response["blocks"][1]['block_id']
        del response["blocks"][2]['block_id']
        # Modify the confirmation message
        response["blocks"][2]['text']['text'] = response["blocks"][2]['text']['text'][:-1] + ", and typed \"" + answer + "\""

    return response

def get_button_click_metadata(body):
    """
    Extract relevant metadata from the body message included with a button
    click action.

    :param body: (dict) the body of the json payload, including details
                 about the message and the button clicked
    :returns: user_id (str), the user who clicked the button
    :returns: ts (str), the timestamp of the original message
    :returns: image_url (str), the image_url of the message if it exists
    :returns: button_label (str), the text on the button
    :returns: action_ts (str), the timestamp of the button click.
    """
    # pprint.pprint(body)
    try:
        user_id = body['user']['id']
    except KeyError as e:
        logging.info("No user_id in button click body %s" % body)
        logging.info("Error %s." % e)
        user_id = None

    try:
        ts = body['container']['message_ts']
    except KeyError as e:
        logging.info("No message_ts in button click body %s" % body)
        logging.info("Error %s." % e)
        ts = None

    try:
        for block in body["message"]["blocks"]:
            if "type" not in block: continue
            if block["type"] == "image":
                if "image_url" not in block: continue
                image_url = block["image_url"]
                break
    except KeyError as e:
        logging.info("No image_url in button click body %s" % body)
        logging.info("Error %s." % e)
        image_url = None

    try:
        button_label = body['actions'][0]['text']['text']
    except KeyError as e:
        logging.info("No button_label in button click body %s" % body)
        logging.info("Error %s." % e)
        button_label = None

    try:
        action_ts = body['actions'][0]['action_ts']
    except KeyError as e:
        logging.info("No action_ts in button click body %s" % body)
        logging.info("Error %s." % e)
        action_ts = None

    return user_id, ts, image_url, button_label, action_ts


def get_submit_input_metadata(body):
    """
    Extract relevant metadata from the body message included with a submit
    input action.

    :param body: (dict) the body of the json payload, including details
                 about the message and the button clicked
    :returns: user_id (str), the user who clicked the button
    :returns: ts (str), the timestamp of the original message
    :returns: image_url (str), the image_url of the message if it exists
    :returns: question (str), the question for the button
    :returns: response (str), the response from the button
    :returns: action_ts (str), the timestamp of the button click.
    """
    pprint.pprint(body)
    try:
        user_id = body['user']['id']
    except KeyError as e:
        logging.info("No user_id in submit input body %s" % body)
        logging.info("Error %s." % e)
        user_id = None

    try:
        ts = body['message']['ts']
    except KeyError as e:
        logging.info("No message_ts in submit input body %s" % body)
        logging.info("Error %s." % e)
        ts = None

    try:
        for block in body["message"]["blocks"]:
            if "type" not in block: continue
            if block["type"] == "image":
                if "image_url" not in block: continue
                image_url = block["image_url"]
                break
    except KeyError as e:
        logging.info("No image_url in button click body %s" % body)
        logging.info("Error %s." % e)
        image_url = None

    try:
        question = body["message"]['blocks'][3]['label']['text']
    except KeyError as e:
        logging.info("No question in button click body %s" % body)
        logging.info("Error %s." % e)
        question = None

    try:
        response = body['state']['values'][body["message"]['blocks'][3]["block_id"]]['plain_input']['value']
    except KeyError as e:
        logging.info("No response in button click body %s" % body)
        logging.info("Error %s." % e)
        response = None

    try:
        action_ts = body['actions'][0]['action_ts']
    except KeyError as e:
        logging.info("No action_ts in button click body %s" % body)
        logging.info("Error %s." % e)
        action_ts = None

    return user_id, ts, image_url, question, response, action_ts
