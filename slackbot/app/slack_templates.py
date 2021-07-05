import copy
import pprint
import random

"""
Types of messages to expect:
    At the beginning of the study:
        X Pre-study message introducing the task
    At the beginning of the day:
        X Intro message -- sent at 9AM daily
        X    Has a different “fun thing” every day
    At every 2 hour mark:
        X Intro message (e.g. “I’ll now send you 5 images I’ve taken that I think you’ll like”)
        X   Have this vary a bit, have ~5 variations that it randomly picks from
    For each of the 5 images:
        X Messages/blocks that go alongside the image
        X Update to the block when an emoji response button is clicked
        X Update to the block when the “confirm” followup button is clicked
        X Ending message (e.g., “I’ll send you more messages in 2 hours”)
        X ~5ish variations
    At the end of the day:
        X Sending the survey, a link to the qualtrics
        X Ideally, pass a random user ID via the Qualtrics URL (Should be easy :))
    Final message for final survey
        Assuming that final survey is different from end of day survey
"""
def pre_study_template(user_id):
    payload = {
        "ts": "",
        "channel": user_id,
        "username": "kuribot",
        "text": "Hi, it's Kuribot! I'm checking in before the study starts with some important information",
        "blocks": [
		{
			"type": "section",
			"text": {
				"type": "mrkdwn",
				"text": "Hi, it's KuriBot! I'm getting in touch with you to ensure everything is in-place for the study to start tomorrow."
			}
		},
		{
			"type": "section",
			"text": {
				"type": "mrkdwn",
				"text": "First off, I want to confirm that I have the right account. Did you sign up to participate in the \"Seeing the World Through the Eyes of a Robot\" study?"
			}
		},
		{
			"type": "section",
			"text": {
				"type": "mrkdwn",
				"text": "If so, then I've got the right person! If not, please contact the researchers running this study at cmavro@cs.washington.edu"
			}
		},
		{
			"type": "section",
			"text": {
				"type": "mrkdwn",
				"text": "As a reminder, I will be moving around the Gates Center sending you pictures of objects that I think you will like. I'm looking forward to sending you some pictures starting tomorrow!"
			}
		},
		{
			"type": "section",
			"text": {
				"type": "mrkdwn",
				"text": "Here is a welcome message from the researchers running this study: \n```\nWelcome to the \"Seeing the World Through the Eyes of a Robot\" study! I'm one of the researchers running the study. Now that you've heard from Kuri, I wanted to tell you more about the study you've signed up for.\n\n- You will be reciving images from Kuri as it moves around the Gates Center. This will continue for three days, starting tomorrow.\n- Each day at 9 AM you will receive an intro message from Kuri, to let you know that it has started. You will then receive four sets of images: at 10AM, 12PM, 2PM, and 4PM. These images are of objects that Kuri thinks you might like.\n- Kuri will ask for your feedback by asking you to click a 'check' or 'x' button, and may also ask you to respond to open-ended followup questions. You can only interact with Kuri in the above ways; Kuri will not understand if you send it a typical message.\n- It is your choice whether you respond to Kuri's requests for feedback. You can respond at anytime that is convenient for you, and it won't hurt Kuri's feelings if you don't respond. \n- If you decide to respond, be sure to read what Kuri is asking you. \n- You will receive four links to surveys over the course of this study. To receive full compensation, you *must* complete all four surveys.\n- Kuri's interaction is designed for Slack's desktop or browser apps -- it may not work correctly on the mobile app.\n- If you have any questions, or you have received these messages without signing up for the study, please contact cmavro@cs.washington.edu. \n\nWe hope you enjoy interacting with Kuri!\n```"
			}
		}
	]
    }
    return payload

def intro_template(user_id, day):
    #The intro message accepts an int that describes what day the participant is on
    #Expected values: 0-2
    #User_id is also needed to post to the correct channel
    if day < 0 or day > 3:
        new_day = abs(day) % 3
        print("intro_template got incorrect day %d, setting to %d" % (day, new_day))
        day = new_day
    if day == 0:
        img_link = "https://amalnanavati.com/wp-content/uploads/2021/06/optimized_kuri_dance.gif"
        message_text = "It's the first day! I'm very excited to work with you and I want to show you a dance I've been working on. I hope you like it."
        alt_text = "My big dance"
    elif day == 1:
        img_link = "https://www.cnet.com/a/img/JkD_3wBEktye-EV2e3G8OdbSGSI=/770x578/2017/01/03/026e660f-4525-4bc8-94f3-9a5e9c61a776/kuriproductphotos-4.jpg"
        message_text = "I'm looking foward to starting the second day! Here's a little token of my appriciation"
        alt_text = "Thanks"
    elif day == 2:
        img_link = "https://thegadgetflow.com/wp-content/uploads/2017/01/Kuri-Intelligent-Home-Robot-004.jpg"
        message_text = "This is the last day, I hope you are ready to finish strong! *Pay extra attention today*, as I'm going to try out a few new things. Here is a note from the researchers running the study: \n\n```\nIt's the last day! Today, you will receive two surveys: one at 1PM and one at 4:45PM. Please fill out the first survey *before* interacting with any of the messages Kuri sends you after 1PM.```\n\nHere's something that might inspire you on this final day. "
        alt_text = "Party time!"
    payload = {
        "ts": "",
        "channel": user_id,
        "username": "kuribot",
        "text": "Hi, it's Kuribot! I'm going to be starting soon. Get ready for some images.",
        "blocks": [
        # {
        #     "type": "section",
        #     "text": {
        #         "type": "mrkdwn",
        #         "text": "Thanks for getting started, I'll send you a message soon after I get things set up."
        #     }
        # },
        {
            "type": "section",
            "text": {
                "type": "mrkdwn",
                "text": message_text
            }
        },
        {
            "type": "image",
            "image_url": img_link,
            "alt_text": alt_text,
        }
    ],
    }
    return payload

def post_images_intro(user_id, n_images):
    payload = {
        "ts": "",
        "channel": user_id,
        "username": "kuribot",
        "icon_emoji": ":robot_face:",
        "text": "Hi, it's Kuribot! I took an image I think you might like. Let me know what you think by clicking the buttons!",
        "blocks": [
            {
                "type": "section",
                "text": {
                    "type": "mrkdwn",
                    "text": "Hi there, it's Kuribot! I took %d images I think you might like." % n_images
                }
            },
        ],
    }
    return payload

def post_image(user_id, image_url, image_description, message_i, n_images, objects):
    #This posts an image given:
    #the user ID: user_id
    #an existing link to that image: public_link
    #An image image_description
    #objects is a list where the top 5 are objects in the picture
    if image_description is None:
        image_description = "Kuri shared this picture with you!"

    photo_message_list= [
    "Please look at the photo. Respond with either a :white_check_mark: if you liked at least one object in the photo, or an :x: if you dislike all the objects in the photo. \n\nI believe these objects are in the photo: ",
    "Take a look at this photo. To tell me if you like or dislike the objects in this photo, please respond with either a :white_check_mark: or an :x:. \n\nI think I see the following objects in this photo: ",
    "Please help me out further by looking at this photo. If you dislike all the objects in this photo, click the :x:. If you like at least one, click the :white_check_mark:. \n\nI believe following objects are in this photo: ",
    "After taking a look at this photo, please respond based on the objects in it. If you like at least one object, please press :white_check_mark:. If you dislike all of them, please press :x:. \n\nI’ve tentatively identified the following objects in this photo: ",
    "Please examine this photo. If you like any objects in the photo, press :white_check_mark:. If you dislike all the objects in the photo, press :x:. \n\nI think these objects are in this photo: "
    ]

    payload = {
        "ts": "",
        "channel": user_id,
        "username": "kuribot",
        "icon_emoji": ":robot_face:",
        "text": "Hi, it's Kuribot! I took an image with some objects I think you might like. Let me know what you think by clicking the buttons!",
        "blocks": [
        {
    		"type": "section",
    		"text": {
    			"type": "mrkdwn",
    			"text": "Here is image %d/%d. " % (message_i+1, n_images) + photo_message_list[message_i % len(photo_message_list)] + str(objects)[1:-1].replace("'", "")
    		}
    	},
        {
    		"type": "image",
    		"image_url": image_url,
    		"alt_text": image_description,
    	},
    	{
    		"type": "actions",
    		"elements": [
    			{
    				"type": "button",
    				"text": {
    					"type": "plain_text",
    					"text": ":white_check_mark:"
    				},
    				"value": "check_mark",
    				"action_id": "action_id_check_mark"
    			},
    			{
    				"type": "button",
    				"text": {
    					"type": "plain_text",
    					"text": ":x:"
    				},
    				"value": "x_mark",
    				"action_id": "action_id_x"
    			}
    		]
		},
	   ],
    }
    return payload


def action_button_check_mark_or_x(body, user_id, expression_of_curiosity_condition, reaction):
    #user_id is the users_id
    #which user study condition: condition. Expected values are 0 (description), 1 (follow up 1), and 2 (follow up 2)
    #reaction is the type of reaction from the button they clicked. 1 is check mark, 0 is X
    print("expression_of_curiosity_condition", expression_of_curiosity_condition, repr(expression_of_curiosity_condition), "reaction", reaction)
    if expression_of_curiosity_condition == 0:
        message_list = [
        "Thank you, I'll use your response to choose better photos. Next, please describe the photo.",
        "Thank you, I've recorded your response which I will learn from. Next, please help me out by describing the photo.",
        "Thank you, I've added your feedback to my database. Next, describe the photo to help me understand what is in it.",
        "Thank you, I'm starting to understand your preferences better. Describe the photo so I can keep learning."
        ]
    elif expression_of_curiosity_condition == 1:
        message_list = [
        "Thank you, I'll use your response to choose better photos. Why did you rate the photo this way?",
        "Thank you, I've recorded your response which I will learn from. Why did you make that choice for the photo?",
        "Thank you, I've added your feedback to my database. I'd like to understand, why did you rate the photo this way?",
        "Thank you, I'm starting to understand your preferences better. I want to learn more about why you made that choice."
        ]
    elif expression_of_curiosity_condition is not None:
        if reaction == 1:
            prefix = ""
        else:
            prefix = "dis"
        message_list = [
        "Thank you, I'll use your response to choose better photos. I'd like to understand more about why you %sliked this photo. Was there an object in the photo that you %sliked? What was it?" % (prefix, prefix),
        "Thank you, I've recorded your response, which I will learn from. I want to learn why you %sliked this photo, was there a particular object you %sliked?" % (prefix, prefix),
        "Thank you, I've added your feedback to my database. If you tell me more about objects you %sliked in the photo, I can learn more efficiently."  % (prefix),
        "Thank you, I'm starting to understand your preferences better. Can you explain more about why you %sliked this photo? Any objects that you %sliked?" % (prefix, prefix)
        ]

    if reaction == 1:
        emoji = ":white_check_mark:"
    else:
        emoji = ":x:"

    blocks = [
        {
            "type": "section",
            "text": {
                "type": "mrkdwn",
                "text": "You chose "+ emoji
            }
        },
    ]
    if expression_of_curiosity_condition is not None:
        message = message_list[random.randrange(len(message_list))]
        blocks.append(
            {
                "type": "input",
                "label":{
                    "type": "plain_text",
                    "text": message,
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
        					"text": "Confirm"
        				},
                        "style": "primary",
        				"value": "confirm",
        				"action_id": "confirm_input"
        			}
                ]
            }
        )

    response={
        # "replace_original" : True,
        "blocks" : blocks
    }

    if "message" in body and "blocks" in body["message"]:
        # Add the same first two blocks as the original message
        response["blocks"].insert(0, copy.deepcopy(body["message"]["blocks"][0]))
        response["blocks"].insert(1, copy.deepcopy(body["message"]["blocks"][1]))
        # Remove any elements that were added after sending the message, otherwise
        # it will not render.
        del response["blocks"][0]['block_id']
        del response["blocks"][1]['block_id']
        del response["blocks"][1]['fallback']
        del response["blocks"][1]['image_bytes']
        del response["blocks"][1]['image_height']
        del response["blocks"][1]['image_width']
    # pprint.pprint(response)

    return response

def confirm_input_template(body, user_id, question, answer):
    #user_input is recovered at the previous step, from something like
        #body["message"]["blocks"][0]
    #user_id is the user's id for correct placement
    response = {
        "blocks": [
        {
            "type": "section",
            "text": {
                "type": "mrkdwn",
                "text": "*"+ question+"*"
                }
        },
        {
            "type": "section",
            "text": {
                "type": "mrkdwn",
                "text": "You wrote: \""+ answer + "\""
                }
        },
        ]
    }

    if "message" in body and "blocks" in body["message"]:
        # Add the same first two blocks as the original message
        response["blocks"].insert(0, copy.deepcopy(body["message"]["blocks"][0]))
        response["blocks"].insert(1, copy.deepcopy(body["message"]["blocks"][1]))
        response["blocks"].insert(2, copy.deepcopy(body["message"]["blocks"][2]))
        # Remove any elements that were added after sending the message, otherwise
        # it will not render.
        del response["blocks"][0]['block_id']
        del response["blocks"][1]['block_id']
        del response["blocks"][1]['fallback']
        del response["blocks"][1]['image_bytes']
        del response["blocks"][1]['image_height']
        del response["blocks"][1]['image_width']
        del response["blocks"][2]['block_id']

    return response

def survey_template(user_id, random_id, is_last_day, survey_i, order):
    #The study URL will need to be changed for the final version, but here's the temp one for testing
    #random_id will be generated beforehand and added to a dictionary to keep track of who got what random_id

    # if day < 0 or day > 2:
    #     new_day = abs(day) % 3
    #     print("survey_template got incorrect day %d, setting to %d" % (day, new_day))
    #     day = new_day
    if survey_i < 0 or survey_i > 1:
        new_survey_i = abs(survey_i) % 2
        print("survey_template got incorrect survey_i %d, setting to %d" % (survey_i, new_survey_i))
        survey_i = new_survey_i
    if is_last_day:
        if survey_i == 0:
            survey_url = "https://ucsantacruz.co1.qualtrics.com/jfe/form/SV_1HbJysAL45gK4XY?random_id=" + str(random_id) + "&order=" + str(order)
            message = "This is the first of two surveys you will receive today. Please go to the following URL to complete the survey, which should take around 5 minutes. *Please do not proceed with the study until you have completed the survey.* " + survey_url
        else:
            survey_url = "https://ucsantacruz.co1.qualtrics.com/jfe/form/SV_9uV0VU7U2epEviu?random_id=" + str(random_id) + "&order=" + str(order)
            message = "Thank you for interacting with me today! This is the last day of the study. Please go to the following URL to complete the final survey, which should take around 10 minutes. " + survey_url + "\n\nA researcher will email you within a week regarding compensation. Please email cmavro@cs.washington.edu if you have any questions/concerns."
    else:
        survey_url = "https://ucsantacruz.co1.qualtrics.com/jfe/form/SV_ekRfwRiSbx2SK9g?random_id=" + str(random_id)
        message = "Thank you for interacting with me today! Before you finish up for the day, please go to the following URL to complete a survey. The survey should take around 5 minutes. " + survey_url

    payload = {
        "ts": "",
        "channel": user_id,
        "username": "kuribot",
        "icon_emoji": ":robot_face:",
        "text": "Hi, it's Kuribot! We are almost done",
        "blocks": [
    	{
    		"type": "section",
    		"text": {
    			"type": "mrkdwn",
    			"text": message
    	           }
        }
        ]
    }
    return payload, survey_url

def low_battery_template(user_id, battery_pct):
    payload = {
        "ts": "",
        "channel": user_id,
        "username": "kuribot",
        "icon_emoji": ":robot_face:",
        "text": "Hi, it's Kuribot! We are almost done",
        "blocks": [
    	{
    		"type": "section",
    		"text": {
    			"type": "mrkdwn",
    			"text": ":rotating_light: :warning: My battery is low (*" + str(battery_pct) + "%*)!! Please come and put me on my charger! :warning: :rotating_light: "
    	           }
        }
        ]
    }
    return payload
