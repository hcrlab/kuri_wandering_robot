def slack_template_1(user_id, direct_link, image_description=None):
    if image_description is None:
        image_description = "Kuri shared this picture with you!"
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
                "text": "Hi :wave:, it's Kuribot :robot_face:, with the \"Seeing Through the Eyes of a Robot\" project :eyes:. I took an image :camera_with_flash: I think you might like."
            }
        },
        {
            "type": "image",
            "image_url": direct_link,
            "alt_text": image_description
        },
        {
            "type": "section",
            "text": {
                "type": "mrkdwn",
                "text": "Could you please tell me whether you like it by clicking :white_check_mark: or :x: below?"
            }
        },
        {
            "type": "actions",
            "elements": [
                {
                    "type": "button",
                    "text": {
                        "type": "plain_text",
                        "text": ":white_check_mark:",
                        "emoji": True
                    },
                    "value": "check_mark",
                    "action_id": "action_id_check_mark"
                },
                {
                    "type": "button",
                    "text": {
                        "type": "plain_text",
                        "text": ":x:",
                        "emoji": True
                    },
                    "value": "x_mark",
                    "action_id": "action_id_x"
                }
            ]
        }
    ],
    }
    return payload
"""
Types of messages to expect:
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
def intro_template(day, user_id):
    #The intro message accepts an int that describes what day the participant is on
    #Expected values: 0-2
    #User_id is also needed to post to the correct channel
    if day == 0:
        img_link = "https://cdn.discordapp.com/attachments/827661547802198016/847616239572090900/mayfield-robotics-ceases-production-of-kuri-robot-amid-a-questionable-future.gif"
        message_text = "In the meantime, I'm very excited to work with you and I want to show you a dance I've been working on. I hope you like it."
        alt_text = "My big dance"
    else if day == 1:
        img_link = "https://www.cnet.com/a/img/JkD_3wBEktye-EV2e3G8OdbSGSI=/770x578/2017/01/03/026e660f-4525-4bc8-94f3-9a5e9c61a776/kuriproductphotos-4.jpg"
        message_text = "I'm looking foward to starting the second day, here's a little token of my appriciation"
        alt_text = "Thanks"
    else if day == 2:
        img_link = "https://thegadgetflow.com/wp-content/uploads/2017/01/Kuri-Intelligent-Home-Robot-004.jpg"
        message_text = "This is the last day, I hope you are ready to finish strong! Here's something that might inspire you."
        alt_text = "Party time!"
    else:
        img_link = ""
        message_text = "I think something went wrong, sorry!"
        alt_text = ""
    payload = {
        "ts": "",
        "channel": user_id,
        "username": "kuribot",
        "text": "Hi, it's Kuribot! I'm going to be starting soon. Get ready for some images.",
        "blocks": [
        {
            "type": "section",
            "text": {
                "type": "mrkdwn",
                "text": "Thanks for getting started, I'll send you a message soon after I get things set up."
            }
        },
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

def post_image(public_link,user_id):
    #This posts an image given:
    #an existing link to that image: public_link
    #the user ID: user_id
    #and which message: message_i. Expected values here are 0-4
    payload = {
        "ts": "",
        "channel": user_id,
        "username": "kuribot",
        "icon_emoji": ":robot_face:",
        "text": "Hi, it's Kuribot! I took an image I think you might like. Let me know what you think by clicking the buttons!",
        "blocks": [
    	{
    		"type": "image",
    		"image_url": public_link,
    		"alt_text": "Kuri shared this picture with you!",
    	}
	   ],
    }
    return payload

def post_message(user_id, message_i, condition):
    #This posts an image given:
    #the user ID: user_id
    #which message: message_i. Expected values here are 0-4
    #List of messages, going in order
    photo_message_list= [
    "Here is the first photo. Please respond with either a :white_check_mark: or a :x: if you like or dislike this photo.",
    "Here is the second photo. Please let me know if you like or dislike this photo with either a :white_check_mark: or a :x:.",
    "Here is the third photo. Please help me out further by replying with either a :white_check_mark: or a :x: if you like or dislike the photo.",
    "Here is the fourth photo. If you like this photo, let me know with a :white_check_mark:, if you dislike it, let me know with a :x:.",
    "Here is the fifth photo. Please select a :white_check_mark: if you like the photo, or select a :x: if you dislike it."
    ]

    #message content
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
    			"text": photo_message_list[message_i]
    		}
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
		}
	],
    }
    return payload

def action_button_check_mark_or_x(user_id, condition,response):
    ack()
    #user_id is the users_id
    #which user study condition: condition. Expected values are 0 (description), 1 (follow up 1), and 2 (follow up 2)
    #response is the type of response from the button they clicked. 1 is check mark, 0 is X
    if condition == 0:
        message_list = [
        "Thank you, I'll use your response to choose better photos. Next, please describe the photo.",
        "Thank you, I've recorded your response which I will learn from. Next, please help me out by describing the photo.",
        "Thank you, I've added your feedback to my database. Next, describe the photo to help me understand what is in it.",
        "Thank you, I'm starting to understand your preferences better. Describe the photo so I can keep learning."
        ]
    if condition == 1:
        message_list = [
        "Thank you, I'll use your response to choose better photos. Why did you rate the photo this way?",
        "Thank you, I've recorded your response which I will learn from. Why did you make that choice for the photo?",
        "Thank you, I've added your feedback to my database. I'd like to understand, why did you rate the photo this way?",
        "Thank you, I'm starting to understand your preferences better. I want to learn more about why you made that choice."
        ]
    if condition == 2 and response == 1:
        message_list = [
        "Thank you, I'll use your response to choose better photos. I'd like to understand more about why you liked this photo. Was there an object in the photo that you liked? What was it?",
        "Thank you, I've recorded your response, which I will learn from. I want to learn why you liked this photo, was there a particular object you liked?",
        "Thank you, I've added your feedback to my database. If you tell me more about objects you liked in the photo, I can learn more efficiently",
        "Thank you, I'm starting to understand your preferences better. Can you explain more about why you liked this photo? Any objects that you liked?"
        ]
    else if condition == 2 and response == 0:
        message_list = [
        "Thank you, I'll use your response to choose better photos. I'd like to understand more about why you disliked this photo. Was there an object in the photo that you disliked? What was it?",
        "Thank you, I've recorded your response, which I will learn from. I want to learn why you disliked this photo, was there a particular object you disliked?",
        "Thank you, I've added your feedback to my database. If you tell me more about objects you disliked in the photo, I can learn more efficiently",
        "Thank you, I'm starting to understand your preferences better. Can you explain more about why you disliked this photo? Any objects that you disliked?"
        ]
    if response == 1:
        emoji = ":white_check_mark:"
    else:
        emoji = ":x:"

    message = message_list[random.randrange(len(message_list))]
    payload={
        "ts": "",
        "channel": user_id,
        "username": "kuribot",
        "icon_emoji": ":robot_face:",
        "text": "Hi, it's Kuribot! I took an image I think you might like. Let me know what you think by clicking the buttons!",
        "replace_original":True, #not sure if this works here, may need to test
        "blocks":[
            {
        		"type": "section",
        		"text": {
        			"type": "mrkdwn",
        			"text": "You chose "+ emoji
        		}
        	},
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
            },
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
        ]
    }

    return payload

def confirm_input(user_input,user_id):
    #user_input is recovered at the pevious step, from something like
        #body["message"]["blocks"][0]
    #user_id is the user's id for correct placement
    payload = {
        "ts": "",
        "channel": user_id,
        "username": "kuribot",
        "icon_emoji": ":robot_face:",
        "text": "Hi, it's Kuribot! We are almost done",
        #the below might cause issues, double check here
        "replace_original":True,
        "blocks": [
        {
            "type": "section",
            "text": {
                "type": "mrkdwn",
                "text": "Thanks for your input!, you said: "+ user_input #How can I display the entered text?
                }
        },
        ]
    }

    return payload

def end_block_template(user_id):
    #Every two hours, when the photos are done, users receive one of five messages
    message_list = [
    "Thank you, we are done for this block. I'll send you more photos in about 2 hours",
    "We are done for now, thank you for the responses. You will get more photos from me in about 2 hours",
    "Great, that's it for now. I'll send you more photos in about 2 hours. Thank you.",
    "Ok, that's it for now. Thank you for the help, I'll send some more photos in about 2 hours.",
    "Thanks for your help. You've finished this block, and I'll have more photos for you about two hours."
    ]
    message = message_list[random.randrange(len(message_list))]
    payload = {
        "ts": "",
        "channel": user_id,
        "username": "kuribot",
        "icon_emoji": ":robot_face:",
        "text": "Hi, it's Kuribot! We are finished with this set",
        #the below might cause issues, double check here
        "replace_original":True,
        "blocks": [
        {
            "type": "section",
            "text": {
                "type": "mrkdwn",
                "text": message
                }
        },
        ]
    }
    return payload

def closing_template(user_id, random_id):
    #The study URL will need to be changed for the final version, but here's the temp one for testing
    #random_id will be generated beforehand and added to a dictionary to keep track of who got what random_id
    study_url = "https://ucsantacruz.co1.qualtrics.com/jfe/form/SV_9uV0VU7U2epEviu?random_id="
    #random_id = random.randomrange(100000,999999)

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
    			"text": "Thank you for responding to all the pictures, you've really helped me out. Before you finish up, please go to this URL:"+ study_url + str(random_id)+" to complete a survey and finish the study"
    	           }
        }
        ]
    }
    return payload
