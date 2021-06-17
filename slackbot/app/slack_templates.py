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
