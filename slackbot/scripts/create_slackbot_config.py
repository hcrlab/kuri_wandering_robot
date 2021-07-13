import time, yaml

class User(object):
    def __init__(self, user_id, order, learning_condition, first_message_ts, n_days=3, message_interval=None):
        """
        Stores the data for a user, and initiatlizes their timing schedule.
        All timing schedules are n_days "days" long. They consist of a pre-study
        message, and then the first (n_days-1) "days" there is a daily intro message,
        at 4 times during the "day" there are images sent, and at the end of the
        "day" there is a survey sent. On the third "day," there is a daily intro,
        at two times there are images sent, then there is a survey, then at
        another two times there are images sent, and then another survey.

        If message_interval is None, the schedule corresponds with real days.
        The pre-study message is sent at 4:45 PM the day before, and on every day
        the intro is sent at 9AM, images are sent at 10AM, 12PM, 2PM, and 4PM,
        and then the end-of-day survey is sent at 4:45PM. Additionally, on the
        third day, the additional survey is sent at 1PM.
          - If message_interval is None, first_message_ts must be a string of the
            form "yyyy-mm-dd", refering to the date that the *pre-study message*
            should be sent.

        If message_interval is not None, then the number and sequence of messages
        remains the same as above, but each message is sent message_interval seconds
        apart.
          - message_interval is not None, first_message_ts must be a epoch timestamp

        If first_message_ts is None, do not have any scheduled times for this user
        """
        self.user_id = user_id
        self.order = order
        self.learning_condition = learning_condition
        self.first_message_ts = first_message_ts
        self.n_days = n_days
        self.message_interval = message_interval

        if self.first_message_ts is not None:
            self.initialize_times()

    def initialize_times(self):
        """
        See the docstring for __init__ for more details
        """
        self.daily_schedule = []
        if self.message_interval is None:
            pattern = '%Y-%m-%d %H:%M:%S'
            self.pre_study_message_timestamp = time.mktime(time.strptime(self.first_message_ts+" 16:45:00", pattern))
            for day in range(1,n_days+1):
                epoch_offset = day*24*60*60.0
                intro_message_timestamp = time.mktime(time.strptime(self.first_message_ts+" 09:00:00", pattern)) + epoch_offset
                images_sent_timestamps = [
                    time.mktime(time.strptime(self.first_message_ts+" 10:00:00", pattern)) + epoch_offset,
                    time.mktime(time.strptime(self.first_message_ts+" 12:00:00", pattern)) + epoch_offset,
                    time.mktime(time.strptime(self.first_message_ts+" 14:00:00", pattern)) + epoch_offset,
                    time.mktime(time.strptime(self.first_message_ts+" 16:00:00", pattern)) + epoch_offset,
                ]
                survey_timestamps = [
                    time.mktime(time.strptime(self.first_message_ts+" 16:45:00", pattern)) + epoch_offset,
                ]
                if day == n_days:
                    survey_timestamps.insert(0, time.mktime(time.strptime(self.first_message_ts+" 13:00:00", pattern)) + epoch_offset)
                self.daily_schedule.append({
                    "intro_message_timestamp" : intro_message_timestamp,
                    "images_sent_timestamps" : images_sent_timestamps,
                    "survey_timestamps" : survey_timestamps,
                })
        else:
            self.pre_study_message_timestamp = self.first_message_ts
            daily_start = self.first_message_ts + self.message_interval
            for day in range(1,n_days+1):
                intro_message_timestamp = daily_start
                images_sent_timestamps = [
                    daily_start + self.message_interval*1,
                    daily_start + self.message_interval*2,
                ]
                if day == n_days:
                    survey_timestamps = [
                        daily_start + self.message_interval*3,
                        daily_start + self.message_interval*6,
                    ]
                    images_sent_timestamps.append(daily_start + self.message_interval*4)
                    images_sent_timestamps.append(daily_start + self.message_interval*5)
                    daily_start += self.message_interval*7
                else:
                    survey_timestamps = [
                        daily_start + self.message_interval*5,
                    ]
                    images_sent_timestamps.append(daily_start + self.message_interval*3)
                    images_sent_timestamps.append(daily_start + self.message_interval*4)
                    daily_start += self.message_interval*6

                self.daily_schedule.append({
                    "intro_message_timestamp" : intro_message_timestamp,
                    "images_sent_timestamps" : images_sent_timestamps,
                    "survey_timestamps" : survey_timestamps,
                })

    def to_dict(self):
        """
        Convert the User to a dict to be written to yaml
        """
        if self.first_message_ts is not None:
            return {
                self.user_id : {
                    "order" : self.order,
                    "learning_condition" : self.learning_condition,
                    "pre_study_message_timestamp" : self.pre_study_message_timestamp,
                    "daily_schedule" : self.daily_schedule,
                }
            }
        else:
            return {
                self.user_id : {
                    "order" : self.order,
                    "learning_condition" : self.learning_condition,
                }
            }

if __name__ == "__main__":
    ############################################################################
    # Parameters To Set
    ############################################################################
    slack_bot_token = "TEST_SLACK_BOT_TOKEN"
    slack_user_token = "TEST_SLACK_USER_TOKEN"
    slack_signing_secret = "TEST_SLACK_SIGNING_SECRET"

    low_battery_savior_user_id = "TEST_USER_0"

    n_days = 1
    users = [
        User("TEST_USER_1", 0, 1, "2021-07-01", n_days=n_days),
        User("TEST_USER_2", 1, 1, 1625520360, n_days=n_days, message_interval=30),
        User("TEST_USER_3", 1, 1, None),
    ]

    ############################################################################
    # Save the file
    ############################################################################
    filepath = "../cfg/slackbot_autogen.yaml"
    with open(filepath, 'w') as f:
        yaml_dict = {
            "slack_bot_token" : slack_bot_token,
            "slack_user_token" : slack_user_token,
            "slack_signing_secret" : slack_signing_secret,
            "low_battery_savior_user_id" : low_battery_savior_user_id,
            "users_list" : [user.user_id for user in users]
        }
        for user in users:
            yaml_dict.update(user.to_dict())
        yaml.dump(yaml_dict, f)
