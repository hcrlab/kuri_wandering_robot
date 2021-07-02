import os
import traceback

if __name__ == "__main__":
    platform = input("Are you running on the (r)obot or the (s)lackbot? ")

    if len(platform) > 0 and platform[0] == "r":
        on_robot = True
    elif len(platform) > 0 and platform[0] == "s":
        on_robot = False
    else:
        print("Unknown platform. Please respond with either r or s. Exiting.")
        exit()

    if on_robot:
        filepaths_to_delete = [
            "./kuri_cmm_demo/cfg/sent_messages_database.pkl",
            "/mayfield/data/kuri_cmm_demo/images/*",
        ]
    else:
        csv_suffix = input("Is there any suffix you'd like to add to the csv files and sent_messages_database.pkl? (default is no suffix, press enter) ")
        filepaths_to_delete = [
            "./slackbot/cfg/sent_messages_database%s.pkl" % csv_suffix,
            "./slackbot/logs/log.txt",
            "./data/received_images%s.csv" % csv_suffix,
            "./data/sent_images%s.csv" % csv_suffix,
            "./data/reactions%s.csv" % csv_suffix,
            "./data/followups%s.csv" % csv_suffix,
            "./data/survey%s.csv" % csv_suffix,
            "./data/images/*",
        ]

    commands = []
    for filepath in filepaths_to_delete:
        commands.append("rm %s" % filepath)
    final_check_msg = "This will run the following commands:\n\n"
    for command in commands:
        final_check_msg += command+"\n"
    final_check_msg += "\nAre you sure you want to run this? (Y)es / (n)o "

    final_check = input(final_check_msg)

    if len(final_check) == 0 or final_check[0] != "Y":
        print("Did not respond with 'Y'. Exiting.")
        exit()

    for command in commands:
        try:
            os.system(command)
            print("Succesfully executed: %s" % command)
        except Exception as e:
            print("Failed to execute: %s" % command)
            print(traceback.format_exc())
            print("Error %s." % e)
