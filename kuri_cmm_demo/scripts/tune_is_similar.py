#!/usr/bin/env python
import os

def entropy(y):
    n_pos = np.count_nonzero(y)
    n_neg = np.shape(y)[0] - n_pos
    p_pos = n_pos/(n_pos+n_neg)
    p_neg = n_neg/(n_pos+n_neg)
    if p_pos == 0.0 or p_neg == 0.0:
        return 0.0
    return -p_pos*math.log(p_pos, 2) - p_neg*math.log(p_neg, 2)


if __name__ == "__main__":
    ############################################################################
    # Set the configuration parameters
    ############################################################################

    base_dir = '/home/amalnanavati/workspaces/amal_ws/src/kuri_cmm_demo/datasets/' # '/workspace/src/kuri_cmm_demo/datasets/' #
    dataset_names = ['20210617_Facing_Living_Room_B', '20210617_Facing_Living_Room_A', '20210617_Facing_Laundry_Room_Outdoors', '20210617_Facing_Kitchen', '20210617_Facing_Dining_Room', '20210617_Amal_Room']
    src_folder = "compressed/"
    dest_folder = "subsampled_images/"

    subsample_images = False # True #
    target_n_images = 50

    vectorize_images = False #True #

    get_similarity_scores = False # True #

    graph_similarity = False # True # 
    only_stationary_images = False # True #

    ############################################################################
    # Load the CMMDemo object (with fake parameters where necessary)
    ############################################################################
    if vectorize_images or get_similarity_scores:
        from kuri_cmm_demo_node import CMMDemo
        import rospy

        rospy.init_node("tune_is_similar")

        img_topic = "fake_image_topic"
        object_detection_srv = rospy.get_param('~object_detection_srv', 'object_detection')
        print("object_detection_srv", object_detection_srv)
        slackbot_url = "fake_slackbot_url"
        send_messages_database_filepath = "fake_send_messages_database_filepath" # This is okay because none of the methods I call actually update the database
        head_state_topic = "fake_head_state_topic"

        cmm_demo = CMMDemo(img_topic, head_state_topic, object_detection_srv, slackbot_url, send_messages_database_filepath, visualize_view_tuner=True)

    ############################################################################
    # Subsample the images
    ############################################################################
    if subsample_images:
        import math
        import shutil

        for i in range(len(dataset_names)):
            src_folder_path = os.path.join(base_dir, dataset_names[i], src_folder)
            dest_folder_path = os.path.join(base_dir, dataset_names[i], dest_folder)
            if not os.path.isdir(dest_folder_path):
                os.mkdir(dest_folder_path)
            n_files = len(os.listdir(src_folder_path))
            image_interval = int(math.floor(n_files/target_n_images))
            for j in range(target_n_images):
                img_num = image_interval*j+1
                src_img_filepath = os.path.join(src_folder_path, str(img_num)+".jpg")
                dest_img_filepath = os.path.join(dest_folder_path, str(j)+".jpg")
                # print(src_img_filepath, dest_img_filepath)
                shutil.copy(src_img_filepath, dest_img_filepath)

    ############################################################################
    # Vectorize the images
    ############################################################################
    if vectorize_images:
        import cv2
        import json
        import numpy as np
        from sensor_msgs.msg import CompressedImage

        for i in range(len(dataset_names)):
            dest_folder_path = os.path.join(base_dir, dataset_names[i], dest_folder)
            for filename in os.listdir(dest_folder_path):
                if not filename.endswith(".jpg"): continue
                filepath = os.path.join(dest_folder_path, filename)
                img_cv2 = cv2.imread(filepath, cv2.IMREAD_COLOR)

                img_msg = CompressedImage()
                img_msg.header.stamp = rospy.Time.now()
                img_msg.format = "jpeg"
                img_msg.data = np.array(cv2.imencode('.jpg', img_cv2)[1]).tostring()

                img_vector = cmm_demo.img_msg_to_img_vector(img_msg)
                img_vector_filepath = filepath[:-4]+"_vector.json"
                print(img_vector, img_vector_filepath)

                with open(img_vector_filepath, 'w') as f:
                    json.dump(img_vector.tolist(), f)

    ############################################################################
    # For every pair of images, get its is_similarity metric
    ############################################################################
    if get_similarity_scores:
        import csv
        import cv2
        import json
        from sensor_msgs.msg import CompressedImage

        csv_header = ["Dataset 1 Name", "Image 1 Num", "Dataset 2 Name", "Image 2 Num", "Object Similarity", "Image Similarity"]
        csv_rows = []

        for dataset_i in range(len(dataset_names)):
            dataset_i_name = dataset_names[dataset_i]
            print("Dataset I %s" % dataset_i_name)
            for image_i in range(target_n_images):
                image_i_filepath = os.path.join(base_dir, dataset_i_name, dest_folder, str(image_i)+".jpg")
                print("image_i_filepath %s" % image_i_filepath)

                # Load image_i_msg and image_i_vector
                image_i_cv2 = cv2.imread(image_i_filepath, cv2.IMREAD_COLOR)
                image_i_msg = CompressedImage()
                image_i_msg.header.stamp = rospy.Time.now()
                image_i_msg.format = "jpeg"
                image_i_msg.data = np.array(cv2.imencode('.jpg', image_i_cv2)[1]).tostring()
                image_i_vector_filepath = os.path.join(base_dir, dataset_i_name, dest_folder, str(image_i)+"_vector.json")
                with open(image_i_vector_filepath, 'r') as f:
                    image_i_vector = np.array(json.load(f))

                for dataset_j in range(len(dataset_names)):
                    dataset_j_name = dataset_names[dataset_j]
                    print("Dataset J %s" % dataset_j_name)
                    for image_j in range(target_n_images):
                        image_j_filepath = os.path.join(base_dir, dataset_j_name, dest_folder, str(image_j)+".jpg")
                        print("image_j_filepath %s" % image_j_filepath)

                        # Load image_i_msg and image_i_vector
                        image_j_cv2 = cv2.imread(image_j_filepath, cv2.IMREAD_COLOR)
                        image_j_msg = CompressedImage()
                        image_j_msg.header.stamp = rospy.Time.now()
                        image_j_msg.format = "jpeg"
                        image_j_msg.data = np.array(cv2.imencode('.jpg', image_j_cv2)[1]).tostring()
                        image_j_vector_filepath = os.path.join(base_dir, dataset_j_name, dest_folder, str(image_j)+"_vector.json")
                        with open(image_j_vector_filepath, 'r') as f:
                            image_j_vector = np.array(json.load(f))

                        # Get the similarity
                        if image_i_vector.shape[0] > image_j_vector.shape[0]:
                            most_recent_stored_img_msgs = [image_j_msg]
                            most_recent_stored_img_vectors = [image_j_vector]
                            img_msg = image_i_msg
                            img_vector = image_i_vector
                        else:
                            most_recent_stored_img_msgs = [image_i_msg]
                            most_recent_stored_img_vectors = [image_i_vector]
                            img_msg = image_j_msg
                            img_vector = image_j_vector
                        _, object_similarities, image_similarities = cmm_demo.is_similar(most_recent_stored_img_msgs, most_recent_stored_img_vectors,
                            img_msg, img_vector, return_similarity=True)

                        # Store it to save in a csv
                        csv_rows.append([dataset_i_name, image_i, dataset_j_name, image_j, object_similarities[0], image_similarities[0]])

        csv_filepath = os.path.join(base_dir, "distances.csv")
        with open(csv_filepath, "w") as f:
            csv_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

            csv_writer.writerow(csv_header)
            for row in csv_rows: csv_writer.writerow(row)

    ############################################################################
    #  Graph Similarity
    ############################################################################
    if graph_similarity:
        import csv
        import math
        import matplotlib.pyplot as plt
        import numpy as np
        import pandas as pd
        import seaborn as sns
        # from sklearn import svm
        from sklearn.metrics import f1_score
        from sklearn.metrics import accuracy_score
        from sklearn.metrics import precision_score
        from sklearn.metrics import recall_score
        from sklearn.metrics import confusion_matrix

        if only_stationary_images:
            img_num_restriction = list(range(15))
        else:
            img_num_restriction = list(range(target_n_images))
        csv_filepath = os.path.join(base_dir, "distances.csv")
        dataset = []
        X, y = [], []
        with open(csv_filepath, "r") as f:
            csv_reader = csv.reader(f)
            headers = next(csv_reader, None)
            headers_to_index = {headers[i] : i for i in range(len(headers))}
            for row in csv_reader:
                image_1_num = int(row[headers_to_index["Image 1 Num"]])
                image_2_num = int(row[headers_to_index["Image 2 Num"]])
                # print(img_num_restriction, image_1_num, image_2_num)
                if image_1_num in img_num_restriction and image_2_num in img_num_restriction:
                    datasets_are_same = (row[headers_to_index["Dataset 1 Name"]] == row[headers_to_index["Dataset 2 Name"]])
                    object_similarity = float(row[headers_to_index["Object Similarity"]])
                    image_similarity = float(row[headers_to_index["Image Similarity"]])
                    dataset.append([datasets_are_same, object_similarity, image_similarity])
                    X.append([object_similarity, image_similarity])
                    y.append(1 if datasets_are_same else 0)

        dataset = pd.DataFrame(dataset, columns=["Datasets Are Same", "Object Similarity", "Image Similarity"])
        X = np.array(X)
        y = np.array(y)
        n = np.shape(y)[0]

        # print("Loading SVM")
        # clf = svm.SVC()
        # print("fitting SVM")
        # clf.fit(X[:,0:1], y)
        # print("Printing SVM results")
        # print(clf.coef_, clf.intercept_)

        linear_separator_granularity = 0.05
        best_separators = []
        for dim in range(2):
            min_val = math.ceil(np.amin(X[:,dim])/linear_separator_granularity)*linear_separator_granularity
            max_val = math.floor(np.amax(X[:,dim])/linear_separator_granularity)*linear_separator_granularity

            parent_entropy = entropy(y)

            separator = min_val
            max_information_gain, best_separator = None, None
            while separator < max_val:
                y_below = y[np.where(X[:,dim] <= separator)]
                y_above = y[np.where(X[:,dim] > separator)]
                if np.shape(y_below)[0] == 0 or np.shape(y_above)[0] == 0:
                    separator += linear_separator_granularity
                    continue

                child_below_entropy = entropy(y_below)
                child_above_entropy = entropy(y_above)

                p_below = np.shape(y_below)[0]/n
                p_above = np.shape(y_above)[0]/n

                information_gain = parent_entropy - p_below*child_below_entropy - p_above*child_above_entropy
                if max_information_gain is None or information_gain > max_information_gain:
                    max_information_gain = information_gain
                    best_separator = separator

                separator += linear_separator_granularity
            best_separators.append(best_separator)

        print("best_separators", best_separators)
        dim_0_results = X[:,0] <= best_separators[0]
        dim_1_results = X[:,1] >= best_separators[1]
        or_results = np.logical_or(dim_0_results, dim_1_results)
        and_results = np.logical_and(dim_0_results, dim_1_results)
        for metric in [accuracy_score, f1_score, precision_score, recall_score, confusion_matrix]:
            print("metric", metric)
            print("  dim_0_results", metric(dim_0_results, y))
            print("  dim_1_results", metric(dim_1_results, y))
            print("  or_results", metric(or_results, y))
            print("  and_results", metric(and_results, y))

        fig = plt.figure(figsize=(12, 8))
        ax = fig.subplots(1, 1)
        sns.scatterplot(x="Object Similarity", y="Image Similarity", hue="Datasets Are Same", data=dataset, ax=ax)
        ax.axvline(best_separators[0])
        ax.axhline(best_separators[1])
        if only_stationary_images:
            filename = "distances_stationary_images.png"
        else:
            filename = "distances_all_images.png"
        plt.savefig(os.path.join(base_dir, filename))
