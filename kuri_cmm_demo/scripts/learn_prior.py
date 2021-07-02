import boto3
import csv
import cv2
import json
import math
import numpy as np
import os
import time
import traceback
from scipy.optimize import minimize, approx_fprime
from scipy.special import expit
from scipy.stats import multivariate_normal
import scipy.stats as stats
from sklearn.linear_model import LogisticRegression
import sklearn.metrics

def load_data(base_dir, label_filenames, client):
    """
    base_dir should point to a folder with numbered jpg files (e.g., 0.jpg, 5.jpg, etc.)
    Those numbers should be in a CSV in that folder called img_nums.csv. All the
    filenames in label_filenames should all be in the same directory, and this
    script will write a prior mean, prior covariance, and list of classes.
    """
    # Get the Image Features
    img_nums_filename = "img_nums.csv"
    img_filename = "%s.jpg"
    classes_filename = "objects.json"

    # Load the images and vectorize them
    X = []
    max_vec_length = 0
    img_nums = []
    classes = []
    with open(base_dir+img_nums_filename, mode='r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        for row in csv_reader:
            img_num = row["img_num"]
            print("img_num", img_num)
            img_nums.append(int(img_num))
            # Load the image
            img_cv2 = cv2.imread(cv2.samples.findFile(os.path.join(base_dir, img_filename % img_num)), cv2.IMREAD_COLOR)
            img_bytes = cv2.imencode('.jpg', img_cv2)[1].tobytes()
            response = client.detect_labels(Image={'Bytes':img_bytes}, MaxLabels=100)

            # Vectorize it
            img_vec = [0.0 for i in range(len(classes))]
            for label in response["Labels"]:
                object_name = label["Name"]
                confidence = label["Confidence"]/100.0
                try:
                    object_i = classes.index(object_name)
                except ValueError: # object_name isn't in list -- extend the dimentionality
                    object_i = len(classes)
                    classes.append(object_name)
                    img_vec.append(0.0)
                img_vec[object_i] = confidence
            img_vec = np.array(img_vec)

            if img_vec.shape[0] > max_vec_length:
                max_vec_length = img_vec.shape[0]
            X.append(img_vec)
    # Pad the different-length vectors
    for i in range(len(X)):
        x = X[i]
        X[i] = np.pad(x, (0, max_vec_length-x.shape[0]))
    X = np.array(X)

    # Load the labels
    label_filename_to_y = {}
    for label_filename in label_filenames:
        img_num_to_labels = {}
        with open(os.path.join(base_dir, label_filename), mode='r') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            for row in csv_reader:
                img_num = row["img_num"]
                label = int(row["label"])
                if label == -1: label = 0
                img_num_to_labels[int(img_num)] = label
        y = []
        for img_num in img_nums:
            y.append(img_num_to_labels[img_num])
        y = np.array(y)
        label_filename_to_y[label_filename] = y

    print("X", X)
    print("label_filename_to_y", label_filename_to_y, {label_filename : np.mean(label_filename_to_y[label_filename]) for label_filename in label_filename_to_y})

    with open(os.path.join(base_dir, classes_filename), mode='w') as f:
        json.dump(classes, f, indent=4)

    return X, label_filename_to_y

if __name__ == "__main__":
    """
    To learn a prior, you should have a folder of images, and CSV files that
    label the images with 0/1 preferences.

    This script will load every jpg image in the folder, get the objects using
    AWS Rekognition, vectorize the images, learn a prior, and save both the
    ordered classes and the priors.
    """

    base_dir = "/Users/amaln/Documents/HCRLab/kuri_cmm_demo/kuri_cmm_demo/training_data/"
    label_filenames = ["labels_lee.csv", "labels_chris.csv", "labels_nick.csv", "labels_amal.csv"]

    # Configure the AWS Rekognition client
    aws_profile_name = "default"
    aws_region_name = "us-west-2"
    session = boto3.session.Session(
        profile_name=aws_profile_name, region_name=aws_region_name)
    client = session.client('rekognition')

    X, label_filename_to_y = load_data(base_dir, label_filenames, client)
    train_set_X = np.zeros((0, X.shape[1]))
    train_set_y = np.zeros((0), dtype=np.int32)
    for label_filename in label_filename_to_y:
        train_set_X = np.append(train_set_X, X, axis=0)
        train_set_y = np.append(train_set_y, label_filename_to_y[label_filename], axis=0)
    n_dims = train_set_X.shape[1]

    prior_filename = "prior.npz"
    covariance_level = 0.1

    model = LogisticRegression(max_iter=1000, C=5, class_weight='balanced')
    model.fit(train_set_X, train_set_y)
    prior_mean = np.insert(model.coef_[0, :], 0, model.intercept_, axis=0)
    prior_covariance = np.zeros((n_dims+1,n_dims+1))
    np.fill_diagonal(prior_covariance, [covariance_level for _ in range(n_dims)])#2.0*np.random.rand(nDims+1))

    print("prior_mean", prior_mean, prior_mean.shape)
    print("prior_covariance", prior_covariance, prior_covariance.shape)

    np.savez(os.path.join(base_dir, prior_filename), prior_mean=prior_mean, prior_covariance=prior_covariance)
