import os
import shutil
import random

# Set the path to the dataset folder
dataset_folder = '.\dataset'

# Set the path to the destination folder where the train, valid, and test sets will be created
destination_folder = r'.\splitted_datasets'

# Create the train, valid, and test folders
train_folder = os.path.join(destination_folder, 'train')
valid_folder = os.path.join(destination_folder, 'valid')
test_folder = os.path.join(destination_folder, 'test')
os.makedirs(train_folder, exist_ok=True)
os.makedirs(valid_folder, exist_ok=True)
os.makedirs(test_folder, exist_ok=True)

# Create the images and labels folders inside train, valid, and test folders
train_images_folder = os.path.join(train_folder, 'images')
train_labels_folder = os.path.join(train_folder, 'labels')
valid_images_folder = os.path.join(valid_folder, 'images')
valid_labels_folder = os.path.join(valid_folder, 'labels')
test_images_folder = os.path.join(test_folder, 'images')
test_labels_folder = os.path.join(test_folder, 'labels')
os.makedirs(train_images_folder, exist_ok=True)
os.makedirs(train_labels_folder, exist_ok=True)
os.makedirs(valid_images_folder, exist_ok=True)
os.makedirs(valid_labels_folder, exist_ok=True)
os.makedirs(test_images_folder, exist_ok=True)
os.makedirs(test_labels_folder, exist_ok=True)

# Get the list of image files in the dataset folder
image_files = [f for f in os.listdir(dataset_folder) if f.endswith('.jpg')]

# Shuffle the image files randomly
random.shuffle(image_files)

# Split the image files into train, valid, and test sets
train_files = image_files[:int(0.7 * len(image_files))]
valid_files = image_files[int(0.7 * len(image_files)):int(0.85 * len(image_files))]
test_files = image_files[int(0.85 * len(image_files)):]

# Move the image files and their corresponding label files to the respective train, valid, and test folders
for file in train_files:
    image_path = os.path.join(dataset_folder, file)
    label_path = os.path.join(dataset_folder, file.replace('.jpg', '.txt'))
    shutil.copy(image_path, train_images_folder)
    shutil.copy(label_path, train_labels_folder)

for file in valid_files:
    image_path = os.path.join(dataset_folder, file)
    label_path = os.path.join(dataset_folder, file.replace('.jpg', '.txt'))
    shutil.copy(image_path, valid_images_folder)
    shutil.copy(label_path, valid_labels_folder)

for file in test_files:
    image_path = os.path.join(dataset_folder, file)
    label_path = os.path.join(dataset_folder, file.replace('.jpg', '.txt'))
    shutil.copy(image_path, test_images_folder)
    shutil.copy(label_path, test_labels_folder)
