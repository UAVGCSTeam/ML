import os
import shutil
import random
import sys


def main():
    type_folder = str(input("Type of folder:"))
    file_txt = str(input("Name of file:" ))
    
    destination_folder = r'.\splitted_data_folder'

    if not os.path.exists(destination_folder):
        os.makedirs(destination_folder)

    # create a specific folder
    type_folder = os.path.join(destination_folder, type_folder)
    os.makedirs(type_folder, exist_ok=True)

    ## Create the images folder and label folders
    images_folder = os.path.join(type_folder, 'images')
    labels_folder = os.path.join(type_folder, 'labels')
    os.makedirs(images_folder, exist_ok=True)
    os.makedirs(labels_folder, exist_ok=True)
    # Path to your dataset folder
    
    dataset_folder = r'C:\Users\hoang\OneDrive - Cal Poly Pomona\Computer_Vision\firedetection\dataset'
    label_extension = ".txt"  # Assuming labels are in .txt format, adjust if different

    # Read the txt file and copy images and labels
    with open(file_txt, 'r') as file:
        for line in file:
            image_name = line.strip()  # Remove newline characters or any leading/trailing spaces
            image_path = os.path.join(dataset_folder, image_name)

            # Check if the image exists in the dataset folder
            if os.path.exists(image_path):
                # Copy image to the new images_folder
                shutil.copy(image_path, images_folder)
                
                # Construct the label filename, assuming the image_name contains the extension, like ".jpg"
                label_name = os.path.splitext(image_name)[0] + label_extension
                label_path = os.path.join(dataset_folder, label_name)

                # Check if the label exists in the dataset folder
                if os.path.exists(label_path):
                    # Copy label to the new labels_folder
                    shutil.copy(label_path, labels_folder)
                else:
                    print(f"Label for {image_name} not found!")
            else:
                print(f"Image {image_name} not found!")

if __name__ == '__main__':
    main()
    sys.exit()