from ultralytics import YOLO
import sys
import os
import shutil
import torch
import pickle
import onnx 


def main():
    name_model = input('Model name: ')
    format = input('Format: ')

    model = YOLO(name_model)

    #model = torch.load(name_model)  # load a pretrained model (recommended for training)
    # Set the input shape
    #batch_size = 1
    #input_shape = (3, 224, 224)

    # Define the input tensor
    #dummy_input = torch.randn(batch_size, *input_shape)

    #new_model = os.path.splitext(name_model)[0] + '.' + format
    
    # Export the model to ONNX format
    if format == 'onnx':
        success = model.export(format=format)
    elif format == 'engine':
        success = model.export(format=format, device = 0, half = True)


    destination = r'..\model'
    shutil.move(success, destination)
    shutil.move(name_model, destination)

if __name__ == "__main__":
    main()
    sys.exit()