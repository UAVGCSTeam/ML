# Edited code from Gannon on YouTube
# https://www.youtube.com/watch?v=7SPNY3D0c5Y
from ultralytics import YOLO
import time
import cv2 
from yolov8 import Yolov8 # import the yolov8.py header file
import torch 


def main():
    # import pytorch model
    myModel = Yolov8()
    myModel.set_up_model('model/fireV2.pt', 'gpu')
    myModel.model.warmup(imgsz=(1 , 3, *myModel.imgsz))
    cap = cv2.VideoCapture('test_folder/test_image.jpg')

    ret, frame = cap.read()

    results = myModel.inference(frame) # where the actual inference happens
    for result in results:
        x1, y1, x2, y2, conf, cls = result
        print("BB Coords: " + str(int(x1)) + " " + str(int(x2)) + " " + str(int(y1)) + " " + str(int(y2)))
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2) # overlays bounding boxes onto the image
        cv2.putText(frame, str(myModel.names[int(cls)]), (int(x1), int(y1)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2) # overlay titles onto the image
    # frame = cv2.resize(frame, (640, 640)) # original at 640 x 640  
    cv2.imshow("frame", frame)

    ONNX_FILE_PATH = 'resnet50.onnx'
    torch.onnx.export(myModel, input, ONNX_FILE_PATH, input_names=['input'],
                    output_names=['output'], export_params=True)

if __name__ == '__main__':
    main()







# # convert to .engine (TensorRT) model
# myMode.export(format='engine', dynamic=True, verbose=False, half=True, data='coco8.yaml')

# # import and test model
# tensorrt_model = YOLO('fireV2.engine', task='detect', verbose=False)
# results = tensorrt_model('test_image.jpg')

# # time to view results
# time.sleep(10)