from yolov8 import Yolov8
import cv2
import numpy as np
import argparse

def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description = 'YoloV8 Fire Detection')
    parser.add_argument(
        "--webcam-resolution",
        default=[1280,720],
        type =int,
        nargs= 2
    )
    parser.add_argument(
        '--model',   
        default= r".\model\fireV2.onnx", 
        type=str
    )
    parser.add_argument(
        "--source", 
        default="0", 
        type=str
    )
    args = parser.parse_args()
    return args

def main():

    # Get information
    args = parse_arguments()
    path = args.source
    model = args.model


    # Set up the Yolo model
    fire_model = Yolov8()
    fire_model.set_up_model(model)
    fire_model.model.warmup(imgsz=(1 , 3, *fire_model.imgsz))
    cap = cv2.VideoCapture(path)
    while True:
        ret, frame = cap.read()
        
        if not ret:
            cap = cv2.VideoCapture(args.source)
            continue
        results = fire_model.inference(frame)
        for result in results:
            x1, y1, x2, y2, conf, cls = result
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, str(fire_model.names[int(cls)]), (int(x1), int(y1)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        frame = cv2.resize(frame, (640, 640))  
        cv2.imshow("frame", frame)

        # press Esc to close the window
        if (cv2.waitKey(30) == 27):
            break
if __name__ == '__main__':
    main()