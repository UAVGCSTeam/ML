from mlsetup import MLSetup # import the yolov8.py file
import cv2
import numpy as np
import argparse

def source_type(value):
    try:
        if value.isdigit():
            return int(value)
    except ValueError:
        return value

def parse_arguments() -> argparse.Namespace:
    """
    === Parse Arguments ============================================================
        Parse the command line arguments for webcam resolution, model selection, and 
        footage selection.
    ================================================================================
    """
    parser = argparse.ArgumentParser(description = 'YoloV8 Fire Detection')
    
    # camera resolution
    # parser.add_argument(
    #     "--webcam-resolution",
    #     default=[1280,720], # originally 1280,720
    #     type=int,
    #     nargs= 2
    # )

    # select compute device 
    parser.add_argument(
        "--device",
            default="cpu",
            # default="gpu",
            type=str,
    )

    parser.add_argument(
        "--webcam-resolution",
            default=[1280,720], # originally 1280,720
            type=int,
            nargs=2
    )

    # model selection
    parser.add_argument(
        # Select the trained weights to use for inference.
        '--weights',   
        # default= r"weights/fire_detection.onnx", 
        # default= r"weights/fire_detection.pt", # shittier
        default= r"weights/fireV2.onnx", 
        # default= r"weights/fireV2.pt", 
        type=str
    )
    # footage source
    parser.add_argument(
        "--source", 
        # type=str
        # default="test_folder/test_img.jpg",
        # default="test_folder/thermal_test.jpg",
        # default="test_folder/test_video.mp4",
        # default="test_folder/test_video2.mp4", # Super far away - most likely not pracitcal
        # default="test_folder/test_video3.mp4",
        # default="test_folder/test_video4.mp4",
        # default="test_folder/test_video5.mp4", # tractor in the video
        # default="test_folder/test_video6.mp4",
        default="test_folder/test_video7.mp4",
        # default="test_folder/test_video8.mp4", # clear fire line
        # default="test_folder/test_video9.mp4",
        # default="test_folder/test_video10.mp4", # tree on fire
        # default=0, # accesses the webcam of your computer 
        help="Path to the input video file",
        type=str
    )
    args = parser.parse_args()
    return args

def main():
    # Get information
    args = parse_arguments()
    device = args.device
    weights = args.weights
    path = source_type(args.source)

    # Set up the Yolo model
    fire_model = MLSetup()
    fire_model.set_up_model(weights, device)
    fire_model.model.warmup(imgsz=(1 , 3, *fire_model.imgsz))
    cap = cv2.VideoCapture(path)

    while True:
        ret, frame = cap.read()
        
        if not ret:
            cap = cv2.VideoCapture(args.source)
            continue
        results = fire_model.inference(frame) # where the actual inference happens
        for result in results:
            x1, y1, x2, y2, conf, cls = result
            print("BB Coords: " + str(int(x1)) + " " + str(int(x2)) + " " + str(int(y1)) + " " + str(int(y2)))
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2) # overlays bounding boxes onto the image
            cv2.putText(frame, str(fire_model.names[int(cls)]), (int(x1), int(y1)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2) # overlay titles onto the image
        # frame = cv2.resize(frame, (640, 640)) # original at 640 x 640  
        cv2.imshow("frame", frame)

        # press `esc` to close the window
        if (cv2.waitKey(30) == 27):
            break

if __name__ == '__main__':
    main()
