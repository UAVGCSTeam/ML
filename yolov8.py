# DEFINITION of the Yolov8 Class: This file encapsulates the functionality related to the
# YOLOv8 model. It defines a class that handles model setup, preprocessing, inference, 
# and postprocessing of images or video frames for object detection.


# === Model Configuration and Initialization:
    # Sets default parameters like confidence threshold, IoU threshold, image sizes, 
    # device selection (CPU or GPU), and model weights.
    # Uses the AutoBackend from Ultralytics to load and configure the YOLOv8 model.

# === Preprocessing and Postprocessing:
    # Preprocessing: Resizes images using LetterBox, normalizes pixel values, and prepares 
    # the tensor for model input.
    # Postprocessing: Applies Non-Max Suppression (NMS) to filter detections and scales 
    # bounding boxes back to the original image size.

# === Inference Method:
    # Runs the model on preprocessed images and processes the raw predictions to return 
    # usable detection results, including bounding boxes, confidence scores, and class labels.



from ultralytics.utils.torch_utils import select_device, smart_inference_mode
from ultralytics.utils.checks import check_imgsz
from ultralytics.nn.autobackend import AutoBackend
from ultralytics.utils import DEFAULT_CFG, ROOT, ops
import torch
import cv2
import numpy as np
from ultralytics.data.augment import LetterBox
import time


class Yolov8:
    def __init__(self):
        self.conf = 0.20 # confidence threshold (originally 0.25)
        self.iou = 0.1 # NMS IoU threshold (originally 0.45) -- Think of this as the overlap threshold
        self.data = "coco.yaml"
        self.imgsz = (640, 640)  # (320, 192) or (416, 256) or (608, 352) for (height, width)
        self.max_det = 1000  # maximum number of detections per image
        self.dataset = ""
        self.names = ""
        self.half = False
        self.dnn = False
        self.agnostic_nms = False
        self.model = None
        self.classes = None

    def set_up_model(self, weights, newDevice):
        self.device = newDevice
        device = select_device(self.device)
        self.weights = weights
        # self.half &= self.device.type != 'cpu'  # half precision only supported on CUDA
        self.model = AutoBackend(self.weights, device=device, dnn=self.dnn, data=self.data, fp16=self.half)
        self.imgsz = check_imgsz(self.imgsz, stride=self.model.stride, min_dim=2)  # check image size
        self.names = self.model.names
        self.model.eval()

    def preprocess(self, img, im0):
        img = LetterBox(self.imgsz, auto=self.model.pt, stride=self.model.stride)(image=im0)
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255  # 0 - 255 to 0.0 - 1.0
        if len(img.shape) == 3:
            img = img[None]  # expand for batch dim
        return img
    
    def postprocess(self, preds, img, orig_img):
        preds = ops.non_max_suppression(preds,
                                        self.conf,
                                        self.iou,
                                        agnostic=self.agnostic_nms,
                                        max_det=self.max_det,
                                        classes=self.classes)
        results = []
        for i, pred in enumerate(preds):
            if len(pred):
                orig_img = orig_img[i] if isinstance(orig_img, list) else orig_img
                shape = orig_img.shape
                pred[:, :4] = ops.scale_boxes(img.shape[2:], pred[:, :4], shape).round()
                for *xyxy, conf, cls in reversed(pred):
                    x1, y1, x2, y2 = xyxy
                    results.append([x1, y1, x2, y2, conf, cls])
        return results
    
    def inference(self, img):
        result = []
        image_copy = img.copy()
        # cv2.imwrite("test.jpg", img)
        img = self.preprocess(img, image_copy)
        pre_time = time.time()
        preds = self.model(img, augment=False, visualize = False)
        print("inference_time: ", int((time.time() - pre_time)*1000), "ms")
        result = self.postprocess(preds, img, image_copy)
        return result