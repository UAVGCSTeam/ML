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
        self.device = "cpu"
        self.dataset = ""
        self.names = ""
        self.weights = "yolov8n.pt"
        self.half = False
        self.dnn = False
        self.agnostic_nms = False
        self.model = None
        self.classes = None

    def set_up_model(self, weights):
        device = select_device(self.device)
        self.weights = weights
        # self.half &= self.device.type != 'cpu'  # half precision only supported on CUDA
        self.model = AutoBackend(self.weights, device=device, dnn=self.dnn, data=self.data, fp16=self.half)
        self.imgsz = check_imgsz(self.imgsz, stride=self.model.stride, min_dim=2)  # check image size
        self.names = self.model.names
        self.device = device
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
    
if __name__ == "__main__":
    yolov8 = Yolov8()
    yolov8.set_up_model("yolov8n.pt")
    # path = "D:/yolov5/test.mp4" # I commented this old lsat semester path out -Carlos
    path = "./test_folder/test_video.mp4" # I added this new path -Carlos
    cap = cv2.VideoCapture(path)
    yolov8.model.warmup(imgsz=(1 , 3, *yolov8.imgsz))
    while True:
        ret, frame = cap.read()
        if not ret:
           cap = cv2.VideoCapture(path)
           continue
        results = yolov8.inference(frame)
        for result in results:
            x1, y1, x2, y2, conf, cls = result
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, str(yolov8.names[int(cls)]), (int(x1), int(y1)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow("frame", frame)
        # press Esc to close the window
        if (cv2.waitKey(30) == 27):
            break