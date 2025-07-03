#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from typing import List

from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import Point

import torch
from PIL import Image as PILImage
from typing import List

import sys
sys.path.append('/home/group8/Project/src/detection/dl_train')
from detector import Detector
import utils as utils

import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
from torchvision.transforms import v2, ToTensor, Normalize
#from runyu_interfaces.msg import StringArray
from std_msgs.msg import String
import time
import os

import matplotlib.pyplot as plt
from tf2_geometry_msgs import PoseStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, TransformStamped

# os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "max_split_size_mb:512"
# torch.cuda.set_per_process_memory_fraction(0.6,0)
# torch.cuda.empty_cache()
class DL_detection_cls(Node):
    def __init__(self):
        super().__init__('dl_detection_cls')
        #Para
        self.cuda = torch.cuda.is_available()
        self.ros_img = None
        self.PILimage = None
        self.NParray_img = None
        self.start_time = time.time()
        self.device = "cuda" #NUC integrated GPU may better than that shit, but need plugins
        
        #Dictionary
        self.category_dic = categories = [
            {"id":0, "name":"Nothing", "supercategory":"none"},
            {"id":1, "name":"bc", "supercategory":"cube"},
            {"id":2, "name":"binky", "supercategory":"animals"},
            {"id":3, "name":"box", "supercategory":"box"},
            {"id":4, "name":"bs", "supercategory":"sphere"},
            {"id":5, "name":"gc", "supercategory":"cube"},
            {"id":6, "name":"gs", "supercategory":"sphere"},
            {"id":7, "name":"hugo", "supercategory":"animals"},
            {"id":8, "name":"kiki", "supercategory":"animals"},
            {"id":9, "name":"muddles", "supercategory":"animals"},
            {"id":10, "name":"oakie", "supercategory":"animals"},
            {"id":11, "name":"rc", "supercategory":"cube"},
            {"id":12, "name":"rs", "supercategory":"sphere"},
            {"id":13, "name":"slush", "supercategory":"animals"},
            {"id":14, "name":"wc", "supercategory":"cube"}
        ]
        self.id2name = {category["id"]: category["name"] for category in categories}
        self.id2supercategory = {category["id"]: category["supercategory"] for category in categories}

        #Normalize
        self.input_transforms = v2.Compose(
        [
            v2.ToImage(),
            v2.ToDtype(torch.float32, scale=True),
            v2.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ]
        )
        
        # Load Deep Learning model
        self.detector = Detector().to(self.device)
        model_path = "/home/group8/Project/src/detection/dl_train/model/Iteration30000-LearningRate1e-05-Timedet_2024-02-26_17-15-11-658950.pt"
        example_forward_input = torch.rand(1, 3, 480, 640)
        self.model = self.load_model(self.detector, model_path, self.device).eval()
        # self.model_opt = torch.jit.trace(self.model, example_forward_input).to(self.device)
        # self.model_opt.eval().to(self.device)

        self.bridge = CvBridge()

        #Sub
        self.image_sub = self.create_subscription(Image, "/camera/color/image_raw", self.image_sub_callback, 10)

        #Pub
        self.boundingbox_pub = self.create_publisher(Image, "/detection/bounding_boxes", 10) #StringArray

    def load_model(self,model: torch.nn.Module, path: str, device: str) -> torch.nn.Module:
        """Load model weights from disk.

        Args:
            model: The model to load the weights into.
            path: The path from which to load the model weights.
            device: The device the model weights should be on.

        Returns:
            The loaded model (note that this is the same object as the passed model).
        """
        state_dict = torch.load(path, map_location=device)
        model.load_state_dict(state_dict)
        return model
    
    def image_sub_callback(self, msg: Image):
        try:
            #Restore RawMsg
            self.ros_img = msg
            time_stamp = msg.header.stamp
            #Transform format
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            cv_img = cv2.resize(cv_img, (640,480), interpolation=cv2.INTER_LINEAR)
            self.PILimage = PILImage.fromarray(cv_img)
            self.NParray_img = np.asarray(cv_img)
            input_img = torch.stack([self.input_transforms(cv_img)]).to(self.device)

            #Get BoundingBox
            with torch.no_grad():
                out = self.model(input_img).cpu()
                bbs = self.model.out_to_bbs(out, threshold = 0.8) 
                                    #confidence_threshold &= threshold
            # print(bbs)
            bbs_nms = self.non_max_suppresion(bbs, confidence_threshold=0.8, IoU_threshold= 0.3, diff_class_thresh=0.75)
           
            #Draw detection BoundingBox
            draw_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            for bb in bbs_nms:
                x, y, width, height, confi, category = int(bb["x"]), int(bb["y"]), int(bb["width"]), int(bb["height"]), bb["score"], bb["category"]
                name = self.id2name[category]
                supercategory = self.id2supercategory[category]
                self.get_logger().info(f"Category: {supercategory}/{name}, Score:{confi}")
                cv2.rectangle(draw_img, (x, y), (x+width, y+height), color=(0, 0, 255), thickness=2)
                cv2.putText(draw_img, f'{supercategory}/{name}: {confi:.2f}', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            #Draw detection FPS
            FPS = round(1 / (time.time() - self.start_time), 1)
            self.start_time = time.time()
            cv2.putText(draw_img, f'FPS: {FPS}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
            #CV2 Window Test
            cv2.imshow('Detection window', draw_img)
            cv2.waitKey(1)

            #Publish Detection Img
            if bbs_nms != None:
                # bbs_nms = np.array(bbs_nms)
                pub_img = self.bridge.cv2_to_imgmsg(draw_img, encoding="bgr8")
                self.boundingbox_pub.publish(pub_img)
                # self.get_logger().info("Publish detection topic successful")

        except CvBridgeError as e:
            self.get_logger().error('Could not convert image: {}'.format(e))

    def non_max_suppresion(self,
        bbs: List[utils.BoundingBox],
        confidence_threshold=0.5,
        IoU_threshold=0.5,
        diff_class_thresh=0.95,
        ) -> List[utils.BoundingBox]:
        
        thresholded_bbs = []
        res = []
        if (len(bbs[0]) == 0):
            return []
        sorted_bboxes = sorted(bbs[0], reverse=True, key=lambda x: x["score"])
        for bbox in sorted_bboxes:
            if bbox["score"] > confidence_threshold:
                thresholded_bbs.append(bbox)

        while len(thresholded_bbs) > 0:
            cur_bb = thresholded_bbs.pop(0)
            res.append(cur_bb)
            for bb in thresholded_bbs:
                if cur_bb["category"] == bb["category"]:
                    iou = self.bb_IoU(cur_bb, bb)
                    if iou > IoU_threshold:
                        thresholded_bbs.remove(bb)
                elif self.bb_IoU(cur_bb, bb) > diff_class_thresh:
                    thresholded_bbs.remove(bb)
        return res
    def bb_IoU(self, bb1: utils.BoundingBox, bb2: utils.BoundingBox):
        x1, y1, x2, y2 = (
            bb1["x"],
            bb1["y"],
            bb1["x"] + bb1["width"],
            bb1["y"] + bb1["height"],
        )
        x3, y3, x4, y4 = (
            bb2["x"],
            bb2["y"],
            bb2["x"] + bb2["width"],
            bb2["y"] + bb2["height"],
        )

        x_inter1 = max(x1, x3)
        y_inter1 = max(y1, y3)
        x_inter2 = min(x2, x4)
        y_inter2 = min(y2, y4)
        width_inter = abs(x_inter2 - x_inter1)
        height_inter = abs(y_inter2 - y_inter1)
        area_inter = width_inter * height_inter
        width_box1 = abs(x2 - x1)
        height_box1 = abs(y2 - y1)
        width_box2 = abs(x4 - x3)
        height_box2 = abs(y4 - y3)
        area_box1 = width_box1 * height_box1
        area_box2 = width_box2 * height_box2
        area_union = area_box1 + area_box2 - area_inter
        iou = area_inter / area_union

        return iou




def main():
    rclpy.init()
    node = DL_detection_cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()