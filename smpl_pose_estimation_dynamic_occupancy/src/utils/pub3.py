#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import os
import sys
from cv_bridge import CvBridge
import cv2
import time
import torch
import argparse
import numpy as np
from pathlib import Path
from collections import Counter
import torch.backends.cudnn as cudnn
from src.utils.general import set_logging
from src.models.common import DetectMultiBackend
from src.utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from src.utils.general import (LOGGER, Profile, check_file, check_img_size, 
                            check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args,
                            scale_coords, strip_optimizer, xyxy2xywh)
from src.utils.plots import Annotator, colors, save_one_box
from src.utils.torch_utils import select_device, time_sync

from std_msgs.msg import String
import skimage
from src.sort import *
import src.realsenseconfig as rs_config 

rs_config.pipeline.start(rs_config.config)

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)

    @staticmethod
    def detect(frame):
        set_logging()  # Set logging level
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')  # Use GPU if available
        model = None  # Initialize the model

        # Load the YOLOv5 model if not already loaded
        

        # Preprocess the frame
        img = torch.from_numpy(frame).to(device)  # Convert frame to torch tensor
        img = img.float() / 255.0  # Normalize pixel values to [0, 1]
        img = img.unsqueeze(0)  # Add batch dimension

        # Perform inference
        with torch.no_grad():
            pred = model(img)  # Pass the frame through the model

        # Post-process the predictions
        # Your post-processing code here...

        # Publish the result (number of detected objects) to a ROS topic
        message = String()
        message.data = str(num_detected_objects)
        self.publisher_.publish(message)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    # Start the RealSense pipeline
    pipeline = rs_config.pipeline.start(rs_config.config)

    try:
        while True:
            # Refresh the frames
            depth_image, color_image = rs_config.D435_para.refresh_mat()

            if depth_image is None or color_image is None:
                continue

            # Call the detect function
            minimal_publisher.detect(color_image)

    except KeyboardInterrupt:
        # Stop the RealSense pipeline
        rs_config.pipeline.stop()

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
