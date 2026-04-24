# sudo docker run -it --rm --name ros2_container \
#   --privileged \
#   --runtime nvidia \
#   -v /dev:/dev \
#   -v $(pwd)/src:/home/ros2_user/ros2_ws/src \
#   ros2_humble_custom_cv
#
#
#
# pip uninstall numpy -y

# FROM ubuntu:22.04

# ENV DEBIAN_FRONTEND=noninteractive \
#     PYTHON_VERSION=3.10

# # Install Python + dependencies
# RUN apt-get update && apt-get install -y --no-install-recommends \
#         python${PYTHON_VERSION} \
#         python${PYTHON_VERSION}-dev \
#         python3-pip \
#         wget \
#         ca-certificates \
#         git \
#         libopenblas-dev \
#         curl \
#     && rm -rf /var/lib/apt/lists/*

# # Upgrade pip
# RUN python3 -m pip install --upgrade pip numpy packaging

# # Install Jetson PyTorch wheel (ARM64 + CUDA)
# RUN python3 -m pip install \
#     --no-cache-dir \
#     --extra-index-url https://pypi.jetson-ai-lab.dev/jp6/cu126 \
#     torch torchvision

# # Test PyTorch + CUDA
# RUN python3 - <<'EOF'
# import torch
# print("Torch version:", torch.__version__)
# print("CUDA available:", torch.cuda.is_available())
# EOF

# CMD ["/bin/bash"]


import time

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

class BuoyDetector:
    def __init__(self):
        
        import torch
        print("=====")
        print(torch.cuda.is_available())
        print(torch.cuda.get_device_name(0))

        # Initialize camera
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        for _ in range(10):
            self.pipeline.wait_for_frames()
        
        self.model = YOLO("buoy2.pt").to('cuda')

    def detect(self, image):
        results = self.model(image, stream=True, conf=0.6)
        return results
    
    def center(self, box):
        x1, y1, x2, y2 = box.xyxy[0]
        return (int((x1 + x2) / 2), int((y1 + y2) / 2))
    
    def show(self, image, results):
        for result in results:
            for box in result.boxes:
                cv2.rectangle(image, (int(box.xyxy[0][0]), int(box.xyxy[0][1])),
                            (int(box.xyxy[0][2]), int(box.xyxy[0][3])), (255, 0, 0), 3)
                cv2.putText(image, f"{result.names[int(box.cls[0])]} {round(float(box.conf), 2)}",
                            (int(box.xyxy[0][0]), int(box.xyxy[0][1]) - 10),
                            cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 0), 3)
                box_center = self.center(box)
                cv2.circle(image, box_center, 3, (255, 0, 0), -1)
        # cv2.imshow('Results', image)
    
    def run(self):
        while True:
            frames = self.pipeline.wait_for_frames(30000)

            # Align the depth frame to color frame
            aligned_frames = self.align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not aligned_depth_frame or not color_frame:
                return

            # Check color frame for buoy
            color_image = np.asanyarray(color_frame.get_data())
            results = self.detect(color_image)
            self.show(color_image, results)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    buoy_detector = BuoyDetector()
    buoy_detector.run()