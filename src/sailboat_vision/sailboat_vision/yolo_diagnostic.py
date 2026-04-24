import os
import subprocess
import time

import torch

RUN_YOLO = True

class BuoyDetector:
    def __init__(self):
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
        results = self.model(image, stream=True, conf=0.6, device='cuda')
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
            frames = self.pipeline.wait_for_frames()

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
    print("==== SYSTEM INFO ====")
    print("PyTorch version:", torch.__version__)
    print("CUDA available:", torch.cuda.is_available())

    if torch.cuda.is_available():
        print("CUDA device:", torch.cuda.get_device_name(0))
        print("CUDA capability:", torch.cuda.get_device_capability(0))
    else:
        print("WARNING: CUDA NOT AVAILABLE")

    print("\n==== Checking tegrastats ====")
    print("Run this manually in another terminal:")
    print("   sudo tegrastats")
    
    if (RUN_YOLO):
        import cv2
        import numpy as np
        import pyrealsense2 as rs
        from ultralytics import YOLO

        import time

        print("\n==== RUNNING BUOY DETECTOR ====")
        for i in range(10):
            try:
                torch.cuda.init()
                torch.cuda.synchronize()
                break
            except Exception as e:
                print("CUDA init failed, retrying...", e)
                time.sleep(1)

        buoy_detector = BuoyDetector()
        buoy_detector.run()
    else:
        print("\n==== GPU Speed Benchmark ====")

        if torch.cuda.is_available():
            device = "cuda"
        else:
            device = "cpu"

        model = torch.nn.Sequential(
            torch.nn.Conv2d(3, 32, 3),
            torch.nn.ReLU(),
            torch.nn.Conv2d(32, 64, 3),
            torch.nn.ReLU()
        ).to(device)

        input_tensor = torch.randn(1, 3, 640, 640).to(device)

        # Warmup
        for _ in range(10):
            _ = model(input_tensor)

        torch.cuda.synchronize() if device == "cuda" else None

        start = time.time()

        for _ in range(100):
            _ = model(input_tensor)

        torch.cuda.synchronize() if device == "cuda" else None

        end = time.time()

        fps = 100 / (end - start)
        print(f"\nInference speed: {fps:.2f} FPS")

"""
sudo docker run -it --rm --name ros2_container \
  --privileged \
  --runtime=nvidia \
  --gpus all \
  --ipc host \
  -v /dev:/dev \
  -v /usr/local/cuda:/usr/local/cuda \
  -v /usr/lib/aarch64-linux-gnu/:/usr/lib/aarch64-linux-gnu \
  -v $(pwd)/src:/home/ros2_user/ros2_ws/src \
  ros2_humble_custom_cv


Pip install torch and torchvision from here: https://pypi.jetson-ai-lab.io/jp6/cu126
for now it's
RUN pip3 install --no-cache-dir torch torchvision \
  --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 \

export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH

maybe have to do inside the container
apt-get update
apt-get install -y --no-install-recommends \
    libopenblas-base \
    libopenblas-dev \
    libblas-dev \
    liblapack-dev \
    libc6-dev \
    build-essential \
    wget \
    ca-certificates

and then probably this inside the container
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get -y install cudss

gpu might be out of memory sometimes

opencv-python==4.8.0.74
ultralytics==8.3.228
"""