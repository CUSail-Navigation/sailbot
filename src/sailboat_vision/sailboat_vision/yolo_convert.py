from ultralytics import YOLO

# Load YOLO model
model = YOLO("buoy2.pt")

# Export to TensorRT engine
model.export(format="engine", device='cpu', half=True)