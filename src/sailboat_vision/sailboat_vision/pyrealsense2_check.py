import pyrealsense2 as rs

# Create a pipeline
pipeline = rs.pipeline()

# Start streaming
pipeline.start()

# Try to get a frame
frames = pipeline.wait_for_frames()
print(frames)

# Stop the pipeline
pipeline.stop()
