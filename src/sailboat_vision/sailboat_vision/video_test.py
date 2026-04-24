import cv2
from ultralytics import YOLO

class BuoyDetector:
    def __init__(self):
        # self.model = YOLO("yolo11x.pt")
        self.model = YOLO("buoy2.pt")
        self.cap = cv2.VideoCapture(2)
        # self.cap.set(cv2.CAP_PROP_FPS, 1)
        # print("fps: ", self.cap.get(cv2.CAP_PROP_FPS))

        # fourcc = cv2.VideoWriter_fourcc(*'avc1') # Example: MPEG-4 codec
        # fps = 30
        # width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        # height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        # frame_size = (width, height)
        # is_color = True
        # self.out = cv2.VideoWriter('IMG_0380_out2.mov', fourcc, fps, frame_size, is_color)
    
    def detect(self, image):
        results = self.model(image, stream=True, conf=0.6)
        return results
    
    def center(self, box):
        x1, y1, x2, y2 = box.xyxy[0]
        return (int((x1 + x2) / 2), int((y1 + y2) / 2))
    
    def show(self, image, results):
        # for result in results:
        #     for box in result.boxes:
        #         print(self.center(box))
                # cv2.rectangle(image, (int(box.xyxy[0][0]), int(box.xyxy[0][1])),
                #             (int(box.xyxy[0][2]), int(box.xyxy[0][3])), (255, 0, 0), 3)
                # cv2.putText(image, f"{result.names[int(box.cls[0])]} {round(float(box.conf), 2)}",
                #             (int(box.xyxy[0][0]), int(box.xyxy[0][1]) - 10),
                #             cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 0), 3)
                # box_center = self.center(box)
                # cv2.circle(image, box_center, 3, (255, 0, 0), -1)
        # self.out.write(image)
        cv2.imshow('Results', image)
    
    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to grab frame")
                break
            print("???")
            results = self.detect(frame)
            self.show(frame, results)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    buoy_detector = BuoyDetector()
    buoy_detector.run()