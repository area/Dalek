import cv2
from signal import pause
import asyncio

# Define your webcam-related functions here

class Webcam:
    def __init__(self):
        self.cam = cv2.VideoCapture(0)
        self.face_classifier = cv2.CascadeClassifier(
            cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        )
        self.x_direction = 0
        self.y_direction = 0
        self.face_tracking = False;

    def get_direction(self):
        return self.x_direction, self.y_direction

    async def start(self):
        if not self.cam.isOpened():
            print("Cannot open camera")
            return
        while True:
            await asyncio.sleep(0)
            # Capture frame-by-frame
            if self.face_tracking:
                ret, frame = self.cam.read()
                if not ret:
                    print("Can't receive frame (stream end?). Exiting...")
                    break

                gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                face = self.face_classifier.detectMultiScale(
                    gray_image, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40)
                )

                for (x, y, w, h) in face:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 4)

                # Sort faces by size
                face = sorted(face, key=lambda x: x[2] * x[3], reverse=True)
                if len(face) > 0:
                    # Get the largest face
                    (x, y, w, h) = face[0]
                    self.x_direction = ((x + w / 2) - (frame.shape[1] / 2)) / (frame.shape[1] / 2)
                    self.y_direction = ((y + h / 2) - (frame.shape[0] / 2)) / (frame.shape[0] / 2)
                else:
                    self.x_direction = 0
                    self.y_direction = 0

                # Save image
                # cv2.imwrite("face.jpg", frame)

        # # Release the webcam
        self.cam.release()
