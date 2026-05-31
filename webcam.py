import threading
from functools import partial

import cv2
import asyncio
import dlib
import mediapipe as mp
from mediapipe.tasks.python import vision

#Create the tracker we will use
tracker = dlib.correlation_tracker()


class FaceTracker:
    def __init__(self):
        base_options = mp.tasks.BaseOptions(
            model_asset_path="./media/blaze_face_full_range_sparse.tflite",
        )
        options = vision.FaceDetectorOptions(
            base_options=base_options,
        )
        self.detector = vision.FaceDetector.create_from_options(options)
        self.capture_thread = None
        self.capture_running = False
        self.latest_frame = None

    def _capture(self, camera):
        while self.capture_running:
            ret, frame = camera.read()
            if ret:
                self.latest_frame = frame
            else:
                self.latest_frame = None
        camera.release()

    def start_capture(self):
        if self.capture_running:
            return
        camera = cv2.VideoCapture(0)
        self.camera_thread = threading.Thread(
            target=partial(self._capture, camera),
            daemon=True,
        )
        self.capture_running = True
        self.camera_thread.start()

    def stop_capture(self):
        if not self.capture_running:
            return
        self.capture_running = False
        self.camera_thread.join()
        self.camera_thread.stop()

    def track(self):
        if not self.capture_running:
            return
        if self.latest_frame is None:
            return
        frame_rgb = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=frame_rgb,
        )
        result = self.detector.detect(mp_image)
        faces_by_size = sorted(
            result.detections,
            key = lambda d: d.bounding_box.width * d.bounding_box.height,
            reverse=True,
        )
        if not faces_by_size:
            return
        face_box = faces_by_size[0].bounding_box
        img_centre = (mp_image.width / 2, mp_image.height / 2)
        face_centre = (face_box.origin_x + face_box.width / 2, face_box.origin_y + face_box.height / 2)
        face_offset = (face_centre[0] - img_centre[0], face_centre[1] - img_centre[1])
        face_offset_weighted = (face_offset[0] / img_centre[0], face_offset[1] / img_centre[1])
        return face_offset_weighted


async def face_track():
    """
    Utility function for running FaceTracker outside of Dalek control.
    """
    tracker = FaceTracker()
    tracker.start_capture()
    while True:
        tracker.track()
        await asyncio.sleep(0)


class Webcam:
    def __init__(self):
        try:
            self.camera = cv2.VideoCapture(0)
            if not self.camera.isOpened():
                raise Exception("Cannot open camera")
        except Exception as e:
            #logging.error(f"Webcam initialization failed: {e}")
            self.camera = None
        #self.cam = cv2.VideoCapture(0)
        self.face_classifier = cv2.CascadeClassifier(
            cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        )
        self.x_direction = 0
        self.y_direction = 0
        self.face_tracking = False
        #The variable we use to keep track of the fact whether we are
        #currently tracking a face we saw previously
        self.currentlyTrackingFace = 0
        self.count = 0

    def get_direction(self):
        return self.x_direction, self.y_direction

    async def start(self):
        if self.camera is None:
            #logging.error("Webcam is not available. Skipping webcam functionality.")
            return
        #if not self.cam.isOpened():
        #    print("Cannot open camera")
        #    exit(1)
        #    return
        print("Webcam started")
        while True:
            await asyncio.sleep(0.1)
            # Capture frame-by-frame
            # print(self.face_tracking)
            ret, frame = self.camera.read()
            # Rotate frame 180 degrees
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            if self.face_tracking:

                if not ret:
                    print("Can't receive frame (stream end?). Exiting...")
                    break


                # If we are not tracking a face, try to find one
                # print("Currently tracking face:" + str(self.currentlyTrackingFace))
                if (self.currentlyTrackingFace == 0):
                    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    face = self.face_classifier.detectMultiScale(
                        gray_image, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40)
                    )

                    # Sort faces by size
                    face = sorted(face, key=lambda x: x[2] * x[3], reverse=True)
                    i = 0
                    for (x, y, w, h) in face:
                        color = (0, 255, 0) if i==0 else (0, 0, 255)
                        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 4)
                        i += 1

                    if len(face) > 0:
                        # Get the largest face
                        (x, y, w, h) = face[0]
                        print(f"Found face at {x} {y} {w} {h}")
                        self.x_direction = ((x + w / 2) - (frame.shape[1] / 2)) / (frame.shape[1] / 2)
                        self.y_direction = ((y + h / 2) - (frame.shape[0] / 2)) / (frame.shape[0] / 2)
                        self.currentlyTrackingFace = 1
                        tracker.start_track(frame,
                                dlib.rectangle( x,
                                                y,
                                                x+w,
                                                y+h))

                    else:
                        self.x_direction = 0
                        self.y_direction = 0

                else:
                    # If we are tracking a face, update the tracker
                    trackingQuality = tracker.update( frame )

                    #If the tracking quality is good enough, determine the
                    #updated position of the tracked region and draw the
                    #rectangle
                    if trackingQuality >= 8.75:
                        tracked_position =  tracker.get_position()

                        t_x = int(tracked_position.left())
                        t_y = int(tracked_position.top())
                        t_w = int(tracked_position.width())
                        t_h = int(tracked_position.height())
                        cv2.rectangle(frame, (t_x, t_y),
                                    (t_x + t_w , t_y + t_h),
                                    (0, 255, 0) ,2)

                        self.x_direction = ((t_x + t_w / 2) - (frame.shape[1] / 2)) / (frame.shape[1] / 2)
                        self.y_direction = ((t_y + t_h / 2) - (frame.shape[0] / 2)) / (frame.shape[0] / 2)


                    else:
                        #If the quality of the tracking update is not
                        #sufficient (e.g. the tracked region moved out of the
                        #screen) we stop the tracking of the face and in the
                        #next loop we will find the largest face in the image
                        #again
                        self.currentlyTrackingFace = 0

            # Save image
            self.count = (self.count + 1) % 100
            if (self.count == 0):
                print("writing image")
                cv2.imwrite("face.jpg", frame)

        # # Release the webcam
        self.camera.release()
