import cv2
from signal import pause
import asyncio
import dlib

#Create the tracker we will use
tracker = dlib.correlation_tracker()


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
        #The variable we use to keep track of the fact whether we are
        #currently tracking a face we saw previously
        self.currentlyTrackingFace = 0
        self.count = 0;

    def get_direction(self):
        return self.x_direction, self.y_direction

    async def start(self):
        if not self.cam.isOpened():
            print("Cannot open camera")
            return
        print("Webcam started")
        await asyncio.sleep(0)
        while True:
            await asyncio.sleep(0)
            # Capture frame-by-frame
            # print(self.face_tracking)
            ret, frame = self.cam.read()
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
                    i = 0;
                    for (x, y, w, h) in face:
                        
                        color = (0, 255, 0) if i==0 else (0, 0, 255)
                        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 4)
                        i += 1

                    if len(face) > 0:
                        # Get the largest face
                        (x, y, w, h) = face[0]
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
            # self.count = (self.count + 1) % 25
            # print(self.count)
            # if (self.count == 0):
            #     print("writing image")
            #     cv2.imwrite("face.jpg", frame)
            # cv2.imwrite("face.jpg", frame)

        # # Release the webcam
        self.cam.release()
