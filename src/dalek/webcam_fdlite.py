import asyncio
import cv2
import numpy as np
from PIL import Image
from fdlite import FaceDetection, FaceDetectionModel


class Webcam:
    """Face detection and tracking using fdlite with adaptive targeting."""
    
    def __init__(self):
        try:
            self.camera = cv2.VideoCapture(0)
            if not self.camera.isOpened():
                raise Exception("Cannot open camera")
        except Exception as e:
            print(f"Camera initialization error: {e}")
            self.camera = None
        
        # Initialize fdlite detector upfront
        print("Initializing fdlite face detector...")
        try:
            self.detector = FaceDetection(model_type=FaceDetectionModel.FULL_SPARSE)
            print("fdlite detector ready (FULL_SPARSE)")
        except Exception as e:
            print(f"Failed to initialize fdlite FULL_SPARSE: {e}")
            try:
                self.detector = FaceDetection(model_type=FaceDetectionModel.FRONT_CAMERA)
                print("fdlite detector ready (fallback FRONT_CAMERA)")
            except Exception as e2:
                print(f"Failed to initialize fallback fdlite FRONT_CAMERA: {e2}")
                self.detector = None
        
        # Face tracking state
        self.x_direction = 0
        self.y_direction = 0
        self.face_tracking = False
        
        # Adaptive tracking parameters
        self.tracked_center = None        # (x, y) of currently tracked face
        self.lost_frame_count = 0         # Grace period counter
        self.MAX_LOST_FRAMES = 20         # ~0.3 seconds at 30fps before resetting
        self.MAX_DISTANCE_THRESHOLD = 200 # Max pixels face can move between frames
        
        self.count = 0
    
    def _get_box_center(self, bbox, img_size):
        """Calculate pixel coordinates of a bounding box center."""
        scaled = bbox.scale(img_size)
        cx = int((scaled.xmin + scaled.xmax) / 2)
        cy = int((scaled.ymin + scaled.ymax) / 2)
        return (cx, cy), scaled

    def _annotate_faces(self, frame, faces, img_size):
        """Return a copy of the frame with bounding boxes drawn around detected faces."""
        annotated = frame.copy()
        for face in faces:
            _, scaled_box = self._get_box_center(face.bbox, img_size)
            x1 = int(scaled_box.xmin)
            y1 = int(scaled_box.ymin)
            x2 = int(scaled_box.xmax)
            y2 = int(scaled_box.ymax)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
        return annotated
    
    def _detect_faces(self, frame):
        """Detect faces in frame using fdlite."""
        if self.detector is None:
            return []
        
        # Convert BGR to RGB for fdlite
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(rgb_frame)
        
        try:
            faces = self.detector(pil_image)
            return faces, pil_image.size
        except Exception as e:
            print(f"Face detection error: {e}")
            return [], (frame.shape[1], frame.shape[0])
    
    def get_direction(self):
        """Return normalized face direction (-1 to 1)."""
        return self.x_direction, self.y_direction

    async def start(self):
        """Main face tracking loop."""
        if self.camera is None:
            print("Webcam is not available. Skipping webcam functionality.")
            return
        
        print("Webcam started with fdlite tracking")
        FRAME_CENTER = (320, 240)  # Center of typical 640x480 frame
        
        while True:
            await asyncio.sleep(0.033)  # ~30fps
            
            ret, frame = self.camera.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting...")
                break
            
            # Rotate frame 180 degrees
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            
            annotated_frame = None
            if self.face_tracking:
                if self.detector is None:
                    self.x_direction = 0
                    self.y_direction = 0
                    continue
                
                # Detect faces
                faces, img_size = self._detect_faces(frame)
                if faces:
                    annotated_frame = self._annotate_faces(frame, faces, img_size)
                
                best_match_face = None
                min_distance = float('inf')
                
                if faces:
                    # CASE 1: Actively tracking. Find the target among detected faces.
                    if self.tracked_center is not None:
                        for face in faces:
                            cx_cy, scaled_box = self._get_box_center(face.bbox, img_size)
                            # Euclidean distance from target's last known position
                            dist = np.linalg.norm(np.array(cx_cy) - np.array(self.tracked_center))
                            
                            if dist < min_distance and dist < self.MAX_DISTANCE_THRESHOLD:
                                min_distance = dist
                                best_match_face = (cx_cy, scaled_box)
                        
                        if best_match_face:
                            self.tracked_center = best_match_face[0]
                            self.lost_frame_count = 0
                        else:
                            # Target wasn't found this frame (occlusion, fast movement)
                            self.lost_frame_count += 1
                    
                    # CASE 2: No active target, or target officially lost. Lock onto centermost face.
                    if self.tracked_center is None or self.lost_frame_count > self.MAX_LOST_FRAMES:
                        self.tracked_center = None
                        best_center_dist = float('inf')
                        
                        for face in faces:
                            cx_cy, scaled_box = self._get_box_center(face.bbox, img_size)
                            # Distance from screen center
                            dist_to_screen_center = np.linalg.norm(
                                np.array(cx_cy) - np.array(FRAME_CENTER)
                            )
                            
                            if dist_to_screen_center < best_center_dist:
                                best_center_dist = dist_to_screen_center
                                best_match_face = (cx_cy, scaled_box)
                        
                        if best_match_face:
                            self.tracked_center = best_match_face[0]
                            self.lost_frame_count = 0
                            print("New Target found!")
                else:
                    # No faces detected
                    if self.tracked_center is not None:
                        self.lost_frame_count += 1
                
                # Clear target if lost too long
                if self.lost_frame_count > self.MAX_LOST_FRAMES and self.tracked_center is not None:
                    print("Target lost. Searching...")
                    self.tracked_center = None
                
                # Calculate direction output
                if self.tracked_center is not None and self.lost_frame_count == 0:
                    cx, cy = self.tracked_center
                    # Normalize to -1 to 1 range
                    self.x_direction = (cx - FRAME_CENTER[0]) / FRAME_CENTER[0]
                    self.y_direction = (cy - FRAME_CENTER[1]) / FRAME_CENTER[1]
                else:
                    self.x_direction = 0
                    self.y_direction = 0
            else:
                # Face tracking disabled
                self.x_direction = 0
                self.y_direction = 0
            
            # Save frame periodically
            self.count = (self.count + 1) % 100
            if self.count == 0:
                cv2.imwrite("face.jpg", frame)
                if self.face_tracking:
                    debug_frame = annotated_frame if annotated_frame is not None else frame
                    cv2.imwrite("face_debug.jpg", debug_frame)
        
        self.camera.release()
