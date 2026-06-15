import cv2
import mediapipe as mp
from mediapipe.tasks.python import vision

# Load image
image = cv2.imread('face.jpg')
if image is None:
    print("Error: Could not load face.jpg")
else:
    print(f"Image loaded: {image.shape}")
    
    # Initialize MediaPipe face detector
    base_options = mp.tasks.BaseOptions(
        model_asset_path="./media/blaze_face_full_range_sparse.tflite",
    )
    options = vision.FaceDetectorOptions(base_options=base_options)
    detector = vision.FaceDetector.create_from_options(options)
    
    # Convert to RGB and create MediaPipe image
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image_rgb)
    
    # Detect faces
    print("Detecting faces...")
    result = detector.detect(mp_image)
    
    # Report results
    print(f"Faces found: {len(result.detections)}")
    for i, detection in enumerate(result.detections):
        box = detection.bounding_box
        print(f"  Face {i}: confidence={detection.categories[0].score:.2f}, "
              f"box=({box.origin_x:.0f}, {box.origin_y:.0f}, "
              f"w={box.width:.0f}, h={box.height:.0f})")