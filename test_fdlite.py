import cv2
from PIL import Image
from fdlite import FaceDetection, FaceDetectionModel

def main():
    # Load image
    image = cv2.imread('face.jpg')
    if image is None:
        print("Error: Could not load face.jpg")
        return
    
    print(f"Image loaded: {image.shape}")
    
    # Initialize fdlite face detector (ultra-fast TFLite)
    print("Initializing fdlite face detector...")
    detect_faces = FaceDetection(model_type=FaceDetectionModel.FRONT_CAMERA)
    
    # Convert BGR to RGB for fdlite
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    pil_image = Image.fromarray(rgb_image)
    img_size = pil_image.size
    
    # Detect faces
    print("Detecting faces...")
    faces = detect_faces(pil_image)
    
    print(f"Faces found: {len(faces)}")
    
    # Draw bounding boxes
    for i, face in enumerate(faces):
        bbox = face.bbox.scaled(img_size)
        
        # Extract coordinates
        x_min = int(bbox.xmin)
        y_min = int(bbox.ymin)
        x_max = int(bbox.xmax)
        y_max = int(bbox.ymax)
        
        # Draw rectangle (green)
        cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        
        # Draw label with face number
        label = f"Face {i+1}"
        cv2.putText(image, label, (x_min, y_min - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        print(f"  Face {i+1}: bbox=({x_min}, {y_min}, {x_max}, {y_max})")
    
    # Save result
    output_path = 'face_detected.jpg'
    cv2.imwrite(output_path, image)
    print(f"Result saved to {output_path}")
    
    # Display
    cv2.imshow("fdlite Face Detection", image)
    print("Press any key to close...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
