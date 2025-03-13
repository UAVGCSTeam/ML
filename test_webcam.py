import cv2
import sys

def test_webcam(camera_index=4):
    # Initialize webcam
    print(f"Attempting to open webcam at index {camera_index}")
    cap = cv2.VideoCapture(camera_index)
    
    # Check if the webcam is opened correctly
    if not cap.isOpened():
        print(f"Error: Could not open webcam at index {camera_index}")
        return False
    
    print("Webcam opened successfully. Press 'q' to quit.")
    
    # Read and display webcam feed
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        # If frame is not read correctly, break
        if not ret:
            print("Error: Failed to capture frame from webcam")
            break
        
        # Display the resulting frame
        cv2.imshow('Webcam Test', frame)
        
        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release the webcam and close windows
    cap.release()
    cv2.destroyAllWindows()
    return True

if __name__ == "__main__":
    # Default camera index is 0, but user can specify another index as command line argument
    camera_index = 1
    if len(sys.argv) > 1:
        try:
            camera_index = int(sys.argv[1])
        except ValueError:
            print(f"Error: Invalid camera index '{sys.argv[1]}'. Using default index 0.")
    
    while(test_webcam(camera_index) == False):
        camera_index += 1
        print(f"Trying next camera index: {camera_index}")
    # success = test_webcam(camera_index)
    
    if success:
        print("Webcam test completed successfully.")
    else:
        print("Webcam test failed. Make sure your camera is connected and not in use by another application.")
        print("You might also try a different camera index (e.g., python test_webcam.py 1)")

