import cv2

def stream_gopro():
    # Try different device indices if 0 doesn't work
    # On macOS, you might need to use indices like 0, 1, or 2
    cap = cv2.VideoCapture(0)
    
    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open GoPro camera.")
        return
    
    # Optional: Set frame properties
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    print("Streaming from GoPro. Press 'q' to quit.")
    
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        # If frame is read correctly ret is True
        if not ret:
            print("Error: Can't receive frame. Exiting...")
            break
        
        # Display the resulting frame
        cv2.imshow('GoPro Stream', frame)
        
        # Process the frame here as needed
        # Your image processing code would go here
        
        # Press 'q' to exit
        if cv2.waitKey(1) == ord('q'):
            break
    
    # Release resources
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    stream_gopro()