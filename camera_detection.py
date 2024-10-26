import cv2
import numpy as np

class LightDetector:
    def __init__(self):
        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        
        # Light detection parameters
        self.light_threshold = 200
        self.min_light_area = 100
        self.max_light_area = 10000

    def cleanup(self):
        """Clean up resources"""
        self.cap.release()
        cv2.destroyAllWindows()

    def detect_light(self, frame):
        """
        Detect the brightest spot in the frame
        Returns: (x,y) coordinates of brightest spot and its area
        """
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Threshold the image
        _, thresh = cv2.threshold(gray, self.light_threshold, 255, cv2.THRESH_BINARY)
        
        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, 
                                     cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, 0
        
        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        if area < self.min_light_area:
            return None, 0
            
        # Get the centroid
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return None, 0
            
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        
        return (cx, cy), area

    def run(self):
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    continue

                height, width = frame.shape[:2]
                
                # Detect light
                light_pos, light_area = self.detect_light(frame)
                
                if light_pos:
                    # Visual feedback
                    cv2.circle(frame, light_pos, 10, (0, 255, 0), -1)
                    cv2.putText(frame, f"Position: ({light_pos[0]}, {light_pos[1]})", 
                              (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f"Area: {light_area}", (10, 60),
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    # Print position and area to console
                    print(f"Light Position: ({light_pos[0]}, {light_pos[1]}), Area: {light_area}")
                else:
                    print("No light detected")
                
                cv2.imshow('Light Detection', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.cleanup()

def main():
    detector = LightDetector()
    detector.run()

if __name__ == "__main__":
    main()
