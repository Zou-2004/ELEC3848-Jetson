import cv2
import numpy as np

class SimpleAirplaneTracker:
    def __init__(self, template_paths):
        """
        Initialize tracker with template images
        template_paths: list of paths to template images
        """
        self.templates = []
        for path in template_paths:
            template = cv2.imread(path)
            if template is not None:
                # Convert to grayscale and store both color and gray versions
                template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
                self.templates.append((template, template_gray))

    def find_airplane(self, frame, threshold=0.8):
        """
        Find airplane using template matching
        Returns: (center_x, center_y) if found, None if not found
        """
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        best_val = -1
        best_loc = None
        best_size = None

        for template, template_gray in self.templates:
            # Try different scales
            for scale in [0.5, 0.75, 1.0, 1.25, 1.5]:
                # Resize template
                width = int(template_gray.shape[1] * scale)
                height = int(template_gray.shape[0] * scale)
                dim = (width, height)
                resized_template = cv2.resize(template_gray, dim, interpolation=cv2.INTER_AREA)

                # Template matching
                result = cv2.matchTemplate(frame_gray, resized_template, cv2.TM_CCOEFF_NORMED)
                min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

                if max_val > best_val:
                    best_val = max_val
                    best_loc = max_loc
                    best_size = (width, height)

        # If confidence is high enough
        if best_val > threshold and best_loc is not None:
            center_x = best_loc[0] + best_size[0]//2
            center_y = best_loc[1] + best_size[1]//2
            return (center_x, center_y, best_val)
        return None

    def find_airplane_color(self, frame, color_lower, color_upper):
        """
        Find airplane using color segmentation
        color_lower, color_upper: BGR color bounds for the airplane
        """
        # Create mask for color
        mask = cv2.inRange(frame, color_lower, color_upper)
        
        # Remove noise
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Filter by area if needed
            if cv2.contourArea(largest_contour) > 500:  # Minimum area threshold
                # Get the center of the contour
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])
                    return (center_x, center_y)
        
        return None

def main():
    # Initialize tracker with template images
    template_paths = ["C:/Users/DELL/Desktop/3848 project/tracker/me1.png",
                      "C:/Users/DELL/Desktop/3848 project/tracker/me2.png",
                      "C:/Users/DELL/Desktop/3848 project/tracker/me3.png"
                      ] 
    tracker = SimpleAirplaneTracker(template_paths)

    # Define color range for the airplane (example: white airplane)
    # Adjust these values based on your airplane's color
    color_lower = np.array([200, 200, 200])  # BGR values
    color_upper = np.array([255, 255, 255])  # BGR values

    # Start video capture
    cap = cv2.VideoCapture(0)  # Use 0 for webcam or video file path

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Try template matching
        template_result = tracker.find_airplane(frame)
        
        # Try color detection
        color_result = tracker.find_airplane_color(frame, color_lower, color_upper)

        # Draw results
        if template_result:
            x, y, conf = template_result
            cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
            cv2.putText(frame, f"Template Match: ({x}, {y})", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        if color_result:
            x, y = color_result
            cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"Color Match: ({x}, {y})", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow('Tracking', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()