import cv2
import numpy as np

class SimpleAirplaneTracker:
    def __init__(self, template_paths):
        """
        Initialize tracker with template images
        template_paths: list of paths to template images
        """
        self.templates = []
        self.template_sizes = []
        for path in template_paths:
            template = cv2.imread(path)
            if template is not None:
                # Convert to grayscale
                template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
                self.templates.append(template_gray)
                self.template_sizes.append(template_gray.shape)

    def resize_frame_to_template(self, frame, template_size):
        """
        Resize frame to create pyramid of different sizes
        """
        frame_height, frame_width = frame.shape[:2]
        template_height, template_width = template_size
        
        # Calculate the ratio between template and frame
        width_ratio = frame_width / template_width
        height_ratio = frame_height / template_height
        
        scales = []
        resized_frames = []
        
        # Create image pyramid with different scales
        scale = 1.0
        while scale * template_width < frame_width and scale * template_height < frame_height:
            # Resize frame
            width = int(frame_width / scale)
            height = int(frame_height / scale)
            resized = cv2.resize(frame, (width, height))
            
            resized_frames.append(resized)
            scales.append(scale)
            
            scale *= 1.5  # Increase scale by 50% each time
            
        return resized_frames, scales

    def find_airplane(self, frame, threshold=0.7):
        """
        Find airplane using template matching with frame resizing
        Returns: (center_x, center_y, confidence) if found, None if not found
        """
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        best_val = -1
        best_loc = None
        best_scale = None
        best_template_idx = None

        # Try each template
        for template_idx, template in enumerate(self.templates):
            # Get different frame sizes
            resized_frames, scales = self.resize_frame_to_template(frame_gray, self.template_sizes[template_idx])
            
            # Try template matching on each resized frame
            for resized_frame, scale in zip(resized_frames, scales):
                # Template matching
                result = cv2.matchTemplate(resized_frame, template, cv2.TM_CCOEFF_NORMED)
                min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

                if max_val > best_val:
                    best_val = max_val
                    best_loc = max_loc
                    best_scale = scale
                    best_template_idx = template_idx

        # If confidence is high enough
        if best_val > threshold and best_loc is not None:
            # Calculate center in original frame coordinates
            template_height, template_width = self.template_sizes[best_template_idx]
            center_x = int((best_loc[0] + template_width/2) * best_scale)
            center_y = int((best_loc[1] + template_height/2) * best_scale)
            return (center_x, center_y, best_val)
        return None

def main():
    # Initialize tracker with template images
    template_paths = ["/home/d34/Downloads/tracker_test_v2/airplane_edited/airplane1.png",
                     "/home/d34/Downloads/tracker_test_v2/airplane_edited/airplane2.png",
                     "/home/d34/Downloads/tracker_test_v2/airplane_edited/airplane3.png",
                     "/home/d34/Downloads/tracker_test_v2/airplane_edited/airplane4.png"
                     ]  # Add your template image path
    tracker = SimpleAirplaneTracker(template_paths)

    # Start video capture
    cap = cv2.VideoCapture(0)  # Use 0 for webcam or video file path

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Find airplane
        result = tracker.find_airplane(frame)

        # Draw results
        if result:
            x, y, conf = result
            cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
            cv2.putText(frame, f"Match: ({x}, {y}) Conf: {conf:.2f}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Draw bounding box (optional)
            if conf > 0.8:  # Only draw if confidence is high
                template_height, template_width = tracker.template_sizes[0]
                top_left = (int(x - template_width/2), int(y - template_height/2))
                bottom_right = (int(x + template_width/2), int(y + template_height/2))
                cv2.rectangle(frame, top_left, bottom_right, (0, 255, 0), 2)

        cv2.imshow('Tracking', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Helper function to test template sizes
def print_template_info(template_path):
    """
    Print information about template and suggested sizes
    """
    template = cv2.imread(template_path)
    if template is not None:
        height, width = template.shape[:2]
        print(f"Template size: {width}x{height}")
        print("\nSuggested template sizes:")
        for scale in [0.25, 0.5, 0.75, 1.0]:
            new_width = int(width * scale)
            new_height = int(height * scale)
            print(f"Scale {scale}: {new_width}x{new_height}")

if __name__ == "__main__":
    # Uncomment to check template sizes
    # print_template_info("airplane_template.jpg")
    main()
