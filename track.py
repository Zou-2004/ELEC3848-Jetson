import cv2
import numpy as np

class AirplaneTracker:
    def __init__(self, template_paths,simple_confidence=0.8,SIFT_confidence=0.7):
        """
        Initialize tracker with multiple template images of the airplane
        template_paths: list of paths to template images
        """
        self.templates = [cv2.imread(path) for path in template_paths]
        self.sift = cv2.SIFT_create()
        self.simple_confidence=simple_confidence
        self.SIFT_confidence=SIFT_confidence
        
        # Compute keypoints and descriptors for all templates
        self.template_keypoints = []
        self.template_descriptors = []
        for template in self.templates:
            keypoints, descriptors = self.sift.detectAndCompute(template, None)
            self.template_keypoints.append(keypoints)
            self.template_descriptors.append(descriptors)
        
        # FLANN matcher parameters
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        
        # Minimum number of good matches required
        self.MIN_MATCH_COUNT = 10

    def find_airplane_template(self, frame):
        """
        Simple template matching approach
        Returns center coordinates if found, None if not found
        """
        best_loc = None
        best_val = -1
        
        for template in self.templates:
            # Convert both to grayscale
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray_template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
            
            # Template matching
            result = cv2.matchTemplate(gray_frame, gray_template, cv2.TM_CCOEFF_NORMED)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
            
            if max_val > best_val:
                best_val = max_val
                best_loc = max_loc
                h, w = template.shape[:2]
        
        # If confidence is high enough
        if best_val > self.simple_confidence:
            center_x = best_loc[0] + w//2
            center_y = best_loc[1] + h//2
            return (center_x, center_y)
        return None

    def find_airplane_features(self, frame):
        """
        Feature matching approach using SIFT
        Returns center coordinates if found, None if not found
        """
        # Convert frame to grayscale
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Find keypoints and descriptors in frame
        frame_keypoints, frame_descriptors = self.sift.detectAndCompute(gray_frame, None)
        
        if frame_descriptors is None:
            return None
        
        best_matches_count = 0
        best_center = None
        
        # Try matching with each template
        for template_idx, template_descriptor in enumerate(self.template_descriptors):
            # Match descriptors
            matches = self.flann.knnMatch(template_descriptor, frame_descriptors, k=2)
            
            # Apply ratio test
            good_matches = []
            for m, n in matches:
                if m.distance < self.SIFT_confidence * n.distance:
                    good_matches.append(m)
            
            if len(good_matches) > best_matches_count:
                if len(good_matches) > self.MIN_MATCH_COUNT:
                    template_pts = np.float32([self.template_keypoints[template_idx][m.queryIdx].pt 
                                            for m in good_matches]).reshape(-1, 1, 2)
                    frame_pts = np.float32([frame_keypoints[m.trainIdx].pt 
                                          for m in good_matches]).reshape(-1, 1, 2)
                    
                    # Calculate homography
                    H, mask = cv2.findHomography(template_pts, frame_pts, cv2.RANSAC, 5.0)
                    
                    if H is not None:
                        # Calculate center of template
                        h, w = self.templates[template_idx].shape[:2]
                        template_center = np.float32([[w/2, h/2]]).reshape(-1, 1, 2)
                        
                        # Transform template center to frame coordinates
                        transformed_center = cv2.perspectiveTransform(template_center, H)
                        
                        best_matches_count = len(good_matches)
                        best_center = (int(transformed_center[0][0][0]), 
                                     int(transformed_center[0][0][1]))
        
        return best_center

# Example usage
if __name__ == "__main__":
    # Initialize tracker with template images
    template_paths = ["C:/Users/DELL/Desktop/3848 project/tracker/me1.png","C:/Users/DELL/Desktop/3848 project/tracker/me2.png","C:/Users/DELL/Desktop/3848 project/tracker/me3.png"]
    tracker = AirplaneTracker(template_paths,0.9,0.9)
    
    # Open video capture
    cap = cv2.VideoCapture(0)  # Use 0 for webcam or video file path
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # Try both methods
        center_template = tracker.find_airplane_template(frame)
        center_features = tracker.find_airplane_features(frame)
        
        # Draw results
        if center_template:
            cv2.circle(frame, center_template, 5, (0, 255, 0), -1)
            cv2.putText(frame, f"Template: {center_template}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
        if center_features:
            cv2.circle(frame, center_features, 5, (0, 0, 255), -1)
            cv2.putText(frame, f"Features: {center_features}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        cv2.imshow('Tracking', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()