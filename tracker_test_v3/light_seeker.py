import cv2
import time 
import threading

TOF_OFFSET = 20
kp_forward = 0.07    # Forward/backward
kp_lateral = 1.5    # Left/right
kp_rotation = 1.5 
target_distance = 130 + TOF_OFFSET


class MecanumLightSeeker():
    cap = None
    movement_vector_list = None
    
    current_distance = 1e9
    angleZ = 0
    
    # print_time = 0
    
    def __init__(self):
        # Initialize camera
        
        time.sleep(2)  
        self.running=False
        # Light detection parameters
        self.light_threshold = 200
        self.min_light_area = 100
        self.max_light_area = 10000

        # Target parameters
        # self.target_distance = 130 + TOF_OFFSET  # cm
        
        # # PID parameters for each movement component
        # self.kp_forward = 0.07    # Forward/backward
        # self.kp_lateral = 1.5    # Left/right
        # self.kp_rotation = 1.5   # Rotation
        # self.movement_vector_list = [0, 0, 0]

        # time.sleep(1)


    def start(self):
        print("Starting light seeker...")
        self.running = True
        self.cap = cv2.VideoCapture(0)

        self.t=threading.Thread(target=self.thread_loop)
        self.t.start()
        print("light seeker started.")


    def stop(self):
        if self.running:
            self.running = False
            self.t.join()

            self.cap.release()
            self.cap = None
            cv2.destroyAllWindows()


    # @property
    def get_vector(self):
        return self.movement_vector_list

    def set_distance(self, distance):
        self.current_distance = distance

    def set_angleZ(self, angleZ):
        self.angleZ = angleZ

    def thread_loop(self):
        while self.running:
            _, frame = self.cap.read()
            # print(frame)
            if frame is not None:
                pos, area = self.detect_light(frame)
                # print(f"pos = {pos}, area = {area}")l
                self.movement_vector_list = self.calculate_movement_vectors(pos, frame.shape[1], self.current_distance, self.angleZ)
            else:
                self.movement_vector_list = 0, 0, 0

                print("FRAME IS NONE!!!")
            time.sleep(0.04)
        

    def detect_light(self, frame):
        if frame is None:
            return None, None
        """
        Detect the brightest spot in the frame
        Returns: (x,y) coordinates of brightest spot and its area
        """
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Threshold the image
        _, thresh = cv2.threshold(gray, self.light_threshold, 255, cv2.THRESH_BINARY)
        
        # Find contours
        _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, 
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


    @staticmethod
    def calculate_movement_vectors(light_pos,frame_width,current_distance,angleZ):
        # params
        
        # movement_vector_list = [0, 0, 0]
        
        vx = 0
        vy = 0
        omega = 0
    
        if light_pos != None:
            center_x_error = light_pos[0] - frame_width // 2
        else:
            center_x_error = 0

            if (current_distance is not None and current_distance > 20):
                omega = -kp_rotation * center_x_error / (frame_width // 2)
                vy = 0
            elif current_distance is not None and current_distance > 8:
                omega = 0
                vy = -kp_lateral * center_x_error / (frame_width // 2)
            else:
                omega = 0
                vy = 0
        

        distance_error = None
        if (current_distance != None):
            distance_error = current_distance - target_distance
        if (distance_error == None):
            vx, vy, omega = 0, 0, 0
        elif (distance_error > 40):
            vx = kp_forward * distance_error 
            vy = 0
            if (light_pos):
                omega = - kp_rotation * center_x_error / (frame_width // 2)
            else:
                omega = 0
        # elif (distance_error > 8):
        elif (distance_error > 0):
            if (abs(angleZ) > 10):
                vx, vy = 0, 0
                omega = -angleZ / 45.0

            elif (abs(center_x_error / (frame_width // 2)) > 0.1):
                vx, omega = 0, 0
                vy = -kp_lateral * center_x_error / (frame_width // 2)
            else:
                vx = kp_forward * distance_error 
                vy = - kp_lateral * center_x_error / (frame_width // 2)
                omega = 0
        # elif (distance_error > 0):
        #     vx = self.kp_forward * distance_error 
        #     vy, omega = 0, 0
        else:
            vx, vy, omega = 0, 0, 0
            yahoo_flag = True
        
        # Clamp values between -1 and 1
        vx = max(min(vx, 1), -1)
        vy = max(min(vy, 1), -1)
        omega = max(min(omega, 1), -1)
        # movement_vector_list[0],movement_vector_list[1],movement_vector_list[2]=vx,vy,omega
        
        return vx,vy,omega


if __name__ == "__main__":
    msl = MecanumLightSeeker()
    msl.start()
    for i in range(50):
        print(msl.get_vector())
        time.sleep(0.2)
    msl.stop()