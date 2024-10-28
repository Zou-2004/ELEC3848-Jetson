import cv2
import numpy as np
import serial
import time
import communication

comm = communication.Communication()

class MecanumLightSeeker:
    def __init__(self):
        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        
        # Initialize Arduino communication
       # self.arduino = serial.Serial('/dev/ttyS0', 115200, timeout=1)
        time.sleep(2)  # Wait for Arduino to reset
        
        # Light detection parameters
        self.light_threshold = 200
        self.min_light_area = 100
        self.max_light_area = 10000
        
        # Target parameters
        self.target_distance = 8  # cm
        
        # PID parameters for each movement component
        self.kp_forward = 2.0    # Forward/backward
        self.kp_lateral = 1.5    # Left/right
        self.kp_rotation = 0.5   # Rotation
        
        # Movement limits
        self.max_speed = 255
        self.min_speed = 50

    def cleanup(self):
        """Clean up resources"""
        self.cap.release()
        cv2.destroyAllWindows()
        self.arduino.close()

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

    def get_distance(self):
        """
        Get distance reading from Arduino
        Returns: distance in cm or None if no valid reading
        """
        try:
            if self.arduino.in_waiting:
                data = self.arduino.readline().decode().strip()
                if data.startswith('D:'):
                    return int(data[2:])
        except:
            pass
        return None

    def calculate_mecanum_wheel_speeds(self, vx, vy, omega):
        """
        Calculate individual wheel speeds for mecanum drive
        vx: forward velocity (-1 to 1)
        vy: lateral velocity (-1 to 1)
        omega: angular velocity (-1 to 1)
        
        Wheel arrangement:
        FL  FR
        BL  BR
        """
        # Mecanum wheel mixing algorithm
        front_left = vx - vy - omega
        front_right = vx + vy + omega
        back_left = vx + vy - omega
        back_right = vx - vy + omega
        
        # Normalize speeds
        max_speed = max(abs(front_left), abs(front_right), 
                       abs(back_left), abs(back_right), 1)
        
        front_left = (front_left / max_speed) * self.max_speed
        front_right = (front_right / max_speed) * self.max_speed
        back_left = (back_left / max_speed) * self.max_speed
        back_right = (back_right / max_speed) * self.max_speed
        
        return (int(front_left), int(front_right), 
                int(back_left), int(back_right))

    def calculate_movement_vectors(self, light_pos, frame_width, frame_height, 
                                 light_area, current_distance):
        if not light_pos:
            return 0, 0, 0.3  # Rotate to search for light
        
        # Calculate forward velocity (based on distance)
        if current_distance is not None:
            distance_error = current_distance - self.target_distance
            vx = self.kp_forward * distance_error / self.max_speed
        else:
            # Use light area as approximate distance
            estimated_distance = self.max_light_area / max(light_area, 1)
            vx = 0.5 * (estimated_distance / self.max_light_area)
        
        # Calculate lateral velocity (based on horizontal position)
        center_x_error = light_pos[0] - frame_width // 2
        vy = -self.kp_lateral * center_x_error / (frame_width // 2)
        
        # Calculate rotational velocity (to keep light centered)
        omega = -self.kp_rotation * center_x_error / (frame_width // 2)
        
        # Clamp values between -1 and 1
        vx = max(min(vx, 1), -1)
        vy = max(min(vy, 1), -1)
        omega = max(min(omega, 1), -1)
        
        return vx, vy, omega

    # def send_mecanum_command(self, fl, fr, bl, br):
    #     """
    #     Send command to Arduino with four wheel speeds
    #     Format: "FLxxxFRxxxBLxxxBRxxx\n"
    #     """
    #     command = "FL{:03d}FR{:03d}BL{:03d}BR{:03d}\n".format(int(fl),int(fr),int(bl),int(br))
    #     self.arduino.write(command.encode())

    def send_chassis_command(self, vx, vy, wz):
        pass

    def run(self):
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    continue

                height, width = frame.shape[:2]
                
                # Detect light
                light_pos, light_area = self.detect_light(frame)
                
                # Get current distance
                current_distance = self.get_distance()
                
                if light_pos:
                    # Calculate movement vectors
                    vx, vy, omega = self.calculate_movement_vectors(
                        light_pos, width, height, light_area, current_distance)
                    
                    # Convert to wheel speeds
                    fl, fr, bl, br = self.calculate_mecanum_wheel_speeds(vx, vy, omega)
                    
                    # Send commands to Arduino
                    self.send_mecanum_command(fl, fr, bl, br)
                    
                    # Visual feedback
                    cv2.circle(frame, light_pos, 10, (0, 255, 0), -1)
                    cv2.putText(frame, f"Area: {light_area}", (10, 30),
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f"vx: {vx:.2f} vy: {vy:.2f} w: {omega:.2f}", 
                              (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                else:
                    # Search mode - rotate in place
                    fl, fr, bl, br = self.calculate_mecanum_wheel_speeds(0, 0, 0.3)
                    self.send_mecanum_command(fl, fr, bl, br)
                
                # Display distance if available
                if current_distance is not None:
                    cv2.putText(frame, f"Distance: {current_distance}cm", (10, 90),
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                cv2.imshow('Light Seeking', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.cleanup()

def main():
    light_seeker = MecanumLightSeeker()
    light_seeker.run()

if __name__ == "__main__":
    main()
