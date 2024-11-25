import tkinter as tk
from tkinter import ttk
import threading
import time

class CarDataVisualizer:
    def __init__(self, root):
        self.root = root
        self.root.title("Car Data Visualization")
        self.root.geometry("800x600")
        
        # Variables for data display
        self.state_var = tk.StringVar(value="Not Started")
        self.velocity_x = tk.StringVar(value="0")
        self.velocity_y = tk.StringVar(value="0")
        self.angular_velocity = tk.StringVar(value="0")
        self.distance = tk.StringVar(value="0")
        self.angle_z = tk.StringVar(value="0")
        self.op_mode = tk.StringVar(value="0")
        self.is_holding = tk.StringVar(value="False")
        self.follow_state = tk.StringVar(value="0")
        
        self.setup_gui()
        self.running = False
        
    def setup_gui(self):
        # Create main frames
        control_frame = ttk.Frame(self.root, padding="10")
        control_frame.pack(fill=tk.X)
        
        data_frame = ttk.Frame(self.root, padding="10")
        data_frame.pack(fill=tk.BOTH, expand=True)
        
        # Control buttons
        self.start_button = ttk.Button(control_frame, text="Start", command=self.start_program)
        self.start_button.pack(side=tk.LEFT, padx=5)
        
        self.stop_button = ttk.Button(control_frame, text="Stop", command=self.stop_program)
        self.stop_button.pack(side=tk.LEFT, padx=5)
        
        # State display
        state_frame = ttk.LabelFrame(data_frame, text="Current State", padding="10")
        state_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(state_frame, textvariable=self.state_var, font=('Arial', 16, 'bold')).pack()
        
        # Data display
        info_frame = ttk.LabelFrame(data_frame, text="Vehicle Data", padding="10")
        info_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Create grid of labels
        labels = [
            ("Velocity X (mm/s):", self.velocity_x),
            ("Velocity Y (mm/s):", self.velocity_y),
            ("Angular Velocity (deg/s):", self.angular_velocity),
            ("Distance (mm):", self.distance),
            ("Angle Z (deg):", self.angle_z),
            ("Operation Mode:", self.op_mode),
            ("Holding Status:", self.is_holding),
            ("Follow State:", self.follow_state)
        ]
        
        for i, (label_text, var) in enumerate(labels):
            ttk.Label(info_frame, text=label_text).grid(row=i, column=0, sticky='e', padx=5, pady=3)
            ttk.Label(info_frame, textvariable=var).grid(row=i, column=1, sticky='w', padx=5, pady=3)
    
    def start_program(self):
        if not self.running:
            self.running = True
            self.start_button.state(['disabled'])
            self.program_thread = threading.Thread(target=self.run_main_program)
            self.program_thread.start()
            
    def stop_program(self):
        self.running = False
        self.start_button.state(['!disabled'])
        self.state_var.set("Stopped")
    
    def update_data(self, comm):
        """Update the display with current data"""
        if not self.running:
            return
            
        self.velocity_x.set(f"{comm.chassis_cmd.vx}")
        self.velocity_y.set(f"{comm.chassis_cmd.vy}")
        self.angular_velocity.set(f"{comm.chassis_cmd.wz}")
        self.distance.set(f"{comm.sensor_info.distance_mm}")
        self.angle_z.set(f"{comm.sensor_info.angleZ}")
        self.op_mode.set(f"{comm.chassis_cmd.op_mode}")
        self.is_holding.set(f"{comm.sensor_info.is_holding}")
        self.follow_state.set(f"{comm.chassis_info.follow_state}")
        
        self.root.update()
    
    def run_main_program(self):
        try:
            # Import your required modules here
            from communication import Communication
            from detectnet import detectPlane
            from light_seeker import MecanumLightSeeker
            
            ports = Communication.detect_ports()
            port = ports["CP2102 USB to UART Bridge Controller - CP2102 USB to UART Bridge Controller"]
            
            comm = Communication(port)
            comm.start()
            
            detect_plane = detectPlane()
            detect_light = MecanumLightSeeker()
            
            state = "IDLE"
            self.state_var.set(state)
            
            while self.running:
                # Your existing main program logic here
                # Update state and data periodically
                self.state_var.set(state)
                self.update_data(comm)
                
                # Rest of your main program logic...
                time.sleep(0.1)
                
        except Exception as e:
            self.state_var.set(f"Error: {str(e)}")
        finally:
            # Cleanup
            if 'detect_plane' in locals():
                detect_plane.stop()
            if 'detect_light' in locals():
                detect_light.stop()
            if 'comm' in locals():
                comm.stop()

def start_visualization():
    root = tk.Tk()
    app = CarDataVisualizer(root)
    root.mainloop()

if __name__ == "__main__":
    start_visualization()

