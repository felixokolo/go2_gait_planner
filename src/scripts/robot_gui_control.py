import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32
from go2_scratch.msg import ParamsSet
from go2_scratch.msg import MoveLeg
from go2_scratch.msg import JointsSet
from go2_scratch.msg import GaitParam
from unitree_go.msg import LowState

import os
import pkg_resources
import threading

class RobotControlGUI:
    def __init__(self, master, node):
        self.master = master
        master.title("Robot Control GUI")
        self.node = node  # Pass ROS 2 Node instance to the GUI class

        # Get the screen width and height
        screen_width = self.master.winfo_screenwidth()
        screen_height = self.master.winfo_screenheight()
        
        # Calculate the size of the window (90% of the screen)
        window_width = int(screen_width * 0.9)
        window_height = int(screen_height * 0.9)
    
        # Center the window on the screen
        position_right = int(screen_width / 2 - window_width / 2)
        position_down = int(screen_height / 2 - window_height / 2)

        self.master.geometry(f"{window_width}x{window_height}+{position_right}+{position_down}")

        self.publisher = self.node.create_publisher(MoveLeg, '/go2_scratch/move', 10)
        self.publisher1 = self.node.create_publisher(JointsSet, '/go2_scratch/joints', 10)
        self.publisher2 = self.node.create_publisher(ParamsSet, '/go2_scratch/params', 10)
        self.publisher3 = self.node.create_publisher(Int32, '/go2_scratch/stand_sit', 10)
        self.publisher4 = self.node.create_publisher(GaitParam, '/go2_scratch/gait_msg', 10)
        self.active_leg = None
        self.joint_angles_FL=[0,0,0]
        self.joint_angles_FR=[0,0,0]
        self.joint_angles_RL=[0,0,0]
        self.joint_angles_RR=[0,0,0]

        self.subscriber = self.node.create_subscription(            
            LowState,  # Correct message type for /lowstate
            '/lowstate',
            self.low_state_callback,
            10
        )

        print("Subscriber created for /lowstate")
        robot_back = 'blanched almond'
        # Add a frame inside a canvas with a scrollbar
        self.canvas = tk.Canvas(master, bg=robot_back)
        self.scrollbar = tk.Scrollbar(master, orient="vertical", command=self.canvas.yview)
        self.h_scrollbar = tk.Scrollbar(master, orient="horizontal", command=self.canvas.xview)
        self.scrollable_frame = tk.Frame(self.canvas, bg=robot_back)

        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.canvas.configure(
                scrollregion=self.canvas.bbox("all")
            )
        )

        self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set, xscrollcommand=self.h_scrollbar.set)

        # Pack the canvas and scrollbars
        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")
        self.h_scrollbar.pack(side="bottom", fill="x")

        package_dir = pkg_resources.resource_filename(__name__, '')
        image_path = os.path.join(package_dir, "Quadruped_Robot.png")
        # Title and Logo
        
        self.title_frame = tk.Frame(self.scrollable_frame,width=window_width, bg=robot_back)
        self.title_frame.pack(fill=tk.BOTH, expand=True)
        self.logo = tk.PhotoImage(file=image_path)  
        self.logo_label = tk.Label(self.title_frame, image=self.logo, bg=robot_back)
        self.logo_label.pack(pady=5)
        self.title_label = tk.Label(self.title_frame, text="Robot Control GUI", bg=robot_back, font=("Arial", 16))
        self.title_label.pack()

        # Use grid layout for the main sections
        self.main_frame = tk.Frame(self.scrollable_frame)
        self.main_frame.pack(fill=tk.BOTH, expand=True)

        # Leg Control Section (Left)
        self.leg_control_frame = tk.Frame(self.main_frame)
        self.leg_control_frame.grid(row=0, column=0, padx=10, pady=10, sticky='nw')

        self.leg_control_label = tk.Label(self.leg_control_frame, text="Leg Control", font=("Arial", 12, "bold"), bg="indian red")
        self.leg_control_label.grid(row=0, column=0, columnspan=5, pady=(0, 5))

        self.active_leg_label = tk.Label(self.leg_control_frame, text="Active Leg")
        self.active_leg_label.grid(row=1, column=0, sticky='w')
        self.fl_leg_var = tk.BooleanVar()
        self.fr_leg_var = tk.BooleanVar()
        self.rl_leg_var = tk.BooleanVar()
        self.rr_leg_var = tk.BooleanVar()

        self.fr_leg = tk.Checkbutton(self.leg_control_frame, text="FR", variable=self.fr_leg_var)
        self.fr_leg.grid(row=1, column=1)
        self.fl_leg = tk.Checkbutton(self.leg_control_frame, text="FL", variable=self.fl_leg_var)
        self.fl_leg.grid(row=1, column=2)
        self.rr_leg = tk.Checkbutton(self.leg_control_frame, text="RR", variable=self.rr_leg_var)
        self.rr_leg.grid(row=1, column=3)
        self.rl_leg = tk.Checkbutton(self.leg_control_frame, text="RL", variable=self.rl_leg_var)
        self.rl_leg.grid(row=1, column=4)

        # Joint Parameters Section (Below Leg Control)
        self.joint_parameters_frame = tk.Frame(self.main_frame)
        self.joint_parameters_frame.grid(row=1, column=0, padx=10, pady=10, sticky='nw')
        self.joint_parameters_label = tk.Label(self.joint_parameters_frame, text="Joint Parameters", font=("Arial", 12, "bold"))
        self.joint_parameters_label.grid(row=0, column=0, columnspan=5, pady=(0, 10))
        self.create_joint_parameters_section(self.joint_parameters_frame)

        #leg control frame
        self.leg_control_frame = tk.Frame(self.main_frame)
        self.leg_control_frame.grid(row=2, column=0, padx=10, pady=10, sticky='nw')

        self.leg_control_label = tk.Label(self.leg_control_frame, text="Leg Control", font=("Arial", 12, "bold"), bg="indian red")
        self.leg_control_label.grid(row=0, column=0, columnspan=5, pady=(0, 10))

        self.active_leg_label = tk.Label(self.leg_control_frame, text="Active Leg")
        self.active_leg_label.grid(row=1, column=0, sticky='w', padx=5, pady=5)

        self.active_leg_var = tk.StringVar(value="FR")

        self.fr_leg1 = tk.Radiobutton(self.leg_control_frame, text="FR", variable=self.active_leg_var, value="FR", command=self.on_leg_select)
        self.fr_leg1.grid(row=1, column=1, padx=5, pady=5)

        self.fl_leg1 = tk.Radiobutton(self.leg_control_frame, text="FL", variable=self.active_leg_var, value="FL", command=self.on_leg_select)
        self.fl_leg1.grid(row=1, column=2, padx=5, pady=5)

        self.rr_leg1 = tk.Radiobutton(self.leg_control_frame, text="RR", variable=self.active_leg_var, value="RR", command=self.on_leg_select)
        self.rr_leg1.grid(row=1, column=3, padx=5, pady=5)

        self.rl_leg1 = tk.Radiobutton(self.leg_control_frame, text="RL", variable=self.active_leg_var, value="RL", command=self.on_leg_select)
        self.rl_leg1.grid(row=1, column=4, padx=5, pady=5)


        # Joint Motion Section (Below Leg Control)
        self.joint_motion_frame = tk.Frame(self.main_frame)
        self.joint_motion_frame.grid(row=3, column=0, padx=10, pady=10, sticky='nw')
        self.joint_motion_label = tk.Label(self.joint_motion_frame, text="Joint Motion", font=("Arial", 12, "bold"))
        self.joint_motion_label.grid(row=0, column=0, columnspan=5, pady=(0, 10))
        self.create_joint_motion_section(self.joint_motion_frame)

        # Cartesian Motion Section (Below Joint Motion)
        self.cartesian_motion_frame = tk.Frame(self.main_frame)
        self.cartesian_motion_frame.grid(row=4, column=0, padx=10, pady=10, sticky='nw')
        self.cartesian_motion_label = tk.Label(self.cartesian_motion_frame, text="Cartesian Motion", font=("Arial", 12, "bold"))
        self.cartesian_motion_label.grid(row=0, column=0, columnspan=5, pady=(0, 10))
        self.create_cartesian_motion_section(self.cartesian_motion_frame)

        # Body Control Section (Center)
        self.body_control_frame = tk.Frame(self.main_frame)
        self.body_control_frame.grid(row=0, column=1, rowspan=4, padx=10, pady=10, sticky='n')

        self.body_control_label = tk.Label(self.body_control_frame, text="Body Control", font=("Arial", 12, "bold"), bg='lightblue')
        self.body_control_label.pack(pady=(0, 15))

        self.stand_up_button = tk.Button(self.body_control_frame, text="Stand Up", width=10, height=1, command=self.stand_up)
        self.stand_up_button.pack(pady=10)

        self.sit_down_button = tk.Button(self.body_control_frame, text="Sit Down", width=10, height=1, command=self.sit_down)
        self.sit_down_button.pack(pady=10)



        # Gait Control Section (Right)
        self.gait_control_frame = tk.Frame(self.main_frame)
        self.gait_control_frame.grid(row=2, column=1, rowspan=4, padx=10, pady=10, sticky='n')

        self.gait_control_label = tk.Label(self.gait_control_frame, text="Gait Control", font=("Arial", 12, "bold"), bg='lightgreen')
        self.gait_control_label.pack(pady=(0, 15))

        self.gait_var = tk.StringVar(value="Trot")  # Default value

        self.trot_radio = tk.Radiobutton(self.gait_control_frame, text="Trot", variable=self.gait_var, value="Trot")
        self.trot_radio.pack(anchor="w", padx=5, pady=5)

        self.sneak_radio = tk.Radiobutton(self.gait_control_frame, text="Sneak", variable=self.gait_var, value="Sneak")
        self.sneak_radio.pack(anchor="w", padx=5, pady=5)

        # Initialize dictionary to store references to entry widgets
        self.entries = {}

        # Define parameters and default values
        self.parameters = [
            ("Velocity:", "0.0"),
            ("Nominal Height:", "0.0"),
            ("Stance Duration:", "0.0"),
            ("Stance Depth:", "0.0"),
            ("Swing Height:", "0.0")
        ]

        # Calculate the maximum width for the labels
        max_label_width = max(len(p[0]) for p in self.parameters) * 10  # Adjust multiplier for desired width

        # Create labels, entries, and buttons
        for param, default_value in self.parameters:
            frame = tk.Frame(self.gait_control_frame)
            frame.pack(fill="x", pady=5)

            # Create and pack the label
            label = tk.Label(frame, text=param, anchor="w", width=(max_label_width // 10))  # Use max width
            label.pack(side="left", padx=5)

            # Create and pack the entry widget
            entry = tk.Entry(frame, width=10)
            entry.insert(0, default_value)
            entry.pack(side="left", padx=5)

            # Create and pack the increment button
            increment_button = tk.Button(frame, text="+", width=2,height=1, command=lambda e=entry: self.adjust_value(e, 1))
            increment_button.pack(side="left", padx=5)

            # Create and pack the decrement button
            decrement_button = tk.Button(frame, text="-", width=2, height=1, command=lambda e=entry: self.adjust_value(e, -1))
            decrement_button.pack(side="left", padx=5)

            # Store reference to the entry widget
            self.entries[param] = entry


        # Gaming-style directional buttons for gait control
        self.gait_direction_frame = tk.Frame(self.gait_control_frame)
        self.gait_direction_frame.pack(pady=15)

        self.gait_up_button = tk.Button(self.gait_direction_frame, text="Forward", width=8, height=2, command=lambda: self.gait_move(1))
        self.gait_up_button.grid(row=0, column=1, padx=5, pady=5)

        self.gait_left_button = tk.Button(self.gait_direction_frame, text="Left", width=8, height=2, command=lambda: self.gait_move(3))
        self.gait_left_button.grid(row=1, column=0, padx=5, pady=5)

       # Stop button in the center
        self.gait_stop_button = tk.Button(self.gait_direction_frame, text="Stop", width=8, height=2, command=lambda: self.gait_move(0))
        self.gait_stop_button.grid(row=1, column=1, padx=5, pady=5)

        self.gait_right_button = tk.Button(self.gait_direction_frame, text="Right", width=8, height=2, command=lambda: self.gait_move(4))
        self.gait_right_button.grid(row=1, column=2, padx=5, pady=5)

        self.gait_down_button = tk.Button(self.gait_direction_frame, text="Backward", width=8, height=2, command=lambda: self.gait_move(2))
        self.gait_down_button.grid(row=2, column=1, padx=5, pady=5)

        self.gait_jump_button = tk.Button(self.gait_direction_frame, text="Jump", width=8, height=2, command=lambda: self.gait_move(5))
        self.gait_jump_button.grid(row=3, column=1, padx=5, pady=5)
     

    
        # Footer Section
        self.footer_frame = tk.Frame(self.scrollable_frame, bg='black')
        self.footer_frame.pack(side=tk.BOTTOM, fill=tk.X)
        self.team_label = tk.Label(self.footer_frame, text="Our Team", fg='white', bg='black')
        self.team_label.pack(side=tk.LEFT, padx=10)
        self.team_members = tk.Label(self.footer_frame, text="Chukwudi, Priyanka, Shreya, Divya", fg='white', bg='black', justify=tk.LEFT)
        self.team_members.pack(side=tk.LEFT, padx=10)
        self.guide_label = tk.Label(self.footer_frame, text="Project Guide", fg='white', bg='black')
        self.guide_label.pack(side=tk.LEFT, padx=10)
        self.guide_info = tk.Label(self.footer_frame, text="Name: Rogalski", fg='white', bg='black', justify=tk.LEFT)
        self.guide_info.pack(side=tk.LEFT, padx=10)

        #self.publisher = rospy.Publisher('robot_command', String, queue_size=10)
        #rospy.init_node('robot_control_gui', anonymous=True)
        self.message_label = tk.Label(self.scrollable_frame, text="", fg="green")
        self.message_label.pack()

    def create_joint_parameters_section(self, frame):
        parts = ["Hip", "Thigh", "Calf"]
       # self.joint_velocity_vars = [tk.DoubleVar() for _ in parts]
       #  self.joint_torque_vars = [tk.DoubleVar() for _ in parts]
       # self.kp_vars = [tk.DoubleVar(value=[70, 150, 150]) for _ in parts]
        self.kp_vars = [
        tk.DoubleVar(value=70),  
        tk.DoubleVar(value=150),  
        tk.DoubleVar(value=150)    
        ]
        self.kd_vars = [tk.DoubleVar(value=[3.5, 3.5, 3.5]) for _ in parts]
        
        for i, part in enumerate(parts):
            tk.Label(frame, text=part).grid(row=1, column=i + 1)
        labels = ["Kp", "Kd"]
        variables = [self.kp_vars, self.kd_vars]
        increments = [0.1, 0.1]
        ranges = [ (0, 200), (0, 8)]
        
        for i, (label, (r_min, r_max), inc, var_set) in enumerate(zip(labels, ranges, increments, variables)):
            tk.Label(frame, text=label).grid(row=i + 2, column=0, sticky='w')
            for j, var in enumerate(var_set):
                spinbox = tk.Spinbox(frame, from_=r_min, to=r_max, increment=inc, textvariable=var)
                spinbox.grid(row=i + 2, column=j + 1)

        tk.Button(frame, text="Send", command=self.send_joint_parameters).grid(row=6, column=3, padx=5)

    def create_joint_motion_section(self, frame):
        parts = ["Hip", "Thigh", "Calf"]
        self.joint_angle_vars = [tk.DoubleVar() for _ in parts]
        self.joint_velocity_vars = [tk.DoubleVar() for _ in parts]
        self.joint_torque_vars = [tk.DoubleVar() for _ in parts]

        for i, part in enumerate(parts):
            tk.Label(frame, text=part).grid(row=1, column=i + 1)
        labels = ["Joint Positions (rad)", "Joint Velocity (rad/sec)", "Joint Torques (Nm)"]  
        variables = [ self.joint_angle_vars, self.joint_velocity_vars, self.joint_torque_vars]
        increments = [0.1, 0.1, 0.1]

        ranges = [(-3.14, 3.14), (-20,20), (-15 , 15)]

        for i, (label, (r_min, r_max), inc, var_set) in enumerate(zip(labels, ranges, increments, variables)):
            tk.Label(frame, text=label).grid(row=i + 2, column=0, sticky='w')
            for j, var in enumerate(var_set):
                spinbox = tk.Spinbox(frame, from_=r_min, to=r_max, increment=inc, textvariable=var)
                spinbox.grid(row=i + 2, column=j + 1) 
        
        tk.Button(frame, text="Send", command=self.send_jointSet_parameters).grid(row=5, column=3, padx=5)

    def add_deg_suffix(self, spinbox):
        value = spinbox.get()
        if "deg" not in value:
            spinbox.delete(0, "end")
            spinbox.insert(0, f"{value}deg")

    def create_cartesian_motion_section(self, frame):
        tk.Label(frame, text="Type").grid(row=1, column=0, sticky='w')
        self.motion_type_var = tk.IntVar()
        tk.Radiobutton(frame, text="Straight", variable=self.motion_type_var, value=0).grid(row=1, column=1, sticky='w')
        tk.Radiobutton(frame, text="Swing", variable=self.motion_type_var, value=1).grid(row=1, column=2, sticky='w')
        
        tk.Label(frame, text="Swing Height (m)").grid(row=2, column=0, sticky='w')
        self.swing_height_vars = tk.DoubleVar()

        tk.Label(frame, text="Direction (m)").grid(row=3, column=0, sticky='w')
        self.swing_directions_vars = [tk.DoubleVar() for _ in range(3)]

        tk.Label(frame, text="Duration (ms)").grid(row=4, column=0, sticky='w')
        self.swing_time_vars = tk.IntVar(value=1000)
        
        tk.Label(frame, text=f"{chr(88+1)}").grid(row=2, column=1)
        tk.Spinbox(frame, from_=-0.5, to=0.5, increment=0.01, textvariable=self.swing_height_vars).grid(row=2, column=1, padx=5)

        for i, var in enumerate(self.swing_directions_vars):
            tk.Label(frame, text=f"{chr(88+i)}").grid(row=3, column=i + 1)
            tk.Spinbox(frame, from_=-0.5, to=0.5, increment=0.01, textvariable=var).grid(row=3, column=i + 1)

        tk.Label(frame, text=f"{chr(88+1)}").grid(row=4, column=1)
        tk.Spinbox(frame, from_=0, to=10000, increment=5, textvariable=self.swing_time_vars).grid(row=4, column=1, padx=5)
        
        tk.Button(frame, text="Send", command=self.send_cartesian_motion).grid(row=4, column=3, padx=5)
        
     

    def on_leg_select(self):
    # Dictionary mapping leg names to their joint angles
     leg_mapping = {
        "FL": self.joint_angles_FL,
        "FR": self.joint_angles_FR,
        "RL": self.joint_angles_RL,
        "RR": self.joint_angles_RR
     }
    
     selected_leg = self.active_leg_var.get()
      # Default to [0.0, 0.0, 0.0] if leg is not found
     angles = [round(angle, 2) for angle in leg_mapping.get(selected_leg, [0.0, 0.0, 0.0])]

     #angles = leg_mapping.get(selected_leg, [0.0, 0.0, 0.0])



    # Update joint angles
     for i in range(3):
      
      self.joint_angle_vars[i].set(angles[i])


    def send_joint_parameters(self):

        kp_values = [float(var.get()) for var in self.kp_vars]       
        kd_values = [float(var.get()) for var in self.kd_vars] 
 
        
        selected_legs = self.get_selected_legs()
        if selected_legs is None:
            print("No leg selected.")
            return
        

        param_array_kp= [0.0] * 12
        param_array_kd= [0.0] * 12
        
        mask = [False] * 12
        for selected_leg in selected_legs:
         leg_index = {"FR": 0, "FL": 1, "RR": 2, "RL": 3}[selected_leg]
         start_idx = leg_index * 3


         param_array_kp[start_idx:start_idx+3] = kp_values
         param_array_kd[start_idx:start_idx+3] = kd_values
         mask[start_idx:start_idx+3] = [True, True, True]
 

        print("Sending joint parameters kp:", param_array_kp)
        print("Sending joint parameters kd:", param_array_kd)
        print("Mask:", mask)
        '''print("Sending joint parameters:", torques)
        print("Sending joint parameters:", kp_values)
        print("Sending joint parameters:", kd_values)'''
#Param Message        
        param_msg = ParamsSet()
        param_msg.kp= param_array_kp
        param_msg.kd= param_array_kd
        param_msg.mask=mask
        self.publisher2.publish(param_msg)

    def send_cartesian_motion(self):
        motion_type = self.motion_type_var.get()
        swing_heights = self.swing_height_vars.get()
        swing_direction = [var.get() for var in self.swing_directions_vars]
        swing_time = self.swing_time_vars.get()
        
        selected_leg = self.get_selected_leg1()
        #selected_leg= self.active_leg_var.get()
        print("The selected item is ",selected_leg)
        if selected_leg is None:
            print("No leg selected.")
            return
        
        # Initialize a smaller param_array
        param_array = [0] * 6
        leg_index = {"FR": 0, "FL": 1, "RR": 2, "RL": 3}[selected_leg]
        print("The selected leg index is ",leg_index)

        
        # Update param_array for the selected leg
        param_array[0] = swing_heights
        param_array[1:3] = swing_direction
        param_array[4] = swing_time
        param_array[5] = motion_type

        print("Sending cartesian motion:", param_array)
# Robot Msg:
        motion_msg = MoveLeg()
        motion_msg.motion_type= int(motion_type)
        direction_floats = [float(value) for value in swing_direction]
        motion_msg.direction= direction_floats
        motion_msg.swing_height= float(swing_heights)
        motion_msg.leg_type= int(leg_index)
        motion_msg.duration= int(swing_time)
        self.publisher.publish(motion_msg)


    def send_jointSet_parameters(self):
        angles = [float(var.get()) for var in self.joint_angle_vars]
        velocities = [float(var.get()) for var in self.joint_velocity_vars] 
        torques = [float(var.get()) for var in self.joint_torque_vars]
        
        param_array_p= [0.0] * 3
        param_array_v= [0.0] * 3
        param_array_t= [0.0] * 3
        

        selected_leg = self.get_selected_leg1()
        #selected_leg= self.active_leg_var.get()
        print("The selected item is ",selected_leg)
        leg_index = {"FR": 0, "FL": 1, "RR": 2, "RL": 3}[selected_leg]
        if selected_leg is None:
            print("No leg selected.")
            return
        start_idx=0
        param_array_p[start_idx:start_idx+3] = angles
        param_array_v[start_idx:start_idx+3] = velocities
        param_array_t[start_idx:start_idx+3] = torques

        print("Sending joint angles:", param_array_p)
        print("Sending joint parameters velocity:", param_array_v)
        print("Sending joint parameters torques:", param_array_t)

        Joint_msg = JointsSet()
        Joint_msg.leg_type = leg_index
        Joint_msg.q= param_array_p
        Joint_msg.dq= param_array_v
        Joint_msg.tau= param_array_t

        self.publisher1.publish(Joint_msg)


    def get_selected_leg(self):
        if self.fl_leg_var.get():
            return "FL"
        elif self.fr_leg_var.get():
            return "FR"
        elif self.rl_leg_var.get():
            return "RL"
        elif self.rr_leg_var.get():
            return "RR"
        else:
            return None
            
    def get_selected_leg1(self):
        if self.active_leg_var.get() == "FL":
            return "FL"
        elif self.active_leg_var.get() == "FR":
            return "FR"
        elif self.active_leg_var.get() == "RL":
            return "RL"
        elif self.active_leg_var.get() == "RR":
            return "RR"
        else:
            return None

    def get_selected_legs(self):
       selected_legs = []
       if self.fl_leg_var.get():
        selected_legs.append("FL")
       if self.fr_leg_var.get():
        selected_legs.append("FR")
       if self.rl_leg_var.get():
        selected_legs.append("RL")
       if self.rr_leg_var.get():
        selected_legs.append("RR")
       return selected_legs

    def stand_up(self):
        stand_msg = Int32()
        stand_msg.data = 1;
        self.publisher3.publish(stand_msg)

        print("Command: Stand Up",stand_msg)


    def sit_down(self):
    
        sit_msg =Int32()
        sit_msg.data = 0;
        self.publisher3.publish(sit_msg)
        print("Command: Sit Down",sit_msg)

    def move(self, direction):
        print(f"{direction} button clicked")

    def get_values(self):
    # Retrieve values from the entry widgets
      values = {param: entry.get() for param, entry in self.entries.items()}
      print(values)  # Print values for debugging
      return values
    
    def gait_move(self, direction):
        Gait_msg = GaitParam()

        gait_index = {"Trot": 0, "Sneak": 1}[self.gait_var.get()]
        values = self.get_values()
    

        Gait_msg.gait_type = gait_index
        # Use a default value if the entry is empty

        velocity = float(values.get("Velocity:", "0.0"))
        
        nominal_height = float(values.get("Nominal Height:", "0.0"))
        
        stance_duration = float(values.get("Stance Duration:", "0.0"))
        
        stance_depth = float(values.get("Stance Depth:", "0.0"))
  
        swing_height = float(values.get("Swing Height:", "0.0"))
    

        Gait_msg.velocity = velocity
        Gait_msg.nominal_height = nominal_height
        Gait_msg.stance_duration = stance_duration
        Gait_msg.stance_depth = stance_depth
        Gait_msg.swing_height = swing_height

        Gait_msg.movement =int( direction)
        self.publisher4.publish(Gait_msg)
        print(f"Gait Control {direction} button clicked")

    def adjust_value(self, entry, delta):
         try:
            # Get current value, default to 0.0 if empty
            current_value = float(entry.get() or "0.0")
            # Update value based on delta
            new_value = current_value + delta
            # Ensure new value is not negative
            if new_value < 0:
                new_value = 0
            # Update the entry widget with the new value
            entry.delete(0, tk.END)
            entry.insert(0, f"{new_value:.1f}")  # Formatting to one decimal place
         except ValueError:
            # Handle the case where the entry cannot be converted to float
            entry.delete(0, tk.END)
            entry.insert(0, "0.0")

    def low_state_callback(self, msg):

     motor_states = msg.motor_state  # This is an array of MotorState messages
     
     #self.cur_joint_angles = [motor_states[i].q for i in range(min(3, len(motor_states)))]
     self.joint_angles_FR = [motor_states[i].q for i in range(3)]
     self.joint_angles_FL = [motor_states[i].q for i in range(3, 6)]
     self.joint_angles_RR = [motor_states[i].q for i in range(6, 9)]
     self.joint_angles_RL = [motor_states[i].q for i in range(9, 12)]

     selected_leg = self.active_leg_var.get()
     if self.active_leg != selected_leg:
        self.active_leg = selected_leg
        self.on_leg_select()


     #self.on_leg_select()
       

    def ros_spin_thread(self):
        # Spin ROS node in a separate thread
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)


def main(args=None):
  root = tk.Tk()
  rclpy.init()
  node = rclpy.create_node('robot_control_gui_node')
  my_gui = RobotControlGUI(root, node)

# Start ROS spinning in a separate thread
  ros_thread = threading.Thread(target=my_gui.ros_spin_thread)
  ros_thread.start()
  
  root.mainloop()
  rclpy.shutdown()

  ros_thread.join()

if __name__ == '__main__':
    main()
