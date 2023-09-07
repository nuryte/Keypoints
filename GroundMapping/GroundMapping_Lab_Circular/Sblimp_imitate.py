# Image Transfer - As The Controller Device
#
# This script is meant to talk to the "image_transfer_jpg_streaming_as_the_remote_device_for_your_computer.py" on the OpenMV Cam.
#
# This script shows off how to transfer the frame buffer to your computer as a jpeg image.

import io, pygame, rpc, serial, serial.tools.list_ports, socket, sys, os
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler
from WallTraker import WallTraker
import matplotlib.pyplot as plt
import math, time
import cv2
import threading
from simple_pid import PID
### SBLIMP
lock = threading.Lock()


def find_point_on_circle(nx, ny, t, s):#n radius, t time, s frequency 
    # Calculate frequency
    f = 1 / s

    # Calculate the angle for time t
    angle = 2 * math.pi * f * t

    # Calculate x and y coordinates
    x = nx * math.cos(angle)
    y = ny * math.sin(angle)

    return x, y

PORT = 'COM5'

feedbackPD = { "roll" : 0,
  "pitch" : 0,
  "yaw" : 0,
  "x" : 0,
  "y" : 0,
  "z" : 1,
  "rotation" : 0,

  "Croll" : 0,
  "Cpitch" : 0, 
  "Cyaw" : 1,
  "Cx" : 1,
  "Cy" : 1,
  "Cz" : 1,
  "Cabsz" : 1,

  "kproll" : 0,
  "kdroll" : 0 ,
  "kppitch" : 0,
  "kdpitch" : 0,
  "kpyaw" : 3.0,
  "kdyaw" : -120,

  "kpx" : 0,
  "kdx" : 0,
  "kpy" : 0,
  "kdy" : 0,
  "kpz" : .05,#.5
  "kdz" : 0,#-3
  "kiz" : 0,

  "integral_dt" : 0,#.0001,
  "z_int_low" : 0,
  "z_int_high" : 200,

  "lx" : .15,
  "pitchSign" : 1,
  "pitchOffset" : -3.2816
}
weights = { "eulerGamma" : 0,
  "rollRateGamma" : 0.7,
  "yawRateGamma" : 0.975,
  "pitchRateGamma" : 0.7,
  "zGamma" : 0.5,
  "vzGamma" : 0.5
}

class Control_Input:
    def __init__(self, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13):
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.p4 = p4
        self.p5 = p5
        self.p6 = p6
        self.p7 = p7
        self.p8 = p8
        self.p9 = p9
        self.p10 = p10
        self.p11 = p11
        self.p12 = p12
        self.p13 = p13

    def __str__(self) -> str:
        return (
            '<'
            + str(self.p1)
            + '|'
            + str(self.p2)
            + '|'
            + str(self.p3)
            + '|'
            + str(self.p4)
            + '|'
            + str(self.p5)
            + '|'
            + str(self.p6)
            + '|'
            + str(self.p7)
            + '|'
            + str(self.p8)
            + '|'
            + str(self.p9)
            + '|'
            + str(self.p10)
            + '|'
            + str(self.p11)
            + '|'
            + str(self.p12)
            + '|'
            + str(self.p13)
            + '>'
        )


def espnow_init():
    ser = serial.Serial(PORT, 115200)
    return ser


def joystick_init():
    pygame.display.init()
    pygame.joystick.init()
    pygame.joystick.Joystick(0).init()

    # Prints the values for axis0
    joystick = pygame.joystick.Joystick(0)
    return joystick


def esp_now_send(ser, input):
    try:
        # NOTE - The daley here need to match the delay in the ESP32 receiver code
        message = str(input)
        ser.write(message.encode())
        try:
            incoming = ser.readline().decode(errors='ignore').strip()
            #print("Received Data: " + incoming)
        except UnicodeDecodeError:
            print("Received malformed data!")
    except KeyboardInterrupt:
        print("Exiting Program")
        ser.close()


def init():
    joystick = joystick_init()
    return joystick

def sendAllFlags(sock):
    esp_now_input = Control_Input(
        10, 0,
        feedbackPD["roll"], 
        feedbackPD["pitch"], 
        feedbackPD["yaw"],  
        feedbackPD["x"], 
        feedbackPD["y"], 
        feedbackPD["z"], 
        feedbackPD["rotation"], 
        0, 0, 0, 0
    )
    esp_now_send(sock, esp_now_input)
    time.sleep(0.05)  # 0.005
    
    esp_now_input = Control_Input(
        11, 0,
        feedbackPD["Croll"], 
        feedbackPD["Cpitch"], 
        feedbackPD["Cyaw"],  
        feedbackPD["Cx"], 
        feedbackPD["Cy"], 
        feedbackPD["Cz"], 
        feedbackPD["Cabsz"],
        0, 0, 0, 0
    )
    esp_now_send(sock, esp_now_input)
    time.sleep(0.05)  # 0.005
    
    esp_now_input = Control_Input(
        12, 0,
        feedbackPD["kproll"], 
        feedbackPD["kdroll"], 
        feedbackPD["kppitch"],  
        feedbackPD["kdpitch"], 
        feedbackPD["kpyaw"], 
        feedbackPD["kdyaw"], 
        0, 0, 0, 0, 0
    )
    esp_now_send(sock, esp_now_input)
    time.sleep(0.05)  # 0.005
    
    esp_now_input = Control_Input(
        13, 0,
        feedbackPD["kpx"], 
        feedbackPD["kdx"], 
        feedbackPD["kpy"],  
        feedbackPD["kdy"], 
        feedbackPD["kpz"], 
        feedbackPD["kdz"],  
        feedbackPD["lx"], 
        feedbackPD["pitchSign"],  
        feedbackPD["pitchOffset"], 
        0,0
    )
    esp_now_send(sock, esp_now_input)
    time.sleep(0.05)  # 0.005
    
    esp_now_input = Control_Input(
        14, 0,
        weights["eulerGamma"], 
        weights["rollRateGamma"], 
        weights["pitchRateGamma"],  
        weights["yawRateGamma"], 
        weights["zGamma"], 
        weights["vzGamma"], 
        0, 0, 0, 0, 0
    )
    esp_now_send(sock, esp_now_input)
    time.sleep(0.05)  # 0.005
    
    esp_now_input = Control_Input(
        15, 0,
        feedbackPD["kiz"], 
        feedbackPD["integral_dt"],  
        feedbackPD["z_int_low"], 
        feedbackPD["z_int_high"],  
        0,0,
        0,0,0,0,0
    )
    esp_now_send(sock, esp_now_input)
    time.sleep(0.05)  # 0.005

# correction of the angle 

def pi_clip(angle):
    if angle > 0:
        while angle > math.pi:
            angle = angle - 2*math.pi
    else:
        while angle < -math.pi:
            angle = angle + 2*math.pi
    return angle
x_con = 0
y_con = 0
h_con = 0
a_con = 0
robo_time = 0
def runRobot():
    global x_con, y_con, h_con, a_con
    print("robo thread")
    sock = espnow_init()


    sendAllFlags(sock)
    
    time.sleep(0.05)  # 0.005

    joystick = init()
    
    absz = 0
    b_old = 0
    b_state = 0
    x_old = 0
    x_state = 1
    

    tauz = 0
    fx = 0
    

    time_start = time.time()
    time_all=  time.time()

    #####################################
    
     ###############################################





    #tow_z_pid = PID(3, 0, 1, setpoint = 1, sample_time=0.01)
    lead_z_pid = PID(.6, 0.01, .4, setpoint = 0)
    yaw_pid = PID(0.2, 0, 0.1, setpoint = 0.)
    yaw_pid.error_map = pi_clip
    
    # xy_d = np.array([positions[lead_robot_id][0], 
    #                 positions[lead_robot_id][1]])1.8234654664993286, -0.22969473898410797, 0.45290014147758484
    xy_d = np.array([1.823,0.4529])
    xy_center = np.array([0,0])

    xyp = .2#.6
    xyd = .1#.4
    xyi = 0
    ex_norm_pid = PID(xyp, xyi, xyd, setpoint = 0.)
    ey_norm_pid = PID(xyp, xyi, xyd, setpoint = 0.)


    
    
    ################################################
    
    try:
        while True:
            # if pygame.joystick.get_count() == 0:
            #     while pygame.joystick.get_count() == 0:
            #         print("joy_lost")
            #         time.sleep(.02)
            #     joystick = init()
                
            # Get the joystick readings
            pygame.event.pump()
            b = joystick.get_button(1)
            x = joystick.get_button(2)
            y = joystick.get_button(3)
            if y:
                break
            left = joystick.get_hat(0)[0] == -1
            right = joystick.get_hat(0)[0] == 1
            fy = 0
            if b == 1 and b_old == 0:
                b_state = not b_state
                
                
            b_old = b

            if x == 1 and x_old == 0:
                x_state = not x_state
            x_old = x

            if abs(joystick.get_axis(3)) > 0.1:
                fx = -1 * joystick.get_axis(3)  # left handler: up-down, inverted
            else:
                fx = 0

            if abs(joystick.get_axis(0)) > 0.1:
                tauz = .5 * joystick.get_axis(0)  # right handler: left-right
            else:
                tauz = 0
            if abs(joystick.get_axis(2)) > 0.1:
                fy = -1 * joystick.get_axis(2)  # right handler: left-right
            else:
                fy = 0

            fz = 0  # -2*joystick.get_axis(1)  # right handler: up-down, inverted



            l_old = left
            r_old = right
            
            tauy = 0
            taux = 0
            # absz = .5

            if abs(joystick.get_axis(1)) > 0.15:
                absz += -(time.time() - time_start) * joystick.get_axis(1)*.3
            
            
            if b_state == 0:
                absz = 0
                x_state = 0

            time_start = time.time()

            lead_fz = 1
            with lock:
                
                if  time.time() - robo_time < .25:
                    #yaw_pid.setpoint = 0

                    
                    lead_tauz = 0#yaw_pid(a_con)
                    #lead_fz += lead_z_pid(h_con)
                    
                        
                    lead_fx = fx - ex_norm_pid(x_con)   # forces of x in the body frame 
                    lead_fy = fy - ey_norm_pid(y_con)   # forces of x in the body frame 
                    force_vec = np.array([lead_fx, lead_fy])
                    
                    if np.linalg.norm(force_vec) > lead_fz*.7:
                        force_vec = force_vec/np.linalg.norm(force_vec) * lead_fz*.7
                        
                    fx = force_vec[0]
                    fy = force_vec[1]    
                    tauz = lead_tauz
                    fz = lead_fz + absz
                else:
                    fx = 0
                    fy = 0    
                    tauz = 0
                    fz = lead_fz
            zOffset = .4#.6
            
            #print(fx, fy, fz, tauz)

            esp_now_input = Control_Input(
                21,int(b_state), fx, fy, fz, taux, tauy, tauz- .035, zOffset, 0, 0, 0, 0
            )
            esp_now_send(sock, esp_now_input)
                

            # state = not state
            time.sleep(0.01)  # 0.005
            # while(time.time() - time_start < 0.01):
            # time.sleep(0.001) #0.005
    except KeyboardInterrupt:
        print("The end")
        


### Global Variables ###


TOTAL_INTERVALS = 100            # Total number of intervals in the demo video
INTERVAL_LENGTH = 5             # Number of frames in a timeline interval
SKIP_INTERVAL = 2                # Interval between donkey and carrot

V_GAIN = 3                       # Gain of velocity
W_GAIN = 400                     # Gain of angular velocity

CONNECT_TO_ROBOT = True          # Whether to connect to the robot
V_VALUES = []                    # A list of linear velocities
ω_VALUES = []                    # A list of angular velocities
NUM_MATCH = []                   # A list of number of matches


positions = {}
rotations = {}


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(id, position, rotation_quaternion):
    # Position and rotation received
    positions[id] = position
    # The rotation is in quaternion. We need to convert it to euler angles
    rotx, roty, rotz = quaternion_to_euler(rotation_quaternion)
    # Store the roll pitch and yaw angles
    rotations[id] = (rotx, roty, rotz)

def plot_speeds():
    # # Plot v values
    # plt.figure()
    # plt.plot(V_VALUES)
    # plt.title('Velocity (v) over Time')
    # plt.xlabel('Time')
    # plt.ylabel('Velocity (v)')
    # plt.grid(True)
    # plt.savefig("v_plot.png")
    # plt.show()

    # Plot ω values
    # plt.figure(1)  # Create a new figure window
    # plt.plot(ω_VALUES)
    # plt.title('Angular Velocity (ω) over Time')
    # plt.xlabel('Time')
    # plt.ylabel('Angular Velocity (ω)')
    # plt.grid(True)
    # plt.savefig("omega_plot.png")

    # # Plot number of matches
    # plt.figure(2)  # Create another new figure window
    # plt.plot(NUM_MATCH)
    # plt.title('Number of Matches over Time')
    # plt.xlabel('Time')
    # plt.ylabel('Number of Matches')
    # plt.grid(True)
    # plt.savefig("match_plot.png")

    # # Now show both figures
    # plt.show()
    pass

clientAddress = "192.168.0.14"
optitrackServerAddress = "192.168.0.4"

robot_id = 384

# This will create a new NatNet client
streaming_client = NatNetClient()
streaming_client.set_client_address(clientAddress)
streaming_client.set_server_address(optitrackServerAddress)
streaming_client.set_use_multicast(True)
# Configure the streaming client to call our rigid body handler on the emulator to send data out.
streaming_client.rigid_body_listener = receive_rigid_body_frame

# Start up the streaming client now that the callbacks are set up.
# This will run perpetually, and operate on a separate thread.
is_running = streaming_client.run()


# Fix Python 2.x.
try: input = raw_input
except NameError: pass

# The RPC library above is installed on your OpenMV Cam and provides mutliple classes for
# allowing your OpenMV Cam to control over USB or WIFI.

##############################################################
# Choose the interface you wish to control an OpenMV Cam over.
##############################################################

# Uncomment the below lines to setup your OpenMV Cam for controlling over a USB VCP.
#
# * port - Serial Port Name.
#
# print("\nAvailable Ports:\n")
# for port, desc, hwid in serial.tools.list_ports.comports():
#     print("{} : {} [{}]".format(port, desc, hwid))
# sys.stdout.write("\nPlease enter a port name: ")
# sys.stdout.flush()
# interface = rpc.rpc_usb_vcp_master(port=input())
# print("")
# sys.stdout.flush()

# Uncomment the below line to setup your OpenMV Cam for controlling over WiFi.
#
# * slave_ip - IP address to connect to.
# * my_ip - IP address to bind to ("" to bind to all interfaces...)
# * port - Port to route traffic to.
#
interface = rpc.rpc_network_master(slave_ip = "192.168.0.47", my_ip = "",port=7610)


fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # or use 'XVID'
fps = 10


##############################################################
# Call Back Handlers
##############################################################

# Initialize the counter
position = -1      # The current interval
lost_count = 0     # The number of times the robot lost the wall
# Create a video writer object

clock = pygame.time.Clock()
screen = None
opti_arr = []
screen_w = 400
screen_h = 300
def ending():
    
    
    print("RELEASE")
    out1.release()
    out2.release()
    #cv2.destroyAllWindows()
    folder_path = 'GroundMapping\\GroundMapping_Lab_Circular\\numpy'
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    # Save the array to a text file
    file_path = os.path.join(folder_path, 'opti_array.npy')
    print("RELEASE2")
    np.save(file_path, np.array(opti_arr))
    pygame.quit()
    print("RELEASE3")
    sys.exit()
    
# This will be called with the bytes() object generated by the slave device.
def jpg_frame_buffer_cb(data):
    global out1, out2, position, lost_count, wall_tracker, first
    global x_con, y_con, h_con, a_con, robo_time
    #print("Data In")
    sys.stdout.flush()

    try:
        dat_img = pygame.image.load(io.BytesIO(data), "jpg")
        dat_img = pygame.transform.scale(dat_img, (screen_w, screen_h))
        screen.blit(dat_img, (0, 0))
        pygame.display.update()
        # Convert Pygame surface to NumPy array
        frame = pygame.surfarray.array3d(dat_img)# Convert frame from RGB to BGR format (OpenCV uses BGR)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        robot_frame = cv2.transpose(frame)
        #frame = cv2.flip(frame, 1)
        if robot_id in positions:
                # last position
                opti_arr.append([positions[robot_id], rotations[robot_id]])
                print('Last position', positions[robot_id], ' rotation', rotations[robot_id])

        if first:
            first = False
            
            # Initialize the counter
            position = -1      # The current interval
            lost_count = 0     # The number of times the robot lost the wall
        else:
            if not position == -1: print("Going to interval: ", position)
            wall_tracker.update_robot(robot_frame)
            x_diff, y_diff, height_diff, angle_diff, num_match, lost = wall_tracker.chase_carrot()
            robot_frame, carrot_frame = wall_tracker.show_all_frames()
            out1.write(robot_frame)
            out2.write(carrot_frame)
            with lock:
                robo_time = time.time()
                x_con = x_diff
                y_con = y_diff
                h_con = height_diff
                a_con = angle_diff
            if np.sqrt(x_diff**2 + y_diff**2) < 10 and not lost: # If the robot is close enough to the carrot
                position = wall_tracker.next_carrot() # Go to the next carrot
                lost_count = 0 # Reset the lost count

            if position == TOTAL_INTERVALS:
                if position == TOTAL_INTERVALS: print("Finish!")
                # De-allocate any associated memory usage
                ending()
        

        clock.tick()
    except pygame.error as e:
        print(f"Pygame error: {e}")

    print(clock.get_fps())

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            ending()
            
def stream_loop():
    global out1, out2, first, wall_tracker
    print("stream thread")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    fps = 20
    out1 = cv2.VideoWriter("robot.mp4", fourcc, fps, (screen_w, screen_h))
    out2 = cv2.VideoWriter("carrot.mp4", fourcc, fps, (screen_w, screen_h))
    pygame.init()
    wall_tracker = WallTraker(None, TOTAL_INTERVALS, INTERVAL_LENGTH, SKIP_INTERVAL)

    global screen
    try:
        screen = pygame.display.set_mode((screen_w, screen_h), flags=pygame.RESIZABLE)
    except TypeError:
        screen = pygame.display.set_mode((screen_w, screen_h))
    pygame.display.set_caption("Frame Buffer")
    
    first = True
    try:
        while(True):
            sys.stdout.flush()

            # You may change the pixformat and the framesize of the image transfered from the remote device
            # by modifying the below arguments.
            result = interface.call("jpeg_image_stream", "sensor.GRAYSCALE,sensor.HVGA")
            if result is not None:

                # THE REMOTE DEVICE WILL START STREAMING ON SUCCESS. SO, WE NEED TO RECEIVE DATA IMMEDIATELY.
                interface.stream_reader(jpg_frame_buffer_cb, queue_depth=8, read_timeout_ms=5000)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    print("RELEASE")
                    out1.release()
                    out2.release()
                    print("RELEASE2")
                    
                    pygame.quit()
                    print("RELEASE3")
                    sys.exit()
                
                
    except KeyboardInterrupt:
        sys.exit()

robo_thread = threading.Thread(target = runRobot)
stream_thread = threading.Thread(target = stream_loop)
print("begin threads!")
robo_thread.start()
stream_thread.start()
print("threads!!")

robo_thread.join()
stream_thread.join()
print("Ended threads!")
# print(threading.enumerate())
sys.exit()