import cv2
import numpy as np
import requests

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
import queue
### SBLIMP
lock = threading.Lock()

data_queue = queue.Queue()
folder = "GroundFinal"
if not os.path.exists(folder):
    os.makedirs(folder)
instance = "6"

def find_point_on_circle(nx, ny, t, s):#n radius, t time, s frequency 
    # Calculate frequency
    f = 1 / s

    # Calculate the angle for time t
    angle = 2 * math.pi * f * t

    # Calculate x and y coordinates
    x = nx * math.cos(angle)
    y = ny * math.sin(angle)

    return x, y

PORT = 'COM20'

mass = 4
massthrust = 10
thrustratio = mass/massthrust

basethrust = .4
heightSet = 1.6
yawSet = 0 + np.pi

feedbackPD = { "roll" : 0,
  "pitch" : 0,
  "yaw" : 1,
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
  "kpyaw" : -.8*thrustratio,
  "kdyaw" : -24*thrustratio,

  "kpx" : 0,
  "kdx" : 0,
  "kpy" : 0,
  "kdy" : 0,
  "kpz" : 1.3*thrustratio,#.5,#.5
  "kdz" : .19*thrustratio,#-3
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



### Global Variables ###


TOTAL_INTERVALS = 99            # Total number of intervals in the demo video
INTERVAL_LENGTH = 6             # Number of frames in a timeline interval
SKIP_INTERVAL = 1                # Interval between donkey and carrot

V_GAIN = 3                       # Gain of velocity
W_GAIN = 400                     # Gain of angular velocity

CONNECT_TO_ROBOT = True          # Whether to connect to the robot
VX_VALUES = []                    # A list of linear velocities
VY_VALUES = []                    # A list of linear velocities
PX_VALUES = []                    # A list of linear velocities
PY_VALUES = []                    # A list of linear velocities
angle_values = []                 # A list of angles
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

print("running", is_running)
#if not is_running:

    
# Fix Python 2.x.
try: input = raw_input
except NameError: pass

def runRobot():
    
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
    
    
    # xy_d = np.array([positions[lead_robot_id][0], 
    #                 positions[lead_robot_id][1]])1.8234654664993286, -0.22969473898410797, 0.45290014147758484
    xy_d = np.array([1.823,0.4529])
    xy_center = np.array([0,0])

    xyp = 0.0031#.0032#.0033
    xyd = 0.007#0.011#0.006#0.0045
    xyi = 0.00001
    ex_norm_pid = PID(xyp, xyi, xyd , setpoint = 0.)
    ey_norm_pid = PID(xyp, xyi, xyd, setpoint = 0.)
    ex_norm_pid.proportional_on_measurement = True
    ey_norm_pid.proportional_on_measurement = True
    ex_norm_pid.differential_on_measurement = False
    ey_norm_pid.differential_on_measurement = False
    
    #lead_z_pid = PID(.6, 0.02, .4, setpoint = 1.8)
    # yaw_pid = PID(0.2, 0.02, 0.4, setpoint = 0)
    # yaw_pid.error_map = pi_clip

    yaw_diff = yawSet
    realx = 0
    realy = 0
    xygamma = 0.975
    
    robo_time = 0
    x_con = 0
    y_con = 0
    h_con = 0
    a_con = 0
    angle_accum = 0
    anglegamma1 = .8
    anglegamma2 = .99
    angle_counter = 0
    ################################################
    ddgamma = 0
    ddx = 0
    ddy = 0
    try:
        while True:
            key = cv2.waitKey(1)
            if key == 27:
                break
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
                yaw_diff += .3 * joystick.get_axis(0)*(time.time() - time_start)  # right handler: left-right
                print(yaw_diff)
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
            absz = 0
            lead_fz = 2
            
            #print(rotations[robot_id], positions[robot_id])
            acquired = lock.acquire(blocking=False)#blocking=False
            if acquired:
                if not data_queue.empty():
                    print("Reading data...")
                    data = data_queue.get()
                    robo_time = data[0]
                    x_con = data[1]
                    y_con = data[2]
                    h_con = data[3]
                    if data[5] > 10:
                        a_con = pi_clip(data[4])# + np.pi/2)
                    else:
                        a_con = 0
                    lock.release()
                    realx = realx * xygamma +x_con * (1-xygamma)
                    realy = realy * xygamma +y_con * (1-xygamma)
                    tx  = ex_norm_pid(realx)
                    ey_norm_pid(realy)
                    px, ix, dx =  ex_norm_pid.components   # forces of x in the body frame 
                    py, iy, dy =  ey_norm_pid.components   # forces of x in the body frame 
                    ddx = ddx * ddgamma + dx * (1-ddgamma)
                    ddy = ddy * ddgamma + dy * (1-ddgamma)
                    lead_fx = px + ix + ddx
                    lead_fy =  py + iy + ddy   # forces of x in the body frame 
                else:
                    lock.release()
                

            if b_state:
                if  time.time() - robo_time < .25:
                    #yaw_pid.setpoint = 0
                    
                    lead_tauz = yaw_diff#yaw_pid(rotations[robot_id][2]*np.pi/180)#yaw_pid(a_con)#
                    lead_fz = heightSet#lead_z_pid(positions[robot_id][2])#lead_z_pid(h_con)#
                    #print(round(a_con,2), round(-rotations[robot_id][2]*np.pi/180, 2), round(h_con,2))
                    
                    force_vec = np.array([lead_fx, lead_fy])
                    
                    
                    if np.linalg.norm(force_vec) > (lead_fz  )*.4:
                        force_vec = force_vec/np.linalg.norm(force_vec) * lead_fz*.4
                        
                        
                    yaw_diff += a_con * .01# * anglegamma2 **angle_counter 
                    #print(yaw_diff)
                    angle_counter += 1
                    #angle_values.append(round(angle_accum,2))
                    #print(round(angle_accum, 2))
                    fx = force_vec[1]
                    fy = force_vec[0]   
                    tauz = lead_tauz 
                    fz = lead_fz 
                    # PX_VALUES.append(positions[robot_id][0])
                    # PY_VALUES.append(positions[robot_id][1])
                    # VX_VALUES.append(fx)
                    # VY_VALUES.append(fy)
                else:
                    lead_tauz = yaw_diff#yaw_pid(rotations[robot_id][2]*np.pi/180)#yaw_pid(a_con)
                    lead_fz = heightSet#lead_z_pid(positions[robot_id][2])
                    fx = fx
                    fy = fy    
                    tauz = lead_tauz
                    fz = lead_fz 
                    # PX_VALUES.append(positions[robot_id][0])
                    # PY_VALUES.append(positions[robot_id][1])
                    # VX_VALUES.append(fx)
                    # VY_VALUES.append(fy)
            
            # print(fx, fy, fz, tauz)

            esp_now_input = Control_Input(
                21,int(b_state), fx, fy, fz+ absz, taux, tauy, tauz, basethrust, 0, 0, 0, 0
            )
            if b_state:
                esp_now_send(sock, esp_now_input)
                

            # state = not state
            time.sleep(0.005)  # 0.005
            # while(time.time() - time_start < 0.01):
            # time.sleep(0.001) #0.005
    except KeyboardInterrupt:
        print("The end")
    #print(angle_values)
    # save positional data
    # xy_array = np.column_stack((PX_VALUES, PY_VALUES))
    # np.save(folder + '/position' + str(instance) + '.npy', xy_array)
    
    
    # Create a figure and a 1x2 grid of subplots
    # fig, axs = plt.subplots(1, 2, figsize=(12, 6))

    # Plot positions as a trajectory in 2D space
    # optiOrigin = np.load("Timeline\\opti_arrayCircle.npy")
    # print(optiOrigin.shape)
    # optiXY = optiOrigin[:, 0, :2][0:500]
    # axs[0].plot(-1*np.array(optiXY[:, 1]),np.array(optiXY[:, 0]), marker='x', linestyle='-')
    # axs[0].plot(-1*np.array(PY_VALUES),np.array(PX_VALUES), marker='o', linestyle='-')
    # axs[0].set_title('Positional Trajectory')
    # axs[0].set_xlabel('X Position')
    # axs[0].set_ylabel('Y Position')
    

    

    # Add annotations to indicate the time step
    # for i, (x, y) in enumerate(zip(PX_VALUES, PY_VALUES)):
    #     axs[0].annotate(f'T={i}', (x, y), textcoords="offset points", xytext=(0, 10), ha='center')

    # Plot velocities
    
    # min_dists = np.empty(xy_array.shape[0])
    # for i, point2 in enumerate(xy_array):
    #     # Compute the Euclidean distances to all points in array1
    #     distances = np.linalg.norm(optiXY - point2, axis=1)
        
    #     # Find the minimum distance for the current point in array2
    #     min_dists[i] = np.min(distances)
    # axs[1].plot(min_dists, label='Error', marker='o', linestyle='-')
    # #axs[1].plot(VY_VALUES, label='Y Velocity', marker='x', linestyle='-')
    # axs[1].set_title('Error In Distance')
    # axs[1].set_xlabel('Time')
    # axs[1].set_ylabel('Error')
    # axs[1].legend()

    # # Show the plots
    # plt.tight_layout()
    # plt.savefig('p_and_e_plot.png')
        


fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # or use 'XVID'
fps = 10



# Initialize the counter
# Create a video writer object



opti_arr = []
mean_arr = []
screen_w = 400
screen_h = 300

    
            
def stream_loop():
    
    print("stream thread")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    fps = 20
    out1 = cv2.VideoWriter(folder + "/robot" + str(instance) + ".mp4", fourcc, fps, (screen_w, screen_h))
    out2 = cv2.VideoWriter(folder + "/carrot" + str(instance) + ".mp4", fourcc, fps, (screen_w, screen_h))
    out3 = cv2.VideoWriter(folder + "/raw" + str(instance) + ".mp4", fourcc, fps, (screen_w, screen_h))
    URL = "http://192.168.0.18"
    AWB = True
    # Face recognition and opencv setup
    cap = cv2.VideoCapture(URL + ":81/stream")
    starting_frame = 0
    wall_tracker = WallTraker(None, TOTAL_INTERVALS, INTERVAL_LENGTH, SKIP_INTERVAL, starting_frame)


    position = -1      # The current interval
    lost_count = 0     # The number of times the robot lost the wall
    
    first = True
    robo_time = time.time()
    mean_count = 0
    meanave = 100
    meangamma = .5
    while True:
        if cap.isOpened() and time.time() - robo_time > .02:
            
            ret, robot_frame = cap.read()
            if ret:
                
                
                #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                #robot_frame = cv2.equalizeHist(gray)
                
                # Resize the frame
                robot_frame = cv2.resize(robot_frame, (screen_w, screen_h), interpolation=cv2.INTER_LINEAR)
                out3.write(robot_frame)
                wall_tracker.update_robot(robot_frame)
                x_diff, y_diff, height_diff, angle_diff, num_match, lost = wall_tracker.chase_carrot()
                robot_frame, carrot_frame = wall_tracker.show_all_frames()
                out1.write(robot_frame)
                out2.write(carrot_frame)
                if not lost:
                    robo_time = time.time()
                    
                    with lock:
                        data_queue.put([robo_time, x_diff, y_diff, height_diff, angle_diff, num_match])
                opti_arr.append([positions[robot_id][0], positions[robot_id][1]])
                mean_arr.append([x_diff, y_diff])
                meanave = meanave * meangamma + np.sqrt(x_diff**2 + y_diff**2) * (1-meangamma)
                if meanave < 20 and num_match > 10: # If the robot is close enough to the carrot
                    
                    mean_count += 1
                    if mean_count == 10:
                        position = wall_tracker.next_carrot() # Go to the next carrot
                        print("next carrot")
                        lost_count = 0 # Reset the lost count
                        mean_count = 0
                else:
                    mean_count = 0

                if position == TOTAL_INTERVALS:
                    print("Finish!")
                    break
                #time.sleep(0.03)
                #cv2.imshow("frame", robot_frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    print("release")
    out1.release()
    out2.release()
    out3.release()
    cv2.destroyAllWindows()
    cap.release()
    

    # Save the array to a text file
    file_path = os.path.join(folder, 'opti_array' + str(instance) + '.npy')
    print("RELEASE2")
    np.save(file_path, np.array(opti_arr))
    file_path = os.path.join(folder, 'mean_array' + str(instance) + '.npy')
    print("RELEASE3")
    np.save(file_path, np.array(mean_arr))





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