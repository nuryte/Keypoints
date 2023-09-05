import cv2 
import sys
import time
from Optitrack_dependency.NatNetClient import NatNetClient
from Optitrack_dependency.util import quaternion_to_euler_angle_vectorized1

positions = {}
rotations = {}

NAME = 'corner.mp4'


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles
    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)
    rotations[robot_id] = rotz


if __name__ == "__main__":
    clientAddress  = "192.168.0.46"
    optitrackServerAddress = "192.168.0.4"
    robot_id = 121

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
    # Create a VideoCapture object to read the video file
    cap = cv2.VideoCapture("http://192.168.0.204:1234/stream.mjpg")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    fps = 20
    out = cv2.VideoWriter(NAME, fourcc, fps, (400, 300))
    with open('record_data.txt', 'w') as file:  # Open the file in write mode
        start_time = time.time()  # Start the timer
        while cap.isOpened():
            # Read a frame from the video
            ret, frame = cap.read()
            # Check if the frame was successfully read
            if not ret: break
            if not is_running: break
            if robot_id in positions:
                x = positions[robot_id][0]
                y = positions[robot_id][1]
                rotation = rotations[robot_id]
                elapsed_time = time.time() - start_time  # Calculate elapsed time
                # Write to the file
                file.write(f"{elapsed_time:.2f}, {x}, {y}, {rotation}\n")
                # last position
                print('x:',x, 'y:',y, ' rotation:', rotation)
                print(f"Elapsed Time: {elapsed_time:.2f} seconds")
            out.write(frame)
            cv2.imshow("frame", frame)
            # Wait for Esc key to stop
            if cv2.waitKey(1) == ord('q'):
                # De-allocate any associated memory usage
                streaming_client.shutdown()
                cv2.destroyAllWindows()
                cap.release()
                break
