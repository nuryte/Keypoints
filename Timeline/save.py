import cv2
import numpy as np
from collections import Counter

MAX_FRAMES = 600         # large numbers will cover the whole video
start_frame = 0
INTERVAL = 15             # frames per inverval 
INTERVAL_ADVANCE = 6
duplicate_minimum = 8
MAX_MATCH_DISTANCE = 25  # match threshold

# Create an ORB object and detect keypoints and descriptors in the template
orb = cv2.ORB_create(nfeatures=1000)
# Create a brute-force matcher object
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps = 5
full = cv2.VideoWriter("Timeline\\line_demo.mp4", fourcc, 20, (400, 300))
final = cv2.VideoWriter("Timeline\\line_demo_fin.mp4", fourcc, 2, (400, 300))

VIDEO = "Timeline\\rawEx5.mp4"
DESCRIPTOR_FILE = "Timeline\\fly_demo2"
# VIDEO = "corner.mp4"
# DESCRIPTOR_FILE = "side_demo"

def extract_keypoints(video):
    # Create a VideoCapture object to read the video file
    cap = cv2.VideoCapture(video)
    # Extract all keypoints and descriptors by frame
    frame_kpt, frame_des = [], []
    video_frames = []
    k = 0
    # Loop through the video frames
    while cap.isOpened() and k < MAX_FRAMES + 1:
        # Read a frame from the video
        ret, frame = cap.read()
        # Check if the frame was successfully read
        if not ret: continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kpt, des = orb.detectAndCompute(gray, None)
        if des is None:
            print("No keypoints/descriptors in frame ", k)
            continue
        print("Frame", k)
        frame_kpt.append(kpt)
        frame_des.append(des)
        video_frames.append(frame)
        k += 1
        # Wait for Esc key to stop
        # if cv2.waitKey(1) == 27:
        #     # De-allocate any associated memory usage
        #     cv2.destroyAllWindows()
        #     cap.release()
        #     break
    cap.release()
    return frame_kpt, frame_des, video_frames


def save_kpt_des(keypoints, descriptors, filename):
    fs = cv2.FileStorage(filename, cv2.FILE_STORAGE_WRITE)
    # Write the descriptors to the file
    fs.write("descriptors", descriptors)
    # Convert keypoints to a numpy array
    keypoints_array = np.array([keypoint.pt for keypoint in keypoints])
    # Write the keypoints to the file
    fs.write("keypoints", keypoints_array)  
    fs.release()



def remove_duplicates(lst, n):
    # Count occurrences of each element
    element_count = Counter(lst)

    # Keep elements that appear at least n times
    filtered_lst = [element for element in lst if element_count[element] >= n]

    # Remove all remaining duplicates
    unique_lst = []
    unique_set = set()

    for element in filtered_lst:
        if element not in unique_set:
            unique_lst.append(element)
            unique_set.add(element)

    return unique_lst


def analyze_kpt_des(frame, keypoints, descriptors, filename, video):
    # Create a VideoWriter object
    out = cv2.VideoWriter(video, fourcc, fps, (400, 300))

    kpts_cur, des_cur = [], []
    kpts_fin, des_fin = [], []
    kpts_fin_idx = []
    k = INTERVAL
    korigin = int(INTERVAL/2)
    final_frame = frame[korigin].copy()

    while k > 0:
        k -= 1
        # if k == INTERVAL - 1:
        #     kpts_cur = keypoints[k] #current frame keypoints
        #     des_cur = descriptors[k] #current frame descriptors
        #     kpts_fin = keypoints[korigin] #reference frame keypoints
        #     des_fin = descriptors[korigin] #reference frame descriptors
        #     continue
        # else:
        kpts_cur = keypoints[k] #current frame keypoints
        des_cur = descriptors[k] #current frame descriptors
        kpts_fin = keypoints[korigin] #reference frame keypoints
        des_fin = descriptors[korigin] #reference frame descriptors
        
        if descriptors is None:
            print("No keypoints/descriptors in frame")
            continue

        matches = bf.match(des_fin, descriptors[k])
        # matches = sorted(matches, key=lambda x: x.distance)
        matches = [m for m in matches if m.distance < MAX_MATCH_DISTANCE]

        kpts_cur_temp = []
        des_cur_temp = []
        kpts_fin_temp = []
        des_fin_temp = []

        for match in matches:
            query_idx = match.queryIdx
            kpts_fin_temp.append(kpts_fin[query_idx])
            des_fin_temp.append(des_fin[query_idx])
            kpts_fin_idx.append(query_idx)
            train_idx = match.trainIdx
            kpts_cur_temp.append(kpts_cur[train_idx])
            des_cur_temp.append(des_cur[train_idx])

        kpts_fin, des_fin = [], []
        kpts_fin = np.array(kpts_fin_temp)
        des_fin = np.array(des_fin_temp)
        kpts_cur, des_cur = [], []
        kpts_cur = np.array(kpts_cur_temp)
        des_cur = np.array(des_cur_temp)
        

        frame[k] = cv2.drawKeypoints(frame[k], kpts_cur, None, color = (255, 0, 0), flags=0)
        frame[k] = cv2.drawKeypoints(frame[k], kpts_fin, None, color=(0, 255, 0), flags=0)

        pt1s = []
        pt2s = []

        for i in range(len(kpts_fin)):
            pt1 = np.int32(kpts_fin[i].pt)
            pt2 = np.int32(kpts_cur[i].pt)
            pt1s.append(pt1)
            pt2s.append(pt2)
            frame[k] = cv2.line(frame[k], pt1, pt2, (0, 0, 255), thickness=2)

        cv2.imshow("frame", frame[k])
        out.write(frame[k])
        full.write(frame[k])
        if k == 0:
            kpts_fin_idx = remove_duplicates(kpts_fin_idx,duplicate_minimum)
            
            kpts_fin = keypoints[korigin] #reference frame keypoints
            des_fin = descriptors[korigin] #reference frame descriptors
            print("Final Keypoints: ",len(kpts_fin_idx))
            kpts_fin2, des_fin2 = [], []
            for idx in kpts_fin_idx:
                kpts_fin2.append(kpts_fin[idx])
                des_fin2.append(des_fin[idx])
            final_frame = cv2.drawKeypoints(final_frame, kpts_fin2, None, color=(0, 255, 0), flags=0)
            final.write(final_frame)
            save_kpt_des(np.array(kpts_fin2), np.array(des_fin2), filename)
        # Wait for Esc key to stop
        # if cv2.waitKey(1) == 27:
        #     # De-allocate any associated memory usage
        #     cv2.destroyAllWindows()
        #     break
    out.release()


if __name__ == "__main__":
    frame_kpt, frame_des, frames = extract_keypoints(VIDEO)
    frames = frames[-(len(frames)-start_frame):]
    frame_kpt = frame_kpt[-(len(frame_kpt)-start_frame):]
    frame_des = frame_des[-(len(frame_des)-start_frame):]
    # for i in range(int(len(frames)/INTERVAL)):
    #     save_kpt_des(frame_kpt[i*INTERVAL], frame_des[i*INTERVAL], "Timeline//fly_demo2_kpt_des//"+ "fly_demo2_kpt_des%d.yml"%(i+1))
    for i in range( int( len(frames)/INTERVAL_ADVANCE) - int(INTERVAL/INTERVAL_ADVANCE)):
        print("Interval", i+1)
        analyze_kpt_des(frames, frame_kpt, frame_des, "Timeline//fly_demo2_kpt_des//"+ "fly_demo2_kpt_des%d.yml"%(i+1), "Timeline//fly_demo2" + "/fly_demo2_%d.mp4"%(i+1))
        frames = frames[-(len(frames)-INTERVAL_ADVANCE):]
        frame_kpt = frame_kpt[-(len(frame_kpt)-INTERVAL_ADVANCE):]
        frame_des = frame_des[-(len(frame_des)-INTERVAL_ADVANCE):]
    full.release()
    final.release()