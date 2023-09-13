import numpy as np
import matplotlib.pyplot as plt

npy_file = "GroundFinal\opti_arrayEx.npy"


# Step 1: Load the NumPy file
def load_numpy_file(file_path):
    try:
        data = np.load(file_path)
        return data
    except Exception as e:
        print("Error loading NumPy file:", e)
        return None
# Step 2: Create a Matplotlib line plot
def create_line_plot(data):
    if data is not None:
        plt.plot(-1*np.array(data[:, 1]),np.array(data[:, 0]), marker='x', linestyle='-')
        plt.xlabel('X-axis label')  # Customize the X-axis label
        plt.ylabel('Y-axis label')  # Customize the Y-axis label
        plt.title('Line Plot')      # Customize the plot title
        plt.grid(True)              # Enable grid lines (optional)
        plt.show()

def main():
    # Specify the path to your NumPy file
    numpy_file_path = "GroundFinal\opti_arrayEx.npy"
    
    # Step 1: Load the NumPy file
    loaded_data = load_numpy_file(numpy_file_path)
    print(np.shape(loaded_data))
    # Step 2: Create a Matplotlib line plot
    if loaded_data is not None:
        create_line_plot(loaded_data)

if __name__ == "__main__":
    main()
# fig, axs = plt.subplots(1, 2, figsize=(12, 6))

# # Plot positions as a trajectory in 2D space
# optiOrigin = np.load("Timeline\\opti_arrayCircle.npy")
# print(optiOrigin.shape)
# optiXY = optiOrigin[:, 0, :2][0:500]
# axs[0].plot(-1*np.array(optiXY[:, 1]),np.array(optiXY[:, 0]), marker='x', linestyle='-')
# axs[0].plot(-1*np.array(PY_VALUES),np.array(PX_VALUES), marker='o', linestyle='-')
# axs[0].set_title('Positional Trajectory')
# axs[0].set_xlabel('X Position')
# axs[0].set_ylabel('Y Position')
# xy_array = np.column_stack((PX_VALUES, PY_VALUES))





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