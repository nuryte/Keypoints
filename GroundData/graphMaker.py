
fig, axs = plt.subplots(1, 2, figsize=(12, 6))

# Plot positions as a trajectory in 2D space
optiOrigin = np.load("Timeline\\opti_arrayCircle.npy")
print(optiOrigin.shape)
optiXY = optiOrigin[:, 0, :2][0:500]
axs[0].plot(-1*np.array(optiXY[:, 1]),np.array(optiXY[:, 0]), marker='x', linestyle='-')
axs[0].plot(-1*np.array(PY_VALUES),np.array(PX_VALUES), marker='o', linestyle='-')
axs[0].set_title('Positional Trajectory')
axs[0].set_xlabel('X Position')
axs[0].set_ylabel('Y Position')
xy_array = np.column_stack((PX_VALUES, PY_VALUES))





min_dists = np.empty(xy_array.shape[0])
for i, point2 in enumerate(xy_array):
    # Compute the Euclidean distances to all points in array1
    distances = np.linalg.norm(optiXY - point2, axis=1)
    
    # Find the minimum distance for the current point in array2
    min_dists[i] = np.min(distances)
axs[1].plot(min_dists, label='Error', marker='o', linestyle='-')
#axs[1].plot(VY_VALUES, label='Y Velocity', marker='x', linestyle='-')
axs[1].set_title('Error In Distance')
axs[1].set_xlabel('Time')
axs[1].set_ylabel('Error')
axs[1].legend()