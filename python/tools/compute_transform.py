import numpy as np
from scipy.spatial.transform import Rotation as R

# Define the nested transforms
# tf1: pos="0.54 0.0 0.0" euler="0 0 3.14159"
pos1 = np.array([0.54, 0.0, 0.0])
rot1 = R.from_euler('xyz', [0, 0, 3.14159], degrees=False)

# tf2: pos="0.017 0 0.552" euler="0 1.134 0"
pos2 = np.array([0.017, 0, 0.552])
rot2 = R.from_euler('xyz', [0, 1.134, 0], degrees=False)

# tf3: pos="0 0 0" (implicit) euler="1.57079 -1.57079 0"
pos3 = np.array([0, 0, 0])
rot3 = R.from_euler('xyz', [1.57079, -1.57079, 0], degrees=False)

# tf4: pos="0 0 0" (implicit) euler="3.1415926 0 0"
pos4 = np.array([0, 0, 0])
rot4 = R.from_euler('xyz', [3.1415926, 0, 0], degrees=False)

# Compute combined transform
# Start from tf1
combined_pos = pos1.copy()
combined_rot = rot1

# Apply tf2
combined_pos = combined_pos + combined_rot.apply(pos2)
combined_rot = combined_rot * rot2

# Apply tf3
combined_pos = combined_pos + combined_rot.apply(pos3)
combined_rot = combined_rot * rot3

# Apply tf4
combined_pos = combined_pos + combined_rot.apply(pos4)
combined_rot = combined_rot * rot4

# Convert back to Euler angles
combined_euler = combined_rot.as_euler('xyz', degrees=False)

# Also compute quaternion for reference
combined_quat = combined_rot.as_quat()  # [x, y, z, w]

print("Combined Transform:")
print(f"Position: {combined_pos}")
print(f"Euler (XYZ): {combined_euler}")
print(f"Quaternion [x,y,z,w]: {combined_quat}")
print()
print("For MJCF:")
print(f'pos="{combined_pos[0]:.6f} {combined_pos[1]:.6f} {combined_pos[2]:.6f}"')
print(f'euler="{combined_euler[0]:.6f} {combined_euler[1]:.6f} {combined_euler[2]:.6f}"')
print()
print("Or using quaternion (w,x,y,z order for MuJoCo):")
print(f'quat="{combined_quat[3]:.6f} {combined_quat[0]:.6f} {combined_quat[1]:.6f} {combined_quat[2]:.6f}"')
