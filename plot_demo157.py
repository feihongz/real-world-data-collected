import numpy as np
import matplotlib.pyplot as plt

data = np.load('data/demo_157.npy', allow_pickle=True)
ee_pose = np.array([d['eepose'] for d in data])
action = np.array([d['action'] for d in data])

fig, axs = plt.subplots(2, 1, figsize=(12, 8))
for i in range(ee_pose.shape[1]):
    axs[0].plot(ee_pose[:, i], label=f'ee_pose_{i}')
axs[0].set_title('EE Pose')
axs[0].legend()

for i in range(action.shape[1]):
    axs[1].plot(action[:, i], label=f'action_{i}')
axs[1].set_title('Action')
axs[1].legend()

plt.tight_layout()
plt.savefig('demo_157_ee_pose_action.png')
print('Saved to demo_157_ee_pose_action.png') 