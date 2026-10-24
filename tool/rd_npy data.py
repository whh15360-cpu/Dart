import numpy as np

# 1. 加载 .npy 文件
mtx = np.load('/home/hh/Dart/tool/camera_params/mtx.npy')
dist = np.load('/home/hh/Dart/tool/camera_params/dist.npy')

# 2. 打印查看内容
print("=== 相机内参矩阵 (mtx) ===")
print(mtx)

print("\n=== 畸变系数 (dist) ===")
print(dist)
