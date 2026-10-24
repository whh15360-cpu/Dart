import glob
import cv2
import numpy as np

# ==================== 参数配置 ====================
# 1. 棋盘格【内角点】的行列数（注意：是内角点的个数，即黑块交点数，而不是大方格数！）
# 请根据你实际打印的 PDF 棋盘格数出它的“内角点列数”和“内角点行数”
CHECKERBOARD = (6, 8)  # 示例：宽 6 个内角点，高 8 个内角点（请按实际修改）

# 2. 实际测量的方格边长，单位为毫米 (mm)
# 你前面实测的 2.8cm = 28.0mm
SQUARE_SIZE = 28.0
# ====================================================

# 准备世界坐标系中的 3D 点 (0,0,0), (1,0,0), (2,0,0) ...
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE  # 转换为实际物理尺寸（毫米）

objpoints = []  # 存储真实的 3D 点
imgpoints = []  # 存储图像平面检测到的 2D 像素点

# 读取保存在文件夹中的所有标定图片
images = glob.glob("/home/hh/Dart/za/captured_imgs/*.jpg")

if not images:
    print(
        "错误：没有在 'captured_imgs/' 目录下找到图片，请检查路径是否正确！"
    )
    exit()

print(f"共找到 {len(images)} 张图片，开始进行角点检测...")

gray = None
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 寻找棋盘格角点
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    # 如果找到了角点，进行亚像素级精确化并保存
    if ret == True:
        objpoints.append(objp)

        # 提高角点检测精度的优化准则
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # 可选：可视化展示角点提取过程
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow("Calibration", img)
        cv2.waitKey(100)
    else:
        print(f"警告：图片 {fname} 未能成功检测到棋盘格角点，已跳过。")

cv2.destroyAllWindows()

# 执行相机标定
print("\n正在计算相机内参和畸变系数...")
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

# 输出标定结果
print("\n========== 标定成功！==========")
print("相机内参矩阵 (Camera Matrix, K):\n", mtx)
print("\n畸变系数 (Distortion Coefficients, dist):\n", dist)

# 将内参和畸变系数保存到本地文件，方便后续在自瞄代码中直接加载调用
np.savez("camera_params.npz", mtx=mtx, dist=dist)
print(
    "\n参数已成功保存至当前目录下的 'camera_params.npz' 文件中！"
)