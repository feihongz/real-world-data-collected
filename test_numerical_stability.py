import numpy as np
from scipy.spatial.transform import Rotation as R

def SE3_inv(SE3):
    """
    计算SE3变换矩阵的逆，使用数值稳定的方法
    """
    R_mat = SE3[:3, :3]
    t = SE3[:3, 3]
    
    # 检查旋转矩阵的条件数，如果接近奇异则使用更稳定的方法
    cond = np.linalg.cond(R_mat)
    if cond > 1e12:  # 条件数过大，使用SVD分解
        U, s, Vt = np.linalg.svd(R_mat)
        # 过滤掉过小的奇异值
        s_inv = np.where(s > 1e-12, 1.0/s, 0.0)
        R_inv = Vt.T @ np.diag(s_inv) @ U.T
    else:
        # 直接求逆，更高效
        R_inv = np.linalg.inv(R_mat)
    
    # 确保旋转矩阵是正交的（修正数值误差）
    U, _, Vt = np.linalg.svd(R_inv)
    R_inv = U @ Vt
    
    # 计算平移部分
    t_inv = -R_inv @ t
    
    # 返回逆矩阵
    SE3_inv = np.eye(4, dtype=np.float32)
    SE3_inv[:3, :3] = R_inv
    SE3_inv[:3, 3] = t_inv
    return SE3_inv

def test_SE3_inv_stability():
    """测试SE3_inv函数的数值稳定性"""
    print("=== 测试SE3_inv函数的数值稳定性 ===")
    
    # 测试1: 正常的SE3变换
    print("\n1. 测试正常SE3变换:")
    SE3 = np.eye(4)
    SE3[:3, :3] = R.from_euler('xyz', [30, 45, 60], degrees=True).as_matrix()
    SE3[:3, 3] = [0.1, 0.2, 0.3]
    
    SE3_inv_result = SE3_inv(SE3)
    SE3_reconstructed = SE3 @ SE3_inv_result
    
    print(f"原始SE3:\n{SE3}")
    print(f"SE3逆:\n{SE3_inv_result}")
    print(f"SE3 @ SE3_inv (应该接近单位矩阵):\n{SE3_reconstructed}")
    print(f"重建误差: {np.linalg.norm(SE3_reconstructed - np.eye(4)):.2e}")
    
    # 测试2: 接近奇异的旋转矩阵
    print("\n2. 测试接近奇异的旋转矩阵:")
    # 创建一个接近奇异的旋转矩阵
    SE3_singular = np.eye(4)
    SE3_singular[:3, :3] = R.from_euler('xyz', [0, 0, 179.999], degrees=True).as_matrix()
    SE3_singular[:3, 3] = [0.1, 0.2, 0.3]
    
    SE3_inv_singular = SE3_inv(SE3_singular)
    SE3_reconstructed_singular = SE3_singular @ SE3_inv_singular
    
    print(f"接近奇异的SE3:\n{SE3_singular}")
    print(f"SE3逆:\n{SE3_inv_singular}")
    print(f"重建误差: {np.linalg.norm(SE3_reconstructed_singular - np.eye(4)):.2e}")
    
    # 测试3: 添加噪声的SE3变换
    print("\n3. 测试带噪声的SE3变换:")
    SE3_noisy = SE3 + np.random.normal(0, 1e-6, (4, 4))  # 添加小噪声
    SE3_noisy[3, 3] = 1.0  # 确保最后一行为[0,0,0,1]
    
    SE3_inv_noisy = SE3_inv(SE3_noisy)
    SE3_reconstructed_noisy = SE3_noisy @ SE3_inv_noisy
    
    print(f"带噪声的SE3:\n{SE3_noisy}")
    print(f"SE3逆:\n{SE3_inv_noisy}")
    print(f"重建误差: {np.linalg.norm(SE3_reconstructed_noisy - np.eye(4)):.2e}")

def test_coordinate_transform_stability():
    """测试坐标变换的数值稳定性"""
    print("\n=== 测试坐标变换的数值稳定性 ===")
    
    # 模拟AVP数据
    hand_init = np.eye(4)
    hand_init[:3, :3] = R.from_euler('xyz', [10, 20, 30], degrees=True).as_matrix()
    hand_init[:3, 3] = [0.5, 0.3, 0.8]
    
    # 模拟当前手部位姿（相对于初始位姿有小的变化）
    hand_pose = np.eye(4)
    hand_pose[:3, :3] = R.from_euler('xyz', [12, 22, 32], degrees=True).as_matrix()
    hand_pose[:3, 3] = [0.52, 0.32, 0.82]
    
    # 坐标变换矩阵
    X_VR2Robot = np.array([
        [ 1,  0,  0,  0],
        [ 0,  1,  0,  0],
        [ 0,  0,  1,  0],
        [ 0,  0,  0,  1],
    ])
    
    print(f"初始手部位姿:\n{hand_init}")
    print(f"当前手部位姿:\n{hand_pose}")
    
    # 计算相对变换
    hand_transform_VR = np.eye(4, dtype=np.float32)
    hand_transform_VR[:3, :3] = hand_pose[:3, :3] @ hand_init[:3, :3].T
    hand_transform_VR[:3, 3] = hand_pose[:3, 3] - hand_init[:3, 3]
    
    print(f"相对变换:\n{hand_transform_VR}")
    
    # 计算机器人坐标系下的变换
    hand_transform_robot = X_VR2Robot @ hand_transform_VR @ SE3_inv(X_VR2Robot)
    
    print(f"机器人坐标系下的变换:\n{hand_transform_robot}")
    
    # 检查变换的合理性
    pos_delta = hand_transform_VR[:3, 3]
    pos_delta_norm = np.linalg.norm(pos_delta)
    print(f"位置变化: {pos_delta}, 大小: {pos_delta_norm:.6f}")
    
    if pos_delta_norm < 1e-3:
        print("WARNING: 位置变化过小，可能导致数值不稳定")

if __name__ == "__main__":
    test_SE3_inv_stability()
    test_coordinate_transform_stability() 