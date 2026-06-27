import numpy as np
import matplotlib.pyplot as plt
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tcp_config_util import load_brush_tcp_xyz
from trajectory_paths import DEFAULTCONFIG_DIR

def load_offsets_from_json(json_path):
    """從 JSON 文件加載偏移量"""
    with open(json_path, 'r') as f:
        offsets = json.load(f)
    return offsets

class RobotPoseVisualizer:
    def __init__(self,
                 txt_path,
                 save_pose_path,
                 ee_pose_txt_path,     # ★ 新增：实际位姿 txt
                 T_ee_obj=None):

        self.txt_path = txt_path
        self.save_pose_path = save_pose_path

        # ★★★★★ 从 GetPose 的 txt 里读 ee_xyz / ee_abc ★★★★★
        self.ee_xyz, self.ee_abc = self.load_ee_pose_from_txt(ee_pose_txt_path)

        # print("[INFO] EE XYZ from GetPose:", self.ee_xyz)
        # print("[INFO] EE ABC from GetPose:", self.ee_abc)

        self.T_ee_obj = self.fix_homogeneous(T_ee_obj)

        self.T_base = np.eye(4)
        self.R_ee = self.euler_zyx_to_R(*self.ee_abc)
        self.T_base_ee = self.make_homogeneous(self.R_ee, self.ee_xyz)
        self.T_base_obj = self.T_base_ee @ self.T_ee_obj
        self.p_obj = self.T_base_obj[:3, 3]


    # ---------------------------
    # 基本旋转矩阵（绕 x,y,z）
    # ---------------------------
    @staticmethod
    def Rx(angle_deg):
        a = np.deg2rad(angle_deg)
        ca, sa = np.cos(a), np.sin(a)
        return np.array([[1, 0, 0],
                         [0, ca, -sa],
                         [0, sa, ca]])

    @staticmethod
    def Ry(angle_deg):
        a = np.deg2rad(angle_deg)
        ca, sa = np.cos(a), np.sin(a)
        return np.array([[ca, 0, sa],
                         [0, 1, 0],
                         [-sa, 0, ca]])

    @staticmethod
    def Rz(angle_deg):
        a = np.deg2rad(angle_deg)
        ca, sa = np.cos(a), np.sin(a)
        return np.array([[ca, -sa, 0],
                         [sa,  ca, 0],
                         [0,   0,  1]])

    # ---------------------------
    # ZYX 欧拉角 -> 旋转矩阵
    # ---------------------------
    @classmethod
    def euler_zyx_to_R(cls, rx, ry, rz):
        return cls.Rz(rz) @ cls.Ry(ry) @ cls.Rx(rx)

    # ---------------------------
    # 旋转矩阵 -> ZYX 欧拉角 (deg)
    # ---------------------------
    @staticmethod
    def R_to_euler_zyx(Rm):
        sy = np.sqrt(Rm[0,0]**2 + Rm[1,0]**2)
        if sy > 1e-6:
            rx = np.arctan2(Rm[2,1], Rm[2,2])
            ry = np.arctan2(-Rm[2,0], sy)
            rz = np.arctan2(Rm[1,0], Rm[0,0])
        else:
            rx = np.arctan2(-Rm[1,2], Rm[1,1])
            ry = np.arctan2(-Rm[2,0], sy)
            rz = 0.0
        return np.rad2deg([rx, ry, rz])

    # ---------------------------
    # 创建/修正齐次矩阵
    # ---------------------------
    @staticmethod
    def make_homogeneous(R, t):
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        return T

    @staticmethod
    def fix_homogeneous(T):
        T = T.copy()
        if np.allclose(T[3, :3], 0.0, atol=1e-8) and not np.isclose(T[3, 3], 1.0):
            T[3, 3] = 1.0
        elif not np.isclose(T[3,3], 1.0) and not np.isclose(T[3,3], 0.0):
            T = T / T[3,3]
        return T

    # ---------------------------
    # 绕指定点（基座系轴）旋转
    # ---------------------------
    @staticmethod
    def apply_rotation_about_point(T, R_axis_3x3, point3):
        Trot = np.eye(4)
        Trot[:3, :3] = R_axis_3x3
        Trot[:3, 3] = (np.eye(3) - R_axis_3x3) @ point3
        return Trot @ T

    # ---------------------------
    # 在基座系下做平移
    # ---------------------------
    @staticmethod
    def apply_translation_in_base(T, translation_vec_base):
        Tt = np.eye(4)
        Tt[:3, 3] = translation_vec_base
        return Tt @ T

    # ---------------------------
    # 绘制坐标系
    # ---------------------------
    @staticmethod
    def plot_frame(ax, T, length=100.0, label=None, linewidth=1.5, alpha=1.0):
        o = T[:3,3]
        R = T[:3,:3]
        x = R @ np.array([1.0,0,0]) * length
        y = R @ np.array([0,1.0,0]) * length
        z = R @ np.array([0,0,1.0]) * length
        ax.quiver(o[0], o[1], o[2], x[0], x[1], x[2], linewidth=linewidth,
                  arrow_length_ratio=0.15, color='r', alpha=alpha)
        ax.quiver(o[0], o[1], o[2], y[0], y[1], y[2], linewidth=linewidth,
                  arrow_length_ratio=0.15, color='g', alpha=alpha)
        ax.quiver(o[0], o[1], o[2], z[0], z[1], z[2], linewidth=linewidth,
                  arrow_length_ratio=0.15, color='b', alpha=alpha)
        if label:
            ax.text(o[0], o[1], o[2], label, fontsize=9)

    # 加载txt文件路径
    def load_ee_pose_from_txt(self,pose_txt_path):
        """
        从 GetPose 导出的 txt 中读取 ee_xyz 和 ee_abc
        txt 格式: x y z rx ry rz
        """
        with open(pose_txt_path, 'r') as f:
            line = f.readline().strip()
            vals = list(map(float, line.split()))

        if len(vals) != 6:
            raise ValueError("EE pose txt must contain 6 values: x y z rx ry rz")

        ee_xyz = np.array(vals[0:3], dtype=float)
        ee_abc = np.array(vals[3:6], dtype=float)
        return ee_xyz, ee_abc

    # ---------------------------
    # 主函数，运行整个流程
    # ---------------------------
    def run(self):
        with open(self.txt_path, 'r') as f:
            lines = [line.strip() for line in f if line.strip()]

        fig = plt.figure(figsize=(10,8))
        ax = fig.add_subplot(111, projection='3d')

        base_len = 150.0
        ee_len   = 120.0
        obj_len  = 80.0

        # 初始姿态绘制
        self.plot_frame(ax, self.T_base, length=base_len, label='Base (Init)', linewidth=2, alpha=1.0)
        self.plot_frame(ax, self.T_base_ee, length=ee_len, label='EE (Init)', alpha=1.0)
        self.plot_frame(ax, self.T_base_obj, length=obj_len, label='ToolEnd (Init)', alpha=1.0)

        with open(self.save_pose_path, 'w') as fout:
            # 先收集所有z值
            z_values = []
            for i, line in enumerate(lines):
                vals = list(map(float, line.split()))
                if len(vals) != 6:
                    continue
                translation_vec = np.array(vals[0:3])
                rx, ry, rz = vals[3:6]

                R_total = self.euler_zyx_to_R(rx, ry, rz)
                T_base_ee_rot = self.apply_rotation_about_point(self.T_base_ee, R_total, self.p_obj)
                T_base_obj_rot = T_base_ee_rot @ self.T_ee_obj
                T_base_ee_final = self.apply_translation_in_base(T_base_ee_rot, translation_vec)
                T_base_obj_final = T_base_ee_final @ self.T_ee_obj

                pos = T_base_ee_final[:3, 3]
                z_values.append(pos[2])

            # 计算z平均值
            z_avg = sum(z_values) / len(z_values) if z_values else 0

            # z_avg +=5

            # 重新处理数据并输出，使用z平均值
            for i, line in enumerate(lines):

                vals = list(map(float, line.split()))
                if len(vals) != 6:
                    continue
                translation_vec = np.array(vals[0:3])
                rx, ry, rz = vals[3:6]

                R_total = self.euler_zyx_to_R(rx, ry, rz)
                T_base_ee_rot = self.apply_rotation_about_point(self.T_base_ee, R_total, self.p_obj)
                T_base_obj_rot = T_base_ee_rot @ self.T_ee_obj
                T_base_ee_final = self.apply_translation_in_base(T_base_ee_rot, translation_vec)
                T_base_obj_final = T_base_ee_final @ self.T_ee_obj

                self.plot_frame(ax, T_base_ee_final, length=ee_len, label=f'EE step {i + 1}', alpha=0.6)
                self.plot_frame(ax, T_base_obj_final, length=obj_len, label=f'ToolEnd step {i + 1}', alpha=0.6)

                # 转成 xyzrxryrz（ZYX），使用z平均值
                pos = T_base_ee_final[:3, 3]
                abc = self.R_to_euler_zyx(T_base_ee_final[:3, :3])
                # 6.36355229, - 8.20525688 ,12.86837693
                # fout.write(f"{pos[0]+6.36355229:.6f} {pos[1]- 8.20525688:.6f} {z_avg+12.86837693:.6f} {abc[0]:.6f} {abc[1]:.6f} {abc[2]:.6f}\n")
                fout.write(f"{pos[0] :.6f} {pos[1] :.6f} {z_avg :.6f} {abc[0]:.6f} {abc[1]:.6f} {abc[2]:.6f}\n")
                # print(f"Step {i + 1}: {pos[0]:.6f} {pos[1]:.6f} {z_avg:.6f} {abc[0]:.6f} {abc[1]:.6f} {abc[2]:.6f}")
                # print(f"{{{pos[0]:.6f}, {pos[1]:.6f}, {z_avg:.6f}, {abc[0]:.6f},  {abc[1]:.6f},  {abc[2]:.6f}}}")
        # 绘制旋转中心
        # ax.scatter(self.p_obj[0], self.p_obj[1], self.p_obj[2], c='m', s=60, label='Rotation center (Tool End)')
        #
        # center = self.ee_xyz
        # pad = 300
        # ax.set_xlim(center[0]-pad, center[0]+pad)
        # ax.set_ylim(center[1]-pad, center[1]+pad)
        # ax.set_zlim(center[2]-pad, center[2]+pad)
        # ax.set_xlabel('X (mm)')
        # ax.set_ylabel('Y (mm)')
        # ax.set_zlabel('Z (mm)')
        # ax.view_init(elev=20, azim=-60)
        # ax.legend(fontsize=8, loc='upper left')
        # plt.tight_layout()
        # plt.show()

def get_subfolder_by_param(param):
    """
    根據參數返回對應的子文件夾名稱
    param: 0-6 對應的參數
    """
    folder_mapping = {
        0: "rightup",
        1: "leftup",
        2: "rightside",
        3: "leftside",
        4: "rightinside",
        5: "leftinside",
        6: "center",
        7: "sideahead"
    }
    return folder_mapping.get(param, "")

if __name__ == "__main__":
    subfolder = ""  # 默認空文件夾
    import sys
    if len(sys.argv) > 1:
        try:
            param = int(sys.argv[1])
            subfolder = get_subfolder_by_param(param)
            print(f"Using rotation parameter: {param}, subfolder: {subfolder}")
        except ValueError:
            print("參數必須是整數")
            sys.exit(1)


    txt_path = f"{DEFAULTCONFIG_DIR}/{subfolder}/transforms.txt"
    save_pose_path = f"{DEFAULTCONFIG_DIR}/{subfolder}/ee_poses.txt"
    ee_pose_txt_path = f"{DEFAULTCONFIG_DIR}/{subfolder}/current_pose_from_getpose.txt"

    tcp_x, tcp_y, tcp_z = load_brush_tcp_xyz()
    print("[TCP] 使用刷尖TCP (mm): x={:.6f}, y={:.6f}, z={:.6f}".format(tcp_x, tcp_y, tcp_z))
    T_ee_obj = np.array([
        [1, 0, 0, tcp_x],
        [0, 1, 0, tcp_y],
        [0, 0, 1, tcp_z],
        [0, 0, 0, 1.0]
    ], dtype=float)
    visualizer = RobotPoseVisualizer(
        txt_path,
        save_pose_path,
        ee_pose_txt_path,
        T_ee_obj
    )

    visualizer.run()

