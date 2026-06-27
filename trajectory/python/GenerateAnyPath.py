# -*- coding: utf-8 -*-
# 其他非中间区域的刷牙轨迹构建代码
import os
import cv2
import copy
import time
from scipy.interpolate import splprep, splev
from scipy.spatial.transform import Rotation as R


import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import numpy as np

import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trajectory_paths import DEFAULTCONFIG_DIR, HAND_EYE_DATA_DIR


# ===============================
# 读取轨迹点
# ===============================
def load_points(path):
    pts = []
    with open(path, 'r') as f:
        for line in f:
            vals = line.strip().split()
            if len(vals) >= 3:
                pts.append([float(vals[0]), float(vals[1]), float(vals[2])])
    return np.array(pts)


# ===============================
# 主类
# ===============================
class BrushBuilderGUI:

    def __init__(self, points, ply_path=None,subfolder=None):
        self.points = points
        self.n = len(points)
        self.subfolder =subfolder

        self.current_index = 0
        self.base_segment = [0]  # 当前基础段
        self.segments_info = []  # 所有段信息

        # ===== GUI 初始化 =====
        self.app = gui.Application.instance
        self.app.initialize()
        self.window = self.app.create_window("Brush Trajectory Builder", 1400, 800)

        # 3D 场景
        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(self.window.renderer)
        self.window.add_child(self.scene)

        # 左侧面板
        self.panel = gui.Vert(10, gui.Margins(10, 10, 10, 10))
        self.window.add_child(self.panel)

        self.create_scene(ply_path)
        self.create_buttons()
        self.window.set_on_layout(self._on_layout)

    # ==========================
    # 场景创建
    # ==========================
    def create_scene(self, ply_path):
        mat = rendering.MaterialRecord()
        mat.shader = "defaultLit"
        mat.point_size = 10

        # 背景点云
        if ply_path is not None:
            background = o3d.io.read_point_cloud(ply_path)
            self.scene.scene.add_geometry("background", background, mat)

        # 轨迹点
        self.pcd = o3d.geometry.PointCloud()
        self.pcd.points = o3d.utility.Vector3dVector(self.points)
        colors = np.tile([0.7, 0.7, 0.7], (self.n, 1))
        self.pcd.colors = o3d.utility.Vector3dVector(colors)
        self.scene.scene.add_geometry("trajectory", self.pcd, mat)

        # 光标球
        self.cursor = o3d.geometry.TriangleMesh.create_sphere(radius=0.2)
        self.cursor.paint_uniform_color([0, 1, 0])
        self.scene.scene.add_geometry("cursor", self.cursor, mat)

        bounds = self.pcd.get_axis_aligned_bounding_box()
        self.scene.setup_camera(60, bounds, bounds.get_center())

        self.update_cursor()
        self.update_colors()

    # ==========================
    # 创建按钮
    # ==========================
    def create_buttons(self):
        self.panel.add_child(gui.Label("GenerateAnyPath"))

        btn_next = gui.Button("Next point")
        btn_next.set_on_clicked(lambda: self.next_point())
        self.panel.add_child(btn_next)

        btn_lock = gui.Button("Lock Segment")
        btn_lock.set_on_clicked(lambda: self.lock_segment())
        self.panel.add_child(btn_lock)

        # 左侧面板添加重复次数输入
        self.panel.add_child(gui.Label("Repeat Count (odd only):"))

        # 指定类型 INT
        self.num_repeat = gui.NumberEdit(gui.NumberEdit.INT)
        self.num_repeat.set_value(1)
        self.num_repeat.set_limits(1, 99)
        self.panel.add_child(self.num_repeat)

        btn_repeat = gui.Button("Repeat Segment")
        btn_repeat.set_on_clicked(lambda: self.repeat_segment_panel())
        self.panel.add_child(btn_repeat)

        btn_print = gui.Button("Print Segments")
        btn_print.set_on_clicked(lambda: self.print_result())
        self.panel.add_child(btn_print)

        btn_save = gui.Button("Save All Segments")
        btn_save.set_on_clicked(lambda: self.save_all_segments())
        self.panel.add_child(btn_save)

        btn_clear = gui.Button("Clear All")
        btn_clear.set_on_clicked(lambda: self.clear_all())
        self.panel.add_child(btn_clear)

    # ==========================
    # 布局
    # ==========================
    def _on_layout(self, layout_context):
        r = self.window.content_rect
        panel_width = 250
        self.panel.frame = gui.Rect(r.x, r.y, panel_width, r.height)
        self.scene.frame = gui.Rect(r.x + panel_width, r.y, r.width - panel_width, r.height)


    # ==========================
    # 更新光标（只针对 points）
    # ==========================
    def update_cursor(self):
        # 移动光标到当前点
        self.cursor.translate(
            self.points[self.current_index] - self.cursor.get_center(),
            relative=False
        )
        # 光标颜色/material保持不变
        self.scene.scene.modify_geometry_material("cursor", rendering.MaterialRecord())

    # ==========================
    # 更新 self.points 颜色（只针对 points，不影响背景）
    # ==========================
    def update_colors(self):
        # 默认灰色
        colors = np.tile([0, 0, 1], (self.n, 1))  # 蓝色

        # 标记基础段为红色
        for idx in self.base_segment:
            if 0 <= idx < self.n:
                colors[idx] = [1, 0, 0]  # 红色

        # 更新 PointCloud 颜色
        self.pcd.colors = o3d.utility.Vector3dVector(colors)

        # **刷新几何体，确保界面显示更新**
        self.scene.scene.remove_geometry("trajectory")  # 先移除
        mat = rendering.MaterialRecord()
        mat.shader = "defaultLit"
        mat.point_size = 10
        self.scene.scene.add_geometry("trajectory", self.pcd, mat)

    # ==========================
    # 添加点
    # ==========================
    def next_point(self):
        if self.current_index not in self.base_segment:
            self.base_segment.append(self.current_index)
        if self.current_index < self.n - 1:
            self.current_index += 1
        if self.current_index not in self.base_segment:
            self.base_segment.append(self.current_index)
        self.update_cursor()
        self.update_colors()


    # ==========================
    # 查看当前段
    # ==========================
    def lock_segment(self):
        print("\n当前基础段:")
        print(self.base_segment)


    # 执行逻辑
    def repeat_segment_panel(self):
        if len(self.base_segment) < 2:
            print("Current base segment too short")
            return

        try:
            # 👉 这里正确读取数字控件的值
            times = int(self.num_repeat.int_value)
        except Exception as e:
            print("Invalid input:", e)
            return

        if times % 2 == 0:
            times += 1
            print("Automatically adjusted to odd:", times)

        base = self.base_segment.copy()
        segment_result = []

        for i in range(times):
            seg = base if i % 2 == 0 else base[::-1]
            if len(segment_result) > 0:
                seg = seg[1:]
            segment_result.extend(seg)

        self.segments_info.append({
            "base": base.copy(),
            "repeat": times,
            "expanded": segment_result.copy(),
            "start": base[0],
            "end": base[-1]
        })

        print("Segment generated, length:", len(segment_result))

        self.base_segment = [base[-1]]
        self.current_index = base[-1]

        self.update_colors()

    # ==========================
    # 打印结果
    # ==========================
    def print_result(self):
        print("\n==============================")
        print("========= 分段信息 =========")
        for i, seg in enumerate(self.segments_info):
            base = seg["base"]
            times = seg["repeat"]
            print(f"\n--- 第 {i} 段 ---")
            print("基础段:", base)
            print("重复次数:", times)
            print("起点:", seg["start"], "终点:", seg["end"])
            print("展开后总长度:", len(seg["expanded"]))
            print("往复拆分：")
            for t in range(times):
                sub_seg = base if t % 2 == 0 else base[::-1]
                direction = "往" if t % 2 == 0 else "返"
                print(f"  第 {t} 次 ({direction}) :", sub_seg)
        print("\n==============================")

    # ==========================
    # 保存txt
    # ==========================
    def save_all_segments(self):
        save_path = f"{DEFAULTCONFIG_DIR}/{self.subfolder}/all_segments.txt"
        with open(save_path, "w") as f:
            for seg in self.segments_info:
                base = seg["base"]
                times = seg["repeat"]
                for t in range(times):
                    sub_seg = base if t % 2 == 0 else base[::-1]
                    line = " ".join(str(idx) for idx in sub_seg)
                    f.write(line + "\n")
        print(f"已保存到 {save_path}")

    # ==========================
    # 清空
    # ==========================
    def clear_all(self):
        self.base_segment = [self.current_index]
        self.segments_info = []
        self.update_colors()
        print("已清空")

    # ==========================
    # 运行
    # ==========================
    def run(self):
        self.app.run()


class SupportPointSelector:
    def __init__(self, points, ply_path=None, subfolder="default"):
        self.points = points
        self.n = len(points)
        self.subfolder = subfolder
        self.current_index = 0
        self.selected_ids = []

        self.app = gui.Application.instance
        self.app.initialize()
        self.window = self.app.create_window("Support Point Selector (ID Sorted)", 1200, 800)

        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(self.window.renderer)
        self.window.add_child(self.scene)

        self.panel = gui.Vert(10, gui.Margins(10, 10, 10, 10))
        self.window.add_child(self.panel)

        # FIX: Setup controls BEFORE setup_scene so that labels exist
        # when update_colors() is called during initialization.
        self.setup_controls()
        self.setup_scene(ply_path)

        self.window.set_on_layout(self._on_layout)

    def setup_scene(self, ply_path):
        mat = rendering.MaterialRecord()
        mat.shader = "defaultLit"
        mat.point_size = 8

        if ply_path and os.path.exists(ply_path):
            background = o3d.io.read_point_cloud(ply_path)
            self.scene.scene.add_geometry("background", background, mat)

        self.pcd = o3d.geometry.PointCloud()
        self.pcd.points = o3d.utility.Vector3dVector(self.points)

        # This calls update_colors which requires self.count_label
        self.update_colors()

        self.cursor = o3d.geometry.TriangleMesh.create_sphere(radius=0.3)
        self.cursor.paint_uniform_color([0, 1, 0])
        self.scene.scene.add_geometry("cursor", self.cursor, mat)

        bounds = self.pcd.get_axis_aligned_bounding_box()
        self.scene.setup_camera(60, bounds, bounds.get_center())
        self.update_cursor()

    def setup_controls(self):
        self.panel.add_child(gui.Label("--- ID Collection Mode ---"))
        self.idx_label = gui.Label(f"Preview ID: {self.current_index}")
        self.panel.add_child(self.idx_label)

        btn_next = gui.Button("Forward (+) ")
        btn_next.set_on_clicked(self.next_preview)
        self.panel.add_child(btn_next)

        btn_prev = gui.Button("Backward (-) ")
        btn_prev.set_on_clicked(self.prev_preview)
        self.panel.add_child(btn_prev)

        self.panel.add_fixed(10)

        btn_add = gui.Button("Select / Lock")
        btn_add.background_color = gui.Color(0.2, 0.5, 0.8)
        btn_add.set_on_clicked(self.add_selection)
        self.panel.add_child(btn_add)

        btn_remove = gui.Button("Undo Last Selection")
        btn_remove.set_on_clicked(self.remove_last)
        self.panel.add_child(btn_remove)

        self.panel.add_fixed(20)
        self.count_label = gui.Label("Selected: 0")
        self.panel.add_child(self.count_label)

        btn_save = gui.Button("Save (Sorted by ID)")
        btn_save.set_on_clicked(self.save_ids)
        self.panel.add_child(btn_save)

        btn_clear = gui.Button("Reset All")
        btn_clear.set_on_clicked(self.clear_all)
        self.panel.add_child(btn_clear)

    def _on_layout(self, layout_context):
        r = self.window.content_rect
        panel_width = 220
        self.panel.frame = gui.Rect(r.x, r.y, panel_width, r.height)
        self.scene.frame = gui.Rect(r.x + panel_width, r.y, r.width - panel_width, r.height)

    def update_cursor(self):
        target_pos = self.points[self.current_index]
        current_center = self.cursor.get_center()
        self.cursor.translate(target_pos - current_center)

        self.scene.scene.remove_geometry("cursor")
        mat = rendering.MaterialRecord()
        mat.shader = "defaultLit"
        self.scene.scene.add_geometry("cursor", self.cursor, mat)
        self.idx_label.text = f"Preview ID: {self.current_index}"

    def update_colors(self):
        colors = np.tile([0.4, 0.4, 0.4], (self.n, 1))
        for idx in self.selected_ids:
            colors[idx] = [1, 0, 0]
        self.pcd.colors = o3d.utility.Vector3dVector(colors)

        self.scene.scene.remove_geometry("trajectory")
        mat = rendering.MaterialRecord()
        mat.shader = "defaultLit"
        mat.point_size = 10
        self.scene.scene.add_geometry("trajectory", self.pcd, mat)

        # Now self.count_label is guaranteed to exist
        self.count_label.text = f"Selected: {len(self.selected_ids)}"

    def next_preview(self):
        if self.current_index < self.n - 1:
            self.current_index += 1
            self.update_cursor()

    def prev_preview(self):
        if self.current_index > 0:
            self.current_index -= 1
            self.update_cursor()

    def add_selection(self):
        if self.current_index not in self.selected_ids:
            self.selected_ids.append(self.current_index)
            self.update_colors()

    def remove_last(self):
        if self.selected_ids:
            self.selected_ids.pop()
            self.update_colors()

    def save_ids(self):
        if not self.selected_ids: return
        sorted_list = sorted(self.selected_ids)
        folder_path = f"{DEFAULTCONFIG_DIR}/{self.subfolder}"
        os.makedirs(folder_path, exist_ok=True)
        save_path = os.path.join(folder_path, "support_points.txt")
        with open(save_path, "w") as f:
            f.write("\n".join(str(idx) for idx in sorted_list))
        print(f"Saved (Sorted by ID): {save_path}")

    def clear_all(self):
        self.selected_ids = []
        self.update_colors()

    def run(self):
        self.app.run()


def rotate_point_cloud_with_selected_axis(ply_path, save_prefix="rotated", rotation_param=None):
    """
    1. Load point cloud.
    2. Select exactly 2 points (Shift + Left click).
    3. Ask user for rotation command or use parameter:
         1 → +45°
         2 → -45°
         3 → no rotation
    4. Save the resulting point cloud.
    """
    # Load point cloud
    pcd = o3d.io.read_point_cloud(ply_path)
    if len(pcd.points) == 0:
        print("Error: empty point cloud.")
        return

    print("load ids")

    picked_ids = np.loadtxt("D:/UsmileProject/hand_eye_calibration/picked_rotation_directionid.txt", dtype=np.int32)
    pts = np.asarray(pcd.points)
    p1, p2 = pts[picked_ids[0]], pts[picked_ids[1]]

    picked_idsrotate = np.loadtxt("D:/UsmileProject/hand_eye_calibration/mesh_direction_ids.txt", dtype=np.int32)
    #
    rotatepoint = pts[picked_idsrotate[2]]
    # Axis
    axis = p2 - p1
    norm = np.linalg.norm(axis)
    if norm < 1e-6:
        print("Error: invalid rotation axis.")
        return
    axis = axis / norm

    # Determine rotation angle based on parameter or keyboard input
    if rotation_param is not None:
        # 根據參數決定旋轉角度
        if rotation_param == 0 or rotation_param == 1:
            angle_rad = 0.0
            print(f"Parameter {rotation_param} → no rotation")
        elif rotation_param == 2 or rotation_param == 5 or rotation_param == 6 or rotation_param == 7:
            angle_rad = np.deg2rad(-45.0)
            print(f"Parameter {rotation_param} → -45° rotation")
        elif rotation_param == 3 or rotation_param == 4 :
            angle_rad = np.deg2rad(45.0)
            print(f"Parameter {rotation_param} → +45° rotation")
        else:
            print(f"Unknown parameter {rotation_param}, defaulting to no rotation")
            angle_rad = 0.0
    else:
        # Keyboard input
        print("\nRotation options:")
        print("  1 → rotate +45°")
        print("  2 → rotate -45°")
        print("  3 → no rotation")

        cmd = input("Enter rotation command (1/2/3): ").strip()

        if cmd not in ["1", "2", "3"]:
            print("Invalid command.")
            return

        angle_deg = 45.0
        if cmd == "1":
            angle_rad = np.deg2rad(angle_deg)
        elif cmd == "2":
            angle_rad = np.deg2rad(-angle_deg)
        else:  # cmd == "3"
            angle_rad = 0.0

    # Rotation function
    def rotate_pcd(pc, angle):
        rot_mat = R.from_rotvec(axis * angle).as_matrix()
        pts = np.asarray(pc.points)
        print(f"-45° rotation")
        # rotated = (pts - p1) @ rot_mat.T + p1
        rotated = (pts - rotatepoint) @ rot_mat.T + rotatepoint

        new_pc = o3d.geometry.PointCloud()
        new_pc.points = o3d.utility.Vector3dVector(rotated)
        if pc.has_colors():
            new_pc.colors = pc.colors
        return new_pc

    # Apply rotation
    pcd_rot = rotate_pcd(pcd, angle_rad)


    return pcd_rot


class BrushPathSaver:
    def __init__(self, XX=7, YY=11, L_horizontal=4, L_vertical=2):
        self.XX = XX
        self.YY = YY
        self.L_horizontal = L_horizontal
        self.L_vertical = L_vertical
        self.mtx = np.array([[4510.87, 0, 1238.18], [0, 4511.26, 1016.29], [0, 0, 1]], dtype=np.float32)
        self.dist = np.array([-0.0606931, 0.329977, -0.00188508, -0.00107541, -1.69546])

    def select_box_from_pointcloud(self, pcd, initial_size=10.0, step=1.0, angle_step=2.0):
        print("请在弹出的窗口中用 Shift+左键选择包围盒中心点，按 Q 完成选择。")
        vis_pick = o3d.visualization.VisualizerWithEditing()
        vis_pick.create_window(window_name="选择包围盒中心点")
        vis_pick.add_geometry(pcd)
        vis_pick.run()
        vis_pick.destroy_window()

        picked_points = vis_pick.get_picked_points()
        if not picked_points:
            print("未选择任何点，使用点云中心作为包围盒中心。")
            bbox_center = np.asarray(pcd.get_center())
        else:
            bbox_center = np.asarray(pcd.points)[picked_points[0]]

        print(f"包围盒中心点为: {bbox_center}")

        extent = np.array([initial_size] * 3)
        obb = o3d.geometry.OrientedBoundingBox(bbox_center, np.eye(3), extent)
        obb.color = (1, 0, 0)

        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(window_name="调整矩形框，按 Q 确认")

        vis.add_geometry(pcd)
        vis.add_geometry(obb)

        def update_obb():
            vis.update_geometry(obb)

        # --- 缩放 ---
        def scale(dx=0, dy=0, dz=0):
            new_extent = obb.extent + np.array([dx, dy, dz])
            obb.extent = new_extent
            update_obb()

        # --- 旋转 ---
        def rotate(axis, direction=1):
            R = self.get_rotation_matrix(axis, angle_step * direction)
            obb.R = R @ obb.R
            update_obb()

        # --- 平移 ---
        def translate(dx=0, dy=0, dz=0):
            translation = np.array([dx, dy, dz])
            obb.center = obb.center + translation
            update_obb()

        # 平移绑定：WASDQE
        vis.register_key_callback(ord("D"), lambda vis: translate(dx=step))  # +X
        vis.register_key_callback(ord("A"), lambda vis: translate(dx=-step))  # -X
        vis.register_key_callback(ord("E"), lambda vis: translate(dy=step))  # +Y
        vis.register_key_callback(ord("Q"), lambda vis: translate(dy=-step))  # -Y
        vis.register_key_callback(ord("W"), lambda vis: translate(dz=step))  # +Z
        vis.register_key_callback(ord("S"), lambda vis: translate(dz=-step))  # -Z

        # 缩放绑定：Z/X/C/V/B/N
        vis.register_key_callback(ord("Z"), lambda vis: scale(dx=step))
        vis.register_key_callback(ord("X"), lambda vis: scale(dx=-step))
        vis.register_key_callback(ord("C"), lambda vis: scale(dy=step))
        vis.register_key_callback(ord("V"), lambda vis: scale(dy=-step))
        vis.register_key_callback(ord("B"), lambda vis: scale(dz=step))
        vis.register_key_callback(ord("N"), lambda vis: scale(dz=-step))

        # 旋转绑定：J/L/I/K/U/O
        vis.register_key_callback(ord("J"), lambda vis: rotate('z', direction=1))  # yaw +
        vis.register_key_callback(ord("L"), lambda vis: rotate('z', direction=-1))  # yaw -
        vis.register_key_callback(ord("I"), lambda vis: rotate('y', direction=1))  # pitch +
        vis.register_key_callback(ord("K"), lambda vis: rotate('y', direction=-1))  # pitch -
        vis.register_key_callback(ord("U"), lambda vis: rotate('x', direction=1))  # roll +
        vis.register_key_callback(ord("O"), lambda vis: rotate('x', direction=-1))  # roll -

        vis.run()
        vis.destroy_window()

        print(obb)

        cropped = pcd.crop(obb)
        return cropped

    def select_box_from_pointcloudbrushandtooth(self, pcd):

        bbox_center = np.array([5.29886, 37.382, -155.51])  # 中心点
        extent = np.array([54, 46, 30])  # 宽、长、高
        rotation = np.eye(3)  # 如果没有旋转，保持单位矩阵

        bbox_centertooth = np.array([5.79912, -11.1134, -169.507])  # 中心点
        # extenttooth = np.array([38, 29, 16])  # 宽、长、高
        extenttooth = np.array([103, 100, 63])  # 宽、长、高
        rotationtooth = np.eye(3)  # 如果没有旋转，保持单位矩阵

        # 创建固定 OBB
        obb = o3d.geometry.OrientedBoundingBox(bbox_center, rotation, extent)
        obb.color = (1, 0, 0)

        obbtooth = o3d.geometry.OrientedBoundingBox(bbox_centertooth, rotationtooth, extenttooth)
        obbtooth.color = (0, 1, 0)

        # 裁剪并返回结果
        cropped = pcd.crop(obb)
        croppedtooth = pcd.crop(obbtooth)
        return cropped ,croppedtooth

    def load_board2cam_results_from_txt(self,path):
        # 构造文件路径
        rvecs_path = os.path.join(path, "rvecs.txt")
        tvecs_path = os.path.join(path, "tvecs.txt")
        indices_path = os.path.join(path, "valid_indices.txt")

        # 加载数据
        rvecs_arr = np.loadtxt(rvecs_path).reshape(-1, 3)
        tvecs_arr = np.loadtxt(tvecs_path).reshape(-1, 3)
        valid_indices = np.loadtxt(indices_path, dtype=int).tolist()

        # 转为列表形式，每个元素是 (3,1) 的 ndarray
        rvecs = [vec.reshape(3, 1) for vec in rvecs_arr]
        tvecs = [vec.reshape(3, 1) for vec in tvecs_arr]

        return rvecs, tvecs, valid_indices

    def get_board2cam_v2(self, image_dir, objp, w=7, h=11, show_result=True):
        import re
        rvecs, tvecs, valid_indices = [], [], []
        files = sorted(
            [f for f in os.listdir(image_dir) if f.endswith('.bmp')],
            key=lambda x: int(re.search(r'(\d+)', x).group(1)) if re.search(r'(\d+)', x) else -1
        )

        max_attempts = 18  # 最多尝试提升亮度5次
        brightness_step = 10  # 每次增加的亮度值

        for idx, fname in enumerate(files):
            img_path = os.path.join(image_dir, fname)
            print(f'处理图片: {img_path}')

            raw_gray = 250 - cv2.imread(img_path, 0)  # 原始反转灰度图
            gray = raw_gray.copy()

            ret, centers = cv2.findCirclesGrid(gray, (w, h), flags=cv2.CALIB_CB_ASYMMETRIC_GRID)
            attempt = 0

            # 若初次未检测成功则尝试提升亮度
            while not ret and attempt < max_attempts:
                attempt += 1
                bright_gray = np.clip(gray + attempt * brightness_step, 0, 255).astype(np.uint8)
                bright_gray =cv2.convertScaleAbs(bright_gray,alpha=1.5,beta=20)
                ret, centers = cv2.findCirclesGrid(bright_gray, (w, h), flags=cv2.CALIB_CB_ASYMMETRIC_GRID)
                if ret:
                    gray = bright_gray  # 用增强后的图继续处理
                    print(f"{fname} 提升亮度后检测成功（第 {attempt} 次尝试）")
                    break

            if ret and centers.shape[0] == objp.shape[0]:
                if show_result:
                    show = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                    cv2.drawChessboardCorners(show, (w, h), centers, ret)
                    cv2.imshow(f"Detected {idx}", show)
                    cv2.waitKey(300)
                    cv2.destroyWindow(f"Detected {idx}")
                success, rvec, tvec = cv2.solvePnP(objp, centers, self.mtx, self.dist)
                if success:
                    rvecs.append(rvec)
                    tvecs.append(tvec)
                    valid_indices.append(idx)
                    print(f"{fname} 检测成功")
            else:
                print(f"{fname} 检测失败")
        print(f"有效样本数: {len(rvecs)}")
        return rvecs, tvecs, valid_indices

    def manual_select_and_detect_on_resized(self, original_gray, pattern_size):
        # 将原图缩放至 640x480
        h, w = original_gray.shape
        resized_gray = cv2.resize(original_gray, (640, 480))
        clone = cv2.cvtColor(resized_gray.copy(), cv2.COLOR_GRAY2BGR)
        selected_points = []

        def click_event(event, x, y, flags, param):
            nonlocal clone  # 提前声明
            if event == cv2.EVENT_LBUTTONDOWN:
                selected_points.append((x, y))
                cv2.circle(clone, (x, y), 5, (0, 0, 255), -1)
            elif event == cv2.EVENT_RBUTTONDOWN and selected_points:
                selected_points.pop()
                clone = cv2.cvtColor(resized_gray.copy(), cv2.COLOR_GRAY2BGR)
                for pt in selected_points:
                    cv2.circle(clone, pt, 5, (0, 0, 255), -1)

        cv2.namedWindow("Manual Select")
        cv2.setMouseCallback("Manual Select", click_event)

        print("请点击标定点，右键撤销，按 Q 确认")

        while True:
            cv2.imshow("Manual Select", clone)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

        cv2.destroyWindow("Manual Select")

        if len(selected_points) != pattern_size[0] * pattern_size[1]:
            print("选择的点数不匹配")
            return None

        # 从 640x480 映射回原图大小
        scale_x = w / 640
        scale_y = h / 480
        mapped_points = np.array([[[x * scale_x, y * scale_y]] for (x, y) in selected_points], dtype=np.float32)

        return mapped_points


    def get_board2cam_v3(self, image_dir, objp, w=7, h=11, show_result=True):
        import re

        rvecs, tvecs, valid_indices = [], [], []
        files = sorted(
            [f for f in os.listdir(image_dir) if f.endswith('.bmp')],
            key=lambda x: int(re.search(r'(\d+)', x).group(1)) if re.search(r'(\d+)', x) else -1
        )

        max_attempts = 18
        brightness_step = 10

        for idx, fname in enumerate(files):
            img_path = os.path.join(image_dir, fname)
            print(f'处理图片: {img_path}')

            raw_gray = 250 - cv2.imread(img_path, 0)
            gray = raw_gray.copy()

            ret, centers = cv2.findCirclesGrid(gray, (w, h), flags=cv2.CALIB_CB_ASYMMETRIC_GRID)
            attempt = 0

            # 亮度增强
            while not ret and attempt < max_attempts:
                attempt += 1
                bright_gray = np.clip(gray + attempt * brightness_step, 0, 255).astype(np.uint8)
                bright_gray = cv2.convertScaleAbs(bright_gray, alpha=1.5, beta=20)
                ret, centers = cv2.findCirclesGrid(bright_gray, (w, h), flags=cv2.CALIB_CB_ASYMMETRIC_GRID)
                if ret:
                    gray = bright_gray
                    print(f"{fname} 提升亮度后检测成功（第 {attempt} 次尝试）")
                    break

            # 如果还失败，手动标注
            if not ret:
                print(f"{fname} 自动检测失败，进入人工标注")
                centers = self.manual_select_and_detect_on_resized(raw_gray, (w, h))
                if centers is not None and centers.shape[0] == objp.shape[0]:
                    ret = True

            if ret and centers.shape[0] == objp.shape[0]:
                if show_result:
                    show = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                    cv2.drawChessboardCorners(show, (w, h), centers, ret)
                    cv2.imshow(f"Detected {idx}", show)
                    cv2.waitKey(300)
                    cv2.destroyWindow(f"Detected {idx}")
                success, rvec, tvec = cv2.solvePnP(objp, centers, self.mtx, self.dist)
                if success:
                    rvecs.append(rvec)
                    tvecs.append(tvec)
                    valid_indices.append(idx)
                    print(f"{fname} 检测成功")
            else:
                print(f"{fname} 检测失败")

        print(f"有效样本数: {len(rvecs)}")

        # === 保存结果到 txt 文件 ===
        np.savetxt(os.path.join(image_dir, "rvecs.txt"), np.array(rvecs).reshape(len(rvecs), 3), fmt="%.6f")
        np.savetxt(os.path.join(image_dir, "tvecs.txt"), np.array(tvecs).reshape(len(tvecs), 3), fmt="%.6f")
        np.savetxt(os.path.join(image_dir, "valid_indices.txt"), np.array(valid_indices), fmt="%d")
        print("结果已保存为 rvecs.txt, tvecs.txt, valid_indices.txt")

        return rvecs, tvecs, valid_indices

    def build_transform_matrix_from_pnp(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        H = np.eye(4)
        H[:3, :3] = R.T
        H[:3, 3] = (-R.T @ tvec).flatten()
        return H
    def scantobase(self, image_dir, ply_dir):
        objp = np.zeros((self.XX * self.YY, 3), np.float32)
        objp[:, :2] = np.array([[(2 * j + i % 2) * self.L_horizontal / 2, i * self.L_vertical]
                                for i in range(self.YY) for j in range(self.XX)])

        rvecs = [np.array([[0.01173681],[0.04523675],[-3.08424954]]),
                 np.array([[0.89172106],[0.07305816],[-2.95855983]]),
                 np.array([[0.75267938],[0.06053063],[-2.99540015]]),
                 np.array([[0.67255477],[0.05203751],[-3.01604053]]),
                 np.array([[-1.03974544],[-0.06113959],[-2.8883657]]),
                 np.array([[-1.03780756],[-0.05715762],[-2.89035821]]),
                 np.array([[-1.0381705],[-0.05733342],[-2.8905926]]),
                 np.array([[0.91776963],[-0.10608101],[0.05247735]]),
                np.array([[0.01280033],[0.05885893],[-2.98164348]])]

        tvecs = [np.array([[0.64802085],[-4.87676941],[169.93098874]]),
                 np.array([[-0.38867357],[0.98417564],[170.71344593]]),
                 np.array([[18.45973782],[2.53781981],[168.38968181]]),
                 np.array([[28.56625346],[3.40250273],[166.25142141]]),
                 np.array([[-2.92064769e+00],[-6.06431508e-02],[1.33494810e+02]]),
                 np.array([[15.34404154],[9.67209366],[129.48189137]]),
                 np.array([[18.35934308],[9.7095731],[126.89604087]]),
                 np.array([[-34.16045948],[-22.27447234],[144.52283874]]),
                 np.array([[-13.36127499],[3.62941632],[168.68189905]])]
        valid_indices= [0, 1, 2, 3, 4, 5, 6, 8, 9]

        rvecs = rvecs[:-1]
        tvecs = tvecs[:-1]
        valid_indices = valid_indices[:-1]

        if not rvecs:
            print("没有有效PnP结果，退出")
            return None

        H_cam_list = [self.build_transform_matrix_from_pnp(r, t) for r, t in zip(rvecs, tvecs)]

        ply_files = sorted(
            [f for f in os.listdir(ply_dir) if f.endswith('.ply')],
            key=lambda x: int(os.path.splitext(x)[0].split('_')[-1]) if x.split('_')[-1].split('.')[0].isdigit() else -1
        )
        selected_ply_files = [ply_files[i] for i in valid_indices]

        # 原始的所有 gripper pose
        gripper_poses = [
            [-9.278, 441.969, 422.757, -179.346, -0.653, 84.695],
            [-119.088, 450.759, 374.552, 148.328, -2.323, 84.687],
            [-119.087, 450.785, 374.559, 153.62, -2.324, 84.688],
            [-119.066, 450.825, 374.563, 156.657, -2.41, 84.552],
            [108.826, 431.026, 341.94, -140.27, -2.835, 84.538],
            [91.074, 440.293, 351.845, -140.274, -2.845, 84.531],
            [86.97, 440.335, 351.855, -140.273, -2.855, 84.524],
            [35.91, 594.716, 484.448, 160.834, 60.901, -107.477],
            [36.791, 635.395, 498.368, 164.879, 52.32, -102.649],
            # [58.315, 463.324, 433.938, -172.218, 1.401, 93.904]
            [25.411, 452.19, 422.686, -179.534, -0.215, 90.558]
        #     最后一个是带牙刷的
        ]
        selected_gripper_poses = [gripper_poses[i] for i in valid_indices]

        if len(selected_ply_files) != len(H_cam_list):
            print("有效点云与变换数量不一致")
            return None

        all_pcds = []
        for ply_file, H in zip(selected_ply_files, H_cam_list):
            pcd = o3d.io.read_point_cloud(os.path.join(ply_dir, ply_file))
            if not pcd.has_points():
                continue
            pcd.transform(H)
            points = np.asarray(pcd.points)
            mask = (points[:, 2] >= -86) & (points[:, 2] <= 86)
            filtered_pcd = o3d.geometry.PointCloud()
            filtered_pcd.points = o3d.utility.Vector3dVector(points[mask])
            if pcd.has_colors():
                filtered_pcd.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors)[mask])
            filtered_pcd, _ = filtered_pcd.remove_statistical_outlier(16, 1.0)
            if not filtered_pcd.has_colors():
                filtered_pcd.paint_uniform_color(np.random.rand(3))
            all_pcds.append(filtered_pcd)

        if not all_pcds:
            print("无有效点云")
            return None

        # testsource = None

        merged = all_pcds[0]
        merged_down = merged.voxel_down_sample(voxel_size=0.5)

        for i, source in enumerate(all_pcds[1:], 1):
            source_down = source.voxel_down_sample(0.25)
            reg = o3d.pipelines.registration.registration_icp(
                source_down, merged_down, 1.0, np.eye(4),
                o3d.pipelines.registration.TransformationEstimationPointToPoint()
            )
            print(f"[ICP] 第{i}个点云: fitness={reg.fitness:.4f}, rmse={reg.inlier_rmse:.4f}")
            source.transform(reg.transformation)
            merged += source
            # testsource = source
            merged_down = merged.voxel_down_sample(0.5)

        o3d.io.write_point_cloud("./saved_objs/mergeredoldboard.ply", merged)

        tranindex = 0

        # ======= 新增：将merged旋转到机械臂基坐标系 =======
        R_cam2board, _ = cv2.Rodrigues(rvecs[tranindex])
        T_cam2board = np.eye(4)
        T_cam2board[:3, :3] = R_cam2board
        T_cam2board[:3, 3] = np.zeros(3)  # 不用平移

        # 手眼标定变换（你需要根据实际调整）
        R_cam2gripper = np.array([[0.10257118, -0.99472467, 0.0014086],
                                  [0.99471407, 0.10256304, -0.00497433],
                                  [0.00480361, 0.00191138, 0.99998664]])
        t_cam2gripper = np.zeros(3)

        T_cam2gripper = np.eye(4)
        T_cam2gripper[:3, :3] = R_cam2gripper
        T_cam2gripper[:3, 3] = t_cam2gripper

        # 取第6号有效 gripper pose（与 rvecs[6] 对应）
        gripper_pose = selected_gripper_poses[tranindex]
        r = R.from_euler('xyz', [gripper_pose[3], gripper_pose[4], gripper_pose[5]], degrees=True)
        R_gripper2base = r.as_matrix()
        # t_gripper2base = np.array(gripper_pose[:3])
        t_gripper2base = np.zeros(3)

        T_gripper2base = np.eye(4)
        T_gripper2base[:3, :3] = R_gripper2base
        T_gripper2base[:3, 3] = t_gripper2base

        # 总变换矩阵
        T_board2base = T_gripper2base @ T_cam2gripper @ T_cam2board

        # 应用变换
        merged.transform(T_board2base)


        # testsource.transform(T_gripper2base)


        # ======= 结束 =======

        return merged


    def scantobasesinglerobot(self, ply_dir):

        pcd = o3d.io.read_point_cloud(ply_dir)

        # pcd = pcd.voxel_down_sample(voxel_size=0.8)
        #
        # points = np.asarray(pcd.points)
        # mask = (points[:, 2] >= -36) & (points[:, 2] <= 36)
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(points[mask])
        # if pcd.has_colors():
        #     pcd.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors)[mask])
        # pcd, _ = pcd.remove_statistical_outlier(16, 1.0)

        # 原始的所有 gripper pose
        gripper_poses = [
            # [-119.088, 450.759, 374.552, 148.328, -2.323, 84.687]
            # [25.411, 452.19, 422.686, -179.534, -0.215, 90.558]
            [264.8929,-285.1852,391.0669,-179.7725,-1.3507,-145.9055]
        ]

        tranindex = 0


        # 手眼标定变换（你需要根据实际调整）
        # R_cam2gripper = np.array([[0.10257118, -0.99472467, 0.0014086],
        #                           [0.99471407, 0.10256304, -0.00497433],
        #                           [0.00480361, 0.00191138, 0.99998664]])

        # R_cam2gripper = np.array([[0.99793716, 0.06251409, 0.0146084],
        #                           [-0.06295546, 0.99750287, 0.03200991],
        #                           [-0.01257085, -0.03286356, 0.99938079]])

        R_cam2gripper = np.array([[0.99926651, 0.03723686, 0.00893671],
                                  [-0.03721491, 0.99930388, - 0.00260967],
                                  [-0.00902766, 0.00227518, 0.99995666]])
        t_cam2gripper = np.zeros(3)

        T_cam2gripper = np.eye(4)
        T_cam2gripper[:3, :3] = R_cam2gripper
        T_cam2gripper[:3, 3] = t_cam2gripper

        # 取第6号有效 gripper pose（与 rvecs[6] 对应）
        gripper_pose = gripper_poses[tranindex]
        r = R.from_euler('xyz', [gripper_pose[3], gripper_pose[4], gripper_pose[5]], degrees=True)
        R_gripper2base = r.as_matrix()
        t_gripper2base = np.zeros(3)

        T_gripper2base = np.eye(4)
        T_gripper2base[:3, :3] = R_gripper2base
        T_gripper2base[:3, 3] = t_gripper2base

        # 总变换矩阵
        T_board2base = T_gripper2base @ T_cam2gripper

        # 应用变换
        pcd.transform(T_board2base)

        # ======= 结束 =======
        return pcd

    def scantobasesingle(self, image_dir, ply_dir, base_ply_path):
        import copy

        objp = np.zeros((self.XX * self.YY, 3), np.float32)
        objp[:, :2] = np.array([[(2 * j + i % 2) * self.L_horizontal / 2, i * self.L_vertical]
                                for i in range(self.YY) for j in range(self.XX)])

        #最后平着的姿态的值



        rvecs = [np.array([[0.04982151],
                  [0.119391],
                  [-1.42787529]])]


        tvecs = [np.array([[-43.17341761],
                  [17.24187208],
                  [182.61674778]])]
        valid_indices = [0]

        if not rvecs:
            print("没有有效PnP结果，退出")
            return None

        H_cam_list = [self.build_transform_matrix_from_pnp(r, t) for r, t in zip(rvecs, tvecs)]


        selected_ply_files = [ply_dir]

        # 原始的所有 gripper pose
        gripper_poses = [
            [25.411, 452.19, 422.686, -179.534, -0.215, 90.558]
        ]


        selected_gripper_poses = [gripper_poses[i] for i in valid_indices]

        if len(selected_ply_files) != len(H_cam_list):
            print("有效点云与变换数量不一致")
            return None
        testsource = None
        all_pcds = []
        for ply_file, H in zip(selected_ply_files, H_cam_list):
            pcd = o3d.io.read_point_cloud(os.path.join(ply_dir, ply_file))
            testsource = copy.deepcopy(pcd)
            if not pcd.has_points():
                continue
            pcd.transform(H)
            points = np.asarray(pcd.points)
            mask = (points[:, 2] >= -36) & (points[:, 2] <= 36)
            filtered_pcd = o3d.geometry.PointCloud()
            filtered_pcd.points = o3d.utility.Vector3dVector(points[mask])
            if pcd.has_colors():
                filtered_pcd.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors)[mask])
            filtered_pcd, _ = filtered_pcd.remove_statistical_outlier(16, 1.0)
            if not filtered_pcd.has_colors():
                filtered_pcd.paint_uniform_color(np.random.rand(3))
            all_pcds.append(filtered_pcd)

        if not all_pcds:
            print("无有效点云")
            return None

        # 这里加载 base_ply_path 点云替代 all_pcds[0]
        base_pcd = o3d.io.read_point_cloud(base_ply_path)
        if not base_pcd.has_points():
            print("base_ply_path 加载点云无效")
            return None
        merged = copy.deepcopy(base_pcd)

        merged_down = merged.voxel_down_sample(voxel_size=0.5)


        # 从all_pcds[1]开始融合，跳过all_pcds[0]
        for i, source in enumerate(all_pcds[0:], 1):
            source_down = source.voxel_down_sample(0.25)
            reg = o3d.pipelines.registration.registration_icp(
                source_down, merged_down, 1.0, np.eye(4),
                o3d.pipelines.registration.TransformationEstimationPointToPoint()
            )
            print(f"[ICP] 第{i}个点云: fitness={reg.fitness:.4f}, rmse={reg.inlier_rmse:.4f}")
            source.transform(reg.transformation)
            merged += source
            merged_down = merged.voxel_down_sample(0.5)


        tranindex = 0

        # rvecs = [np.array([[-0.23307685],[-0.20718013],[ 1.11039331]])]

        rvecs = [np.array([[0.04982151],
                           [0.119391],
                           [-1.42787529]])]
        # ======= 新增：将merged旋转到机械臂基坐标系 =======
        R_cam2board, _ = cv2.Rodrigues(rvecs[tranindex])
        T_cam2board = np.eye(4)
        T_cam2board[:3, :3] = R_cam2board
        T_cam2board[:3, 3] = np.zeros(3)  # 不用平移

        # 手眼标定变换（你需要根据实际调整）

        # R_cam2gripper = np.array([[0.09884656, - 0.99501953, - 0.0128642],
        #                           [0.99358876, 0.09940076, - 0.05385968],
        #                           [0.05487015, - 0.00745788, 0.99846565]])
        R_cam2gripper = np.array([[0.99926651, 0.03723686, 0.00893671],
                                  [-0.03721491, 0.99930388, - 0.00260967],
                                  [-0.00902766, 0.00227518, 0.99995666]])
        t_cam2gripper = np.zeros(3)

        T_cam2gripper = np.eye(4)
        T_cam2gripper[:3, :3] = R_cam2gripper
        T_cam2gripper[:3, 3] = t_cam2gripper

        # 取第6号有效 gripper pose（与 rvecs[6] 对应）
        gripper_pose = selected_gripper_poses[tranindex]
        r = R.from_euler('xyz', [gripper_pose[3], gripper_pose[4], gripper_pose[5]], degrees=True)
        R_gripper2base = r.as_matrix()
        # t_gripper2base = np.array(gripper_pose[:3])
        t_gripper2base = np.zeros(3)

        T_gripper2base = np.eye(4)
        T_gripper2base[:3, :3] = R_gripper2base
        T_gripper2base[:3, 3] = t_gripper2base

        # 总变换矩阵
        T_board2base = T_gripper2base @ T_cam2gripper @ T_cam2board

        T_board2base2 = T_gripper2base @ T_cam2gripper

        # 应用变换
        merged.transform(T_board2base)

        testsource.transform(T_board2base2)
        o3d.io.write_point_cloud("D:/UsmileProject/hand_eye_calibration/toothsingle.ply", testsource)

        # ======= 结束 =======
        return merged


    def get_rotation_matrix(self,axis, angle_degrees):
        angle = np.radians(angle_degrees)
        c = np.cos(angle)
        s = np.sin(angle)
        if axis == 'x':
            return np.array([[1, 0, 0],
                             [0, c, -s],
                             [0, s, c]])
        elif axis == 'y':
            return np.array([[c, 0, s],
                             [0, 1, 0],
                             [-s, 0, c]])
        elif axis == 'z':
            return np.array([[c, -s, 0],
                             [s, c, 0],
                             [0, 0, 1]])
        else:
            raise ValueError("Invalid axis")

    def interactive_path_on_pointcloudv1(self, pcd, num_samples=28):
        print("请选择多个点用于地测线（Shift+左键），按 Q 完成")

        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="选择路径点")
        vis.add_geometry(pcd)
        vis.run()
        vis.destroy_window()

        picked_ids = vis.get_picked_points()
        if len(picked_ids) < 2:
            print("至少需要选择两个点！")
            return None

        print(f"选中的点索引: {picked_ids}")
        points_np = np.asarray(pcd.points)
        picked_path = points_np[picked_ids]

        # === 在这里插入第一个点和最后一个点 ===
        picked_path = list(picked_path)  # 转为列表便于插入

        # 在第一个点和第二个点之间添加新点，沿y轴负方向8单位
        second_point = picked_path[1]
        new_first_point = second_point + np.array([0.0, -3.0, 0.0])
        picked_path.insert(1, new_first_point)

        # 在最后一个点后添加一个点，沿y轴正方向8单位
        last_point = picked_path[-1]
        new_last_point = last_point + np.array([0.0, 15.0, 0.0])
        picked_path.append(new_last_point)

        # 转回 numpy 数组
        picked_path = np.array(picked_path)

        # === 拟合三维样条曲线 ===
        path_pts = picked_path.T  # 转置为 (3, N) 形式
        tck, u = splprep(path_pts, s=0)  # s=0 表示严格通过所有点

        # 均匀采样 num_samples 个点
        u_fine = np.linspace(0, 1, num_samples)
        sampled_points = np.array(splev(u_fine, tck)).T  # 转回 (N, 3)

        return sampled_points

    def interactive_path_on_pointcloud(self, pcd, num_samples=28):
        print("请选择多个点用于地测线（Shift+左键），按 Q 完成")

        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="选择路径点")
        vis.add_geometry(pcd)
        vis.run()
        vis.destroy_window()

        picked_ids = vis.get_picked_points()
        if len(picked_ids) < 2:
            print("至少需要选择两个点！")
            return None

        print(f"选中的点索引: {picked_ids}")
        points_np = np.asarray(pcd.points)
        picked_path = points_np[picked_ids]

        # === 插入第一个和最后一个扩展点 ===
        picked_path = list(picked_path)  # 转为列表
        second_point = picked_path[1]
        new_first_point = second_point + np.array([0.0, -8.0, 0.0])
        picked_path.insert(1, new_first_point)

        last_point = picked_path[-1]
        new_last_point = last_point + np.array([0.0, 18.0, 0.0])
        picked_path.append(new_last_point)
        picked_path = np.array(picked_path)

        # === 拆分：前2个点保留，中间段拟合，最后1个点保留 ===
        fixed_start = picked_path[:2]  # 前两个原点
        fixed_end = picked_path[-1:]  # 最后一个原点
        fit_segment = picked_path[2:-1]  # 第3个到倒数第1个点

        # 曲线拟合
        path_pts = fit_segment.T
        tck, u = splprep(path_pts, s=0)

        # 计算各段分配的点数
        n_fixed_start = len(fixed_start)
        n_fixed_end = len(fixed_end)
        n_fit = num_samples - n_fixed_start - n_fixed_end
        if n_fit < 2:
            print("样本数太少，无法拟合中间段！")
            return None

        u_fine = np.linspace(0, 1, n_fit)
        fit_points_sampled = np.array(splev(u_fine, tck)).T

        # 拼接完整路径
        sampled_points = np.vstack([fixed_start, fit_points_sampled, fixed_end])

        return sampled_points

    def interactive_path_on_pointcloudv2(self, pcd, num_samples=28, smooth_factor=0.1):
        """
        一次性拟合经过所有选中点的平滑曲线
        :param pcd: Open3D 点云
        :param num_samples: 采样点数量
        :param smooth_factor: 平滑因子，0 表示严格经过所有点，值越大越平滑
        """
        print("请选择多个点用于路径（Shift+左键），按 Q 完成")

        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="选择路径点")
        vis.add_geometry(pcd)
        vis.run()
        vis.destroy_window()

        picked_ids = vis.get_picked_points()
        if len(picked_ids) < 2:
            print("至少需要选择两个点！")
            return None

        print(f"选中的点索引: {picked_ids}")
        points_np = np.asarray(pcd.points)
        picked_path = points_np[picked_ids]

        # 样条拟合
        path_pts = picked_path.T
        tck, u = splprep(path_pts, s=smooth_factor)

        # 采样
        u_fine = np.linspace(0, 1, num_samples)
        fit_points_sampled = np.array(splev(u_fine, tck)).T

        return fit_points_sampled

    def interactive_translate_path(self, pcd, path_pts, step=1.0):
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(window_name="地测线可移动")

        # 主点云
        vis.add_geometry(pcd)

        # 初始化路径线
        path_line = o3d.geometry.LineSet()
        path_line.points = o3d.utility.Vector3dVector(path_pts)
        path_line.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(path_pts) - 1)])
        path_line.colors = o3d.utility.Vector3dVector([[1, 0, 0]] * (len(path_pts) - 1))
        vis.add_geometry(path_line)

        # 定义回调函数
        def move(dx=0, dy=0, dz=0):
            nonlocal path_pts
            translation = np.array([dx, dy, dz])
            # path_pts[1:] += translation  # 只平移第1个之后的点
            path_pts += translation  # 只平移第1个之后的点
            path_line.points = o3d.utility.Vector3dVector(path_pts)
            vis.update_geometry(path_line)
            print(f"平移: dx={dx}, dy={dy}, dz={dz}")

        vis.register_key_callback(ord("S"), lambda v: move(dz=-step))
        vis.register_key_callback(ord("W"), lambda v: move(dz=step))
        vis.register_key_callback(ord("A"), lambda v: move(dx=-step))
        vis.register_key_callback(ord("D"), lambda v: move(dx=step))
        vis.register_key_callback(ord("Q"), lambda v: move(dy=step))
        vis.register_key_callback(ord("E"), lambda v: move(dy=-step))
        vis.register_key_callback(256, lambda v: vis.close())  # ESC退出

        vis.run()
        vis.destroy_window()
        return path_pts


    def interactive_translate_pathv2(self, pcd, path_pts, model_pcd=None, step=1.0):
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(window_name="地测线可移动")

        # 主点云
        vis.add_geometry(pcd)

        # 初始化路径线
        path_line = o3d.geometry.LineSet()
        path_line.points = o3d.utility.Vector3dVector(path_pts)
        path_line.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(path_pts) - 1)])
        path_line.colors = o3d.utility.Vector3dVector([[1, 0, 0]] * (len(path_pts) - 1))
        vis.add_geometry(path_line)

        # 添加模型点云（如果传入）
        model = None
        if model_pcd is not None and len(path_pts) >= 3:
            model = copy.deepcopy(model_pcd)  # 深度复制传入的点云
            # 将模型中心放到第三个点
            model.translate(path_pts[2] - path_pts[0])
            vis.add_geometry(model)

        # 定义回调函数
        def move(dx=0, dy=0, dz=0):
            nonlocal path_pts, model

            # 获取当前视角参数
            ctr = vis.get_view_control()
            params = ctr.convert_to_pinhole_camera_parameters()

            translation = np.array([dx, dy, dz])
            path_pts[1:] += translation  # 只平移第1个之后的点

            # 更新路径线
            path_line.points = o3d.utility.Vector3dVector(path_pts)
            vis.update_geometry(path_line)

            # 如果存在模型且路径至少有3个点，更新模型位置
            if model is not None and len(path_pts) >= 3:
                # 移除之前的模型
                vis.remove_geometry(model, reset_bounding_box=False)
                # 创建新的模型副本
                model = copy.deepcopy(model_pcd)
                # 计算模型需要移动的向量（直接对齐到第三个点）
                move_vector = path_pts[2] - path_pts[0]
                model.translate(move_vector)
                vis.add_geometry(model, reset_bounding_box=False)

            # 恢复视角参数
            ctr.convert_from_pinhole_camera_parameters(params)

            print(f"平移: dx={dx}, dy={dy}, dz={dz}")

        vis.register_key_callback(ord("S"), lambda v: move(dz=-step))
        vis.register_key_callback(ord("W"), lambda v: move(dz=step))
        vis.register_key_callback(ord("A"), lambda v: move(dx=-step))
        vis.register_key_callback(ord("D"), lambda v: move(dx=step))
        vis.register_key_callback(ord("Q"), lambda v: move(dy=step))
        vis.register_key_callback(ord("E"), lambda v: move(dy=-step))
        vis.register_key_callback(256, lambda v: vis.close())  # ESC退出

        vis.run()
        vis.destroy_window()
        return path_pts


    def interactive_translate_pathv8(self, pcd, path_txt_file, model_pcd=None, step=1.0):
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(window_name="地测线可移动")

        # 主点云
        vis.add_geometry(pcd)

        # 读取路径信息（平移和旋转）
        def read_path_from_txt(txt_file):
            path_data = []
            with open(txt_file, 'r') as file:
                for line in file:
                    parts = line.strip().split()  # 假设每行数据是用空格分开的
                    if len(parts) == 6:  # dx, dy, dz, rx, ry, rz
                        dx, dy, dz, rx, ry, rz = map(float, parts)
                        path_data.append((dx, dy, dz, rx, ry, rz))
            return path_data

        # 从txt文件读取路径数据
        path_data = read_path_from_txt(path_txt_file)

        # 初始化路径线
        path_line = o3d.geometry.LineSet()
        path_line.points = o3d.utility.Vector3dVector(np.zeros((len(path_data), 3)))  # 初始化为零点
        path_line.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(path_data) - 1)])
        path_line.colors = o3d.utility.Vector3dVector([[1, 0, 0]] * (len(path_data) - 1))
        vis.add_geometry(path_line)

        # 添加模型点云（如果传入）
        model = None
        if model_pcd is not None:
            model = copy.deepcopy(model_pcd)  # 深度复制传入的点云
            vis.add_geometry(model)

        # 定义平移和旋转的函数
        def transform_point(dx, dy, dz, rx, ry, rz, origin=np.zeros(3)):
            # 将角度转换为弧度
            rx, ry, rz = np.deg2rad([rx, ry, rz])

            # 平移
            translated_point = np.array([dx, dy, dz]) + origin

            # 旋转矩阵 (绕z, y, x轴的旋转)
            R_z = np.array([[np.cos(rz), -np.sin(rz), 0],
                            [np.sin(rz), np.cos(rz), 0],
                            [0, 0, 1]])

            R_y = np.array([[np.cos(ry), 0, np.sin(ry)],
                            [0, 1, 0],
                            [-np.sin(ry), 0, np.cos(ry)]])

            R_x = np.array([[1, 0, 0],
                            [0, np.cos(rx), -np.sin(rx)],
                            [0, np.sin(rx), np.cos(rx)]])

            # 组合旋转矩阵（旋转顺序：绕z轴->绕y轴->绕x轴）
            R = np.dot(R_z, np.dot(R_y, R_x))

            # 旋转后的点
            rotated_point = np.dot(R, translated_point - origin) + origin

            return rotated_point

        # 定义回调函数
        def move(dx=0, dy=0, dz=0):
            nonlocal path_data, model

            # 获取当前视角参数
            ctr = vis.get_view_control()
            params = ctr.convert_to_pinhole_camera_parameters()

            # 更新路径
            for i in range(len(path_data)):
                path_data[i] = (path_data[i][0] + dx, path_data[i][1] + dy, path_data[i][2] + dz,
                                path_data[i][3], path_data[i][4], path_data[i][5])

            # 更新路径线
            path_line.points = o3d.utility.Vector3dVector(
                [transform_point(dx, dy, dz, rx, ry, rz) for dx, dy, dz, rx, ry, rz in path_data])
            vis.update_geometry(path_line)

            # 如果存在模型且路径至少有3个点，更新模型位置
            if model is not None:
                vis.remove_geometry(model, reset_bounding_box=False)
                model = copy.deepcopy(model_pcd)
                model.translate([dx, dy, dz])  # 直接平移模型
                vis.add_geometry(model, reset_bounding_box=False)

            # 恢复视角参数
            ctr.convert_from_pinhole_camera_parameters(params)

            print(f"平移: dx={dx}, dy={dy}, dz={dz}")

        def animate_model():
            nonlocal model, path_data, is_animating

            if len(path_data) < 3 or model is None:
                return

            is_animating = True
            total_points = len(path_data)
            current_index = 0  # 从第一个点开始

            # 获取初始位置
            initial_position = np.zeros(3)  # 假设从原点开始

            while current_index < total_points and is_animating:
                # 获取平移和旋转信息
                dx, dy, dz, rx, ry, rz = path_data[current_index]
                # 计算目标位置
                target_position = transform_point(dx, dy, dz, rx, ry, rz, origin=initial_position)

                # 更新模型位置
                vis.remove_geometry(model, reset_bounding_box=False)
                model = copy.deepcopy(model_pcd)
                model.translate(target_position - initial_position)
                vis.add_geometry(model, reset_bounding_box=False)

                # 更新初始位置为当前位置
                initial_position = target_position
                current_index += 1

                # 小延迟，控制动画速度
                time.sleep(0.1)  # 延迟可调整

                # 更新可视化
                vis.poll_events()
                vis.update_renderer()

            is_animating = False

        # 添加全局变量
        is_animating = False

        # 注册按键回调
        vis.register_key_callback(ord("S"), lambda v: move(dz=-step))
        vis.register_key_callback(ord("W"), lambda v: move(dz=step))
        vis.register_key_callback(ord("A"), lambda v: move(dx=-step))
        vis.register_key_callback(ord("D"), lambda v: move(dx=step))
        vis.register_key_callback(ord("Q"), lambda v: move(dy=step))
        vis.register_key_callback(ord("E"), lambda v: move(dy=-step))
        vis.register_key_callback(ord("P"), lambda v: animate_model())  # P键触发动画
        vis.register_key_callback(256, lambda v: vis.close())  # ESC退出

        vis.run()
        vis.destroy_window()
        return path_data

    def interactive_translate_pathv3(self, pcd, path_pts, model_pcd=None, step=1.0):
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(window_name="地测线可移动")

        # 主点云
        vis.add_geometry(pcd)

        # 初始化路径线
        path_line = o3d.geometry.LineSet()
        path_line.points = o3d.utility.Vector3dVector(path_pts)
        path_line.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(path_pts) - 1)])
        path_line.colors = o3d.utility.Vector3dVector([[1, 0, 0]] * (len(path_pts) - 1))
        vis.add_geometry(path_line)

        # 添加模型点云（如果传入）
        model = None
        if model_pcd is not None and len(path_pts) >= 3:
            model = copy.deepcopy(model_pcd)  # 深度复制传入的点云
            # 将模型中心放到第三个点
            model.translate(path_pts[2] - path_pts[0])
            vis.add_geometry(model)

        # 定义回调函数
        def move(dx=0, dy=0, dz=0):
            nonlocal path_pts, model

            # 获取当前视角参数
            ctr = vis.get_view_control()
            params = ctr.convert_to_pinhole_camera_parameters()

            translation = np.array([dx, dy, dz])
            path_pts[1:] += translation  # 只平移第1个之后的点

            # 更新路径线
            path_line.points = o3d.utility.Vector3dVector(path_pts)
            vis.update_geometry(path_line)

            # 如果存在模型且路径至少有3个点，更新模型位置
            if model is not None and len(path_pts) >= 3:
                # 移除之前的模型
                vis.remove_geometry(model, reset_bounding_box=False)
                # 创建新的模型副本
                model = copy.deepcopy(model_pcd)
                # 计算模型需要移动的向量（直接对齐到第三个点）
                move_vector = path_pts[2] - path_pts[0]
                model.translate(move_vector)
                vis.add_geometry(model, reset_bounding_box=False)

            # 恢复视角参数
            ctr.convert_from_pinhole_camera_parameters(params)

            print(f"平移: dx={dx}, dy={dy}, dz={dz}")

        def animate_model():
            nonlocal model, path_pts, is_animating

            if len(path_pts) < 3 or model is None:
                return

            is_animating = True
            total_points = len(path_pts)
            current_index = 2  # 从第三个点开始

            # 获取初始位置
            initial_position = path_pts[0]

            while current_index < total_points and is_animating:
                # 计算移动向量
                target_position = path_pts[current_index]
                move_vector = target_position - initial_position

                # 更新模型位置
                vis.remove_geometry(model, reset_bounding_box=False)
                model = copy.deepcopy(model_pcd)
                model.translate(move_vector)
                vis.add_geometry(model, reset_bounding_box=False)

                # 更新初始位置为当前位置
                # initial_position = target_position
                current_index += 1

                # 小延迟，控制动画速度
                time.sleep(0.00005)

                # 更新可视化
                vis.poll_events()
                vis.update_renderer()

            is_animating = False

        # 添加全局变量
        is_animating = False

        # 注册按键回调
        vis.register_key_callback(ord("S"), lambda v: move(dz=-step))
        vis.register_key_callback(ord("W"), lambda v: move(dz=step))
        vis.register_key_callback(ord("A"), lambda v: move(dx=-step))
        vis.register_key_callback(ord("D"), lambda v: move(dx=step))
        vis.register_key_callback(ord("Q"), lambda v: move(dy=step))
        vis.register_key_callback(ord("E"), lambda v: move(dy=-step))
        vis.register_key_callback(ord("P"), lambda v: animate_model())  # P键触发动画
        vis.register_key_callback(256, lambda v: vis.close())  # ESC退出

        vis.run()
        vis.destroy_window()
        return path_pts


    def interactive_translate_pathv6(self, pcd, path_pts, model_pcd=None, step=1.0):
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(window_name="地测线可移动")

        # 主点云
        vis.add_geometry(pcd)

        # 初始化路径线
        path_line = o3d.geometry.LineSet()
        path_line.points = o3d.utility.Vector3dVector(path_pts)
        path_line.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(path_pts) - 1)])
        path_line.colors = o3d.utility.Vector3dVector([[1, 0, 0]] * (len(path_pts) - 1))
        vis.add_geometry(path_line)

        # 添加模型点云（如果传入）
        model = None
        if model_pcd is not None and len(path_pts) >= 3:
            model = copy.deepcopy(model_pcd)  # 深度复制传入的点云
            # 将模型中心放到第三个点
            model.translate(path_pts[1] - path_pts[0])
            vis.add_geometry(model)

        # 定义回调函数
        def move(dx=0, dy=0, dz=0):
            nonlocal path_pts, model

            # 获取当前视角参数
            ctr = vis.get_view_control()
            params = ctr.convert_to_pinhole_camera_parameters()

            translation = np.array([dx, dy, dz])
            path_pts[1:] += translation  # 只平移第1个之后的点

            # 更新路径线
            path_line.points = o3d.utility.Vector3dVector(path_pts)
            vis.update_geometry(path_line)

            # 如果存在模型且路径至少有3个点，更新模型位置
            if model is not None and len(path_pts) >= 3:
                # 移除之前的模型
                vis.remove_geometry(model, reset_bounding_box=False)
                # 创建新的模型副本
                model = copy.deepcopy(model_pcd)
                # 计算模型需要移动的向量（直接对齐到第三个点）
                move_vector = path_pts[2] - path_pts[0]
                model.translate(move_vector)
                vis.add_geometry(model, reset_bounding_box=False)

            # 恢复视角参数
            ctr.convert_from_pinhole_camera_parameters(params)

            print(f"平移: dx={dx}, dy={dy}, dz={dz}")

        def animate_model():
            nonlocal model, path_pts, is_animating

            if len(path_pts) < 3 or model is None:
                return

            is_animating = True
            total_points = len(path_pts)
            current_index = 2  # 从第三个点开始

            # 获取初始位置
            initial_position = path_pts[0]

            while current_index < total_points and is_animating:
                # 计算移动向量
                target_position = path_pts[current_index]
                move_vector = target_position - initial_position

                # 更新模型位置
                vis.remove_geometry(model, reset_bounding_box=False)
                model = copy.deepcopy(model_pcd)
                model.translate(move_vector)
                vis.add_geometry(model, reset_bounding_box=False)

                # 更新初始位置为当前位置
                # initial_position = target_position
                current_index += 1

                # 小延迟，控制动画速度
                time.sleep(0.00005)

                # 更新可视化
                vis.poll_events()
                vis.update_renderer()

            is_animating = False

        # 添加全局变量
        is_animating = False

        # 注册按键回调
        vis.register_key_callback(ord("S"), lambda v: move(dz=-step))
        vis.register_key_callback(ord("W"), lambda v: move(dz=step))
        vis.register_key_callback(ord("A"), lambda v: move(dx=-step))
        vis.register_key_callback(ord("D"), lambda v: move(dx=step))
        vis.register_key_callback(ord("Q"), lambda v: move(dy=step))
        vis.register_key_callback(ord("E"), lambda v: move(dy=-step))
        vis.register_key_callback(ord("P"), lambda v: animate_model())  # P键触发动画
        vis.register_key_callback(256, lambda v: vis.close())  # ESC退出

        vis.run()
        vis.destroy_window()
        return path_pts


class BrushPathProcessor:
    def __init__(self):
        pass

    @staticmethod
    def load_pointcloud(filename):
        """
        只加载点云(ply)
        返回 np.ndarray 的点坐标和 open3d 点云对象
        """
        pcd = o3d.io.read_point_cloud(filename)
        V = np.asarray(pcd.points)
        return V, pcd

    @staticmethod
    def pick_vertex_indices_from_pointcloud(points):
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="选择路径点")
        vis.add_geometry(points)
        vis.run()
        vis.destroy_window()

        picked_ids = vis.get_picked_points()
        if len(picked_ids) < 2:
            print("至少需要选择两个点！")
            return None

        print(f"选中的点索引: {picked_ids}")
        points_np = np.asarray(points.points)
        picked_path = points_np[picked_ids]
        return picked_path,picked_ids

    @staticmethod
    def pick_vertex_indices_from_pointcloud2(points):
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="选择MarKer的中心点")
        vis.add_geometry(points)
        vis.run()
        vis.destroy_window()

        picked_ids = vis.get_picked_points()

        print(f"选中的MarKer点索引: {picked_ids}")
        points_np = np.asarray(points.points)
        picked_path = points_np[picked_ids]
        return picked_path,picked_ids

    @staticmethod
    def smooth_and_sample_path(points_np, smooth_factor=0.0, num_samples=100):
        """
        对选定的路径点进行样条拟合和平滑处理，并返回采样后的点。

        参数:
        - points_np: 原始路径点，形状为 (N, 2) 或 (N, M)
        - picked_ids: 选定的点的索引
        - smooth_factor: 样条拟合的平滑因子 (默认 0.0 表示无平滑)
        - num_samples: 采样点的数量 (默认 100)

        返回:
        - fit_points_sampled: 经过样条拟合和采样后的路径点
        """
        # 获取选定的路径点
        picked_path = points_np

        # 样条拟合
        path_pts = picked_path.T
        tck, u = splprep(path_pts, s=smooth_factor)

        # 采样
        u_fine = np.linspace(0, 1, num_samples)
        fit_points_sampled = np.array(splev(u_fine, tck)).T

        return fit_points_sampled

    @staticmethod
    def smooth_and_sample_pathv21(points_np, smooth_factor=0.0, num_samples=36):
        """
        在原始点构成的折线路径上进行等距离插补，返回采样后的点。

        参数:
        - points_np: 原始路径点，形状为 (N, 2) 或 (N, M)
        - smooth_factor: 保留参数以兼容旧代码，实际不使用
        - num_samples: 采样点的数量 (默认 36)

        返回:
        - sampled_points: 沿折线等距离插补后的路径点，形状为 (num_samples, M)
        """
        # 计算各段路径的累计弧长
        distances = np.cumsum(np.linalg.norm(np.diff(points_np, axis=0), axis=1))
        distances = np.concatenate(([0], distances))
        total_length = distances[-1]

        # 生成等间距的弧长参数
        target_distances = np.linspace(0, total_length, num_samples)

        # 对每个目标弧长进行线性插值
        sampled_points = np.zeros((num_samples, points_np.shape[1]))
        for i, d in enumerate(target_distances):
            # 找到目标弧长所在的线段
            idx = np.searchsorted(distances, d) - 1
            idx = max(0, min(idx, len(points_np) - 2))

            # 计算线段内的插值比例
            if distances[idx + 1] - distances[idx] > 0:
                t = (d - distances[idx]) / (distances[idx + 1] - distances[idx])
            else:
                t = 0

            # 线性插值
            sampled_points[i] = (1 - t) * points_np[idx] + t * points_np[idx + 1]

        return sampled_points

    @staticmethod
    def fit_arc_and_sample_points(path_pts, num_samples=30):
        if len(path_pts) < 3:
            raise ValueError("至少需要三个点来拟合圆弧")
        centroid = path_pts.mean(axis=0)
        pts_centered = path_pts - centroid
        U, S, Vt = np.linalg.svd(pts_centered)
        normal = Vt[2]
        plane_basis = Vt[:2]
        pts_2d = pts_centered @ plane_basis.T
        A = np.hstack([2 * pts_2d, np.ones((len(pts_2d), 1))])
        b = np.sum(pts_2d ** 2, axis=1).reshape(-1, 1)
        x = np.linalg.lstsq(A, b, rcond=None)[0].flatten()
        cx, cy, c = x
        radius = np.sqrt(c + cx ** 2 + cy ** 2)
        angles = np.arctan2(pts_2d[:, 1] - cy, pts_2d[:, 0] - cx)
        angles = np.unwrap(angles)
        start_angle = angles[0]
        end_angle = angles[-1]
        sampled_angles = np.linspace(start_angle, end_angle, num_samples)
        sampled_pts_2d = np.stack([
            cx + radius * np.cos(sampled_angles),
            cy + radius * np.sin(sampled_angles)
        ], axis=1)
        sampled_pts_3d = sampled_pts_2d @ plane_basis + centroid
        return sampled_pts_3d

    @staticmethod
    def fit_poly_curve_xys(path_pts, degree=8, num_samples=200):
        """
        基于用户选点进行多项式拟合（仅XY平面）
        支持多次曲线：U / W / M / S 型
        输出采样点方向与输入点方向保持一致
        """

        if len(path_pts) < degree + 1:
            return path_pts


        # ---------------- 取XY拟合 ----------------
        X = path_pts[:, 0]
        Y = path_pts[:, 1]

        # 🔥多项式拟合 (Y = f(X))
        poly_coeff = np.polyfit(X, Y, degree)
        poly = np.poly1d(poly_coeff)

        # ---------------- 匀采样曲线 ----------------
        x_min, x_max = X.min(), X.max()
        xs = np.linspace(x_min, x_max, num_samples)
        ys = poly(xs)

        # Z 平均值
        z_mean = path_pts[:, 2].mean()
        sampled_pts = np.column_stack([xs, ys, np.full(num_samples, z_mean)])

        # ---------------- 保持方向一致 ----------------
        # 输入序列方向：True=递增，False=递减
        input_increasing = X[0] < X[-1]

        # 输出序列也是递增（xs 是从小到大）
        output_increasing = sampled_pts[0, 0] < sampled_pts[-1, 0]

        # 如果方向不一致，反转
        if input_increasing != output_increasing:
            sampled_pts = sampled_pts[::-1].copy()

        return sampled_pts

    @staticmethod
    def fit_u_curve_xy(path_pts, num_samples=80):
        """
        根据用户选取点拟合 U 型曲线，仅在 XY 平面拟合 y = ax^2 + bx + c
        :param path_pts: ndarray(N,3) 输入点，部分Z可忽略
        :param num_samples: 采样密度
        :return: 曲线采样后的3D点
        """

        if len(path_pts) < 3:
            raise ValueError("至少需要三个点用于拟合 U 型曲线")

        # ---- 只取 XY 平面进行拟合 ----
        xy = path_pts[:, :2]  # (N,2)
        X = xy[:, 0]  # x 坐标
        Y = xy[:, 1]  # y 坐标

        # 🔥 拟合 U 型曲线：y = Ax^2 + Bx + C
        A = np.column_stack([X ** 2, X, np.ones_like(X)])
        coeff, *_ = np.linalg.lstsq(A, Y, rcond=None)  # 最小二乘求 a b c
        a, b, c = coeff

        # ---- 沿曲线上进行均匀采样 ----
        x_min, x_max = X.min(), X.max()
        xs = np.linspace(x_min, x_max, num_samples)
        ys = a * xs ** 2 + b * xs + c

        # ---- 生成输出 3D 点（Z 使用原点均值or保持不变）----
        z_mean = path_pts[:, 2].mean()  # 默认保持同一个平面高度
        curve_pts = np.column_stack([xs, ys, np.full(num_samples, z_mean)])

        return curve_pts

    @staticmethod
    def interactive_pick_path_points_v3(pcd, picked_path):
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window("Pick Path Points")

        # ========== 提高点显示明显度 ==========
        opt = vis.get_render_option()
        opt.point_size = 6  # ⭐点大小
        opt.light_on = True  # 更亮
        opt.background_color = np.array([0, 0, 0])  # 黑色背景更清晰（可去掉）

        # ===== 显示点云原色 =====
        vis.add_geometry(pcd)

        # ===== 显示路径(蓝色) =====
        path_pcd = o3d.geometry.PointCloud()
        path_pcd.points = o3d.utility.Vector3dVector(picked_path)
        path_pcd.paint_uniform_color([0.1, 0.1, 1.0])
        vis.add_geometry(path_pcd)

        # ===== 已选点 =====
        selected_points = []  # 真实选择顺序
        selected_colors = []  # 红/绿
        is_backward = []  # True=逆向

        line_set = o3d.geometry.LineSet()
        vis.add_geometry(line_set)

        ptr_forward = 0
        ptr_backward = len(picked_path) - 1

        # ============================================================
        # 更新显示线段
        # ============================================================
        def update_line_set():
            if len(selected_points) == 0:
                line_set.points = o3d.utility.Vector3dVector([])
                line_set.lines = o3d.utility.Vector2iVector([])
                line_set.colors = o3d.utility.Vector3dVector([])
                vis.update_geometry(line_set)
                vis.poll_events()
                vis.update_renderer()
                return

            vis_order = []
            vis_colors = []
            for p, c, back in zip(selected_points, selected_colors, is_backward):
                if back:
                    vis_order.insert(0, p)
                    vis_colors.insert(0, c)
                else:
                    vis_order.append(p)
                    vis_colors.append(c)

            vis_order = np.array(vis_order)

            line_set.points = o3d.utility.Vector3dVector(vis_order)
            lines = [[i, i + 1] for i in range(len(vis_order) - 1)]
            line_set.lines = o3d.utility.Vector2iVector(lines)

            # 为每段线赋予与起点相同的颜色
            ls = []
            for i in range(len(lines)):
                ls.append(vis_colors[i])
            line_set.colors = o3d.utility.Vector3dVector(ls)

            vis.update_geometry(line_set)
            vis.poll_events()
            vis.update_renderer()

        # ============================================================
        # 按键逻辑
        # ============================================================

        def on_key_A(v):
            nonlocal ptr_forward
            if ptr_forward < len(picked_path):
                selected_points.append(picked_path[ptr_forward])
                selected_colors.append([1, 0, 0])
                is_backward.append(False)
                ptr_forward += 1
                update_line_set()
            return True

        def on_key_D(v):
            nonlocal ptr_forward
            if len(selected_points) > 0:
                idx = None
                for i in range(len(selected_points) - 1, -1, -1):
                    if not is_backward[i]:
                        idx = i
                        break
                if idx is not None:
                    selected_points.pop(idx)
                    selected_colors.pop(idx)
                    is_backward.pop(idx)
                    ptr_forward = max(0, ptr_forward - 1)
                    update_line_set()
            return True

        def on_key_W(v):
            nonlocal ptr_backward
            if ptr_backward >= 0:
                selected_points.append(picked_path[ptr_backward])
                selected_colors.append([0, 1, 0])
                is_backward.append(True)
                ptr_backward -= 1
                update_line_set()
            return True

        def on_key_S(v):
            nonlocal ptr_backward
            if len(selected_points) > 0:
                idx = None
                for i in range(len(selected_points) - 1, -1, -1):
                    if is_backward[i]:
                        idx = i
                        break
                if idx is not None:
                    selected_points.pop(idx)
                    selected_colors.pop(idx)
                    is_backward.pop(idx)
                    ptr_backward = min(len(picked_path) - 1, ptr_backward + 1)
                    update_line_set()
            return True

        def on_key_G(v):
            """逆向删除最后选中的点（无论正向逆向）"""
            nonlocal ptr_forward, ptr_backward
            if len(selected_points) > 0:
                # 删除最后一个点
                last_is_backward = is_backward[-1]
                selected_points.pop()
                selected_colors.pop()
                is_backward.pop()

                # 根据删除的点类型调整指针
                if last_is_backward:
                    ptr_backward = min(len(picked_path) - 1, ptr_backward + 1)
                else:
                    ptr_forward = max(0, ptr_forward - 1)

                update_line_set()
            return True

        def on_key_F(v):
            """正向删除第一个选中的点（无论正向逆向）"""
            nonlocal ptr_forward, ptr_backward
            if len(selected_points) > 0:
                # 删除第一个点
                first_is_backward = is_backward[0]
                selected_points.pop(0)
                selected_colors.pop(0)
                is_backward.pop(0)

                # 根据删除的点类型调整指针
                if first_is_backward:
                    ptr_backward = min(len(picked_path) - 1, ptr_backward + 1)
                else:
                    ptr_forward = max(0, ptr_forward - 1)

                update_line_set()
            return True

        def on_key_Q(v):
            vis.close()
            return False

        vis.register_key_callback(ord("A"), on_key_A)
        vis.register_key_callback(ord("D"), on_key_D)
        vis.register_key_callback(ord("W"), on_key_W)
        vis.register_key_callback(ord("S"), on_key_S)
        vis.register_key_callback(ord("G"), on_key_G)  # 逆向删除最后点
        vis.register_key_callback(ord("F"), on_key_F)  # 正向删除第一个点
        vis.register_key_callback(ord("Q"), on_key_Q)
        vis.register_key_callback(256, on_key_Q)

        vis.run()
        vis.destroy_window()

        return np.array(selected_points)
    # def interactive_pick_path_points_v3(pcd, picked_path):
    #     vis = o3d.visualization.VisualizerWithKeyCallback()
    #     vis.create_window("Pick Path Points")
    #
    #     # ========== 提高点显示明显度 ==========
    #     opt = vis.get_render_option()
    #     opt.point_size = 20  # ⭐点大小
    #     opt.light_on = True  # 更亮
    #     opt.background_color = np.array([0, 0, 0])  # 黑色背景更清晰（可去掉）
    #
    #     # ===== 显示点云原色 =====
    #     vis.add_geometry(pcd)
    #
    #     # ===== 显示路径(蓝色) =====
    #     path_pcd = o3d.geometry.PointCloud()
    #     path_pcd.points = o3d.utility.Vector3dVector(picked_path)
    #     path_pcd.paint_uniform_color([0.1, 0.1, 1.0])
    #     vis.add_geometry(path_pcd)
    #
    #     # ===== 已选点 =====
    #     selected_points = []  # 真实选择顺序
    #     selected_colors = []  # 红/绿
    #     is_backward = []  # True=逆向
    #
    #     line_set = o3d.geometry.LineSet()
    #     vis.add_geometry(line_set)
    #
    #     ptr_forward = 0
    #     ptr_backward = len(picked_path) - 1
    #
    #     # ============================================================
    #     # 更新显示线段
    #     # ============================================================
    #     def update_line_set():
    #         if len(selected_points) == 0:
    #             line_set.points = o3d.utility.Vector3dVector([])
    #             line_set.lines = o3d.utility.Vector2iVector([])
    #             line_set.colors = o3d.utility.Vector3dVector([])
    #             vis.update_geometry(line_set)
    #             vis.poll_events()
    #             vis.update_renderer()
    #             return
    #
    #         vis_order = []
    #         vis_colors = []
    #         for p, c, back in zip(selected_points, selected_colors, is_backward):
    #             if back:
    #                 vis_order.insert(0, p)
    #                 vis_colors.insert(0, c)
    #             else:
    #                 vis_order.append(p)
    #                 vis_colors.append(c)
    #
    #         vis_order = np.array(vis_order)
    #
    #         line_set.points = o3d.utility.Vector3dVector(vis_order)
    #         lines = [[i, i + 1] for i in range(len(vis_order) - 1)]
    #         line_set.lines = o3d.utility.Vector2iVector(lines)
    #
    #         # 为每段线赋予与起点相同的颜色
    #         ls = []
    #         for i in range(len(lines)):
    #             ls.append(vis_colors[i])
    #         line_set.colors = o3d.utility.Vector3dVector(ls)
    #
    #         vis.update_geometry(line_set)
    #         vis.poll_events()
    #         vis.update_renderer()
    #
    #     # ============================================================
    #     # 按键逻辑
    #     # ============================================================
    #
    #     def on_key_A(v):
    #         nonlocal ptr_forward
    #         if ptr_forward < len(picked_path):
    #             selected_points.append(picked_path[ptr_forward])
    #             selected_colors.append([1, 0, 0])
    #             is_backward.append(False)
    #             ptr_forward += 1
    #             update_line_set()
    #         return True
    #
    #     def on_key_D(v):
    #         nonlocal ptr_forward
    #         if len(selected_points) > 0:
    #             idx = None
    #             for i in range(len(selected_points) - 1, -1, -1):
    #                 if not is_backward[i]:
    #                     idx = i
    #                     break
    #             if idx is not None:
    #                 selected_points.pop(idx)
    #                 selected_colors.pop(idx)
    #                 is_backward.pop(idx)
    #                 ptr_forward = max(0, ptr_forward - 1)
    #                 update_line_set()
    #         return True
    #
    #     def on_key_W(v):
    #         nonlocal ptr_backward
    #         if ptr_backward >= 0:
    #             selected_points.append(picked_path[ptr_backward])
    #             selected_colors.append([0, 1, 0])
    #             is_backward.append(True)
    #             ptr_backward -= 1
    #             update_line_set()
    #         return True
    #
    #     def on_key_S(v):
    #         nonlocal ptr_backward
    #         if len(selected_points) > 0:
    #             idx = None
    #             for i in range(len(selected_points) - 1, -1, -1):
    #                 if is_backward[i]:
    #                     idx = i
    #                     break
    #             if idx is not None:
    #                 selected_points.pop(idx)
    #                 selected_colors.pop(idx)
    #                 is_backward.pop(idx)
    #                 ptr_backward = min(len(picked_path) - 1, ptr_backward + 1)
    #                 update_line_set()
    #         return True
    #
    #     def on_key_Q(v):
    #         vis.close()
    #         return False
    #
    #     vis.register_key_callback(ord("A"), on_key_A)
    #     vis.register_key_callback(ord("D"), on_key_D)
    #     vis.register_key_callback(ord("W"), on_key_W)
    #     vis.register_key_callback(ord("S"), on_key_S)
    #     vis.register_key_callback(ord("Q"), on_key_Q)
    #     vis.register_key_callback(256, on_key_Q)
    #
    #     vis.run()
    #     vis.destroy_window()
    #
    #     return np.array(selected_points)

    @staticmethod
    def visualize_path_and_pointcloud(points, path_pts):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.paint_uniform_color([0.7, 0.7, 0.7])

        path_line = o3d.geometry.LineSet()
        path_line.points = o3d.utility.Vector3dVector(path_pts)
        path_line.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(path_pts) - 1)])
        path_line.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(len(path_pts) - 1)])

        o3d.visualization.draw_geometries([pcd, path_line])

    @staticmethod
    def compute_directions(path_pts):
        directions = path_pts[1:] - path_pts[:-1]
        norms = np.linalg.norm(directions, axis=1, keepdims=True)
        directions = directions / (norms + 1e-8)
        # directions = np.vstack([directions, directions[-1]])
        print('directions = ',directions)
        return directions

    @staticmethod
    def rotation_from_vector_to_vector(v1, v2):
        # 归一化向量
        v1 = v1 / np.linalg.norm(v1)
        v2 = v2 / np.linalg.norm(v2)

        # 计算旋转轴，限制旋转轴在 Z 轴平面
        axis = np.cross(v1, v2)
        axis_norm = np.linalg.norm(axis)

        if axis_norm < 1e-8:
            return np.eye(3)  # 如果两个向量平行，则返回单位矩阵

        # 限制旋转轴只在Z轴上
        axis = axis / axis_norm
        # 需要限制当平刷的时候
        axis[0] = 0  # X轴方向旋转忽略
        axis[1] = 0  # Y轴方向旋转忽略

        # 计算旋转角度
        angle = np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))

        # print("angles :",angle)

        # 如果旋转轴过于接近Z轴, 就直接按Z轴旋转计算
        if np.abs(axis[2]) < 1e-8:
            # 如果方向非常接近Z轴，按X和Y旋转
            R = np.eye(3)
            R[0, 0] = np.cos(angle)
            R[0, 1] = -np.sin(angle)
            R[1, 0] = np.sin(angle)
            R[1, 1] = np.cos(angle)
            return R
        else:
            # 使用旋转矩阵公式
            K = np.array([
                [0, -axis[2], axis[1]],
                [axis[2], 0, -axis[0]],
                [-axis[1], axis[0], 0]
            ])
            R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
            return R

    @staticmethod
    def rotate_pointcloud(pcd: o3d.geometry.PointCloud, R: np.ndarray, center: np.ndarray):
        pts = np.asarray(pcd.points)
        pts = pts - center
        pts = pts @ R.T
        pts = pts + center
        pcd_rot = o3d.geometry.PointCloud()
        pcd_rot.points = o3d.utility.Vector3dVector(pts)
        if pcd.has_colors():
            pcd_rot.colors = pcd.colors
        return pcd_rot

    @staticmethod
    def translate_pointcloud(pcd: o3d.geometry.PointCloud, translation: np.ndarray):
        pts = np.asarray(pcd.points)
        pts = pts + translation
        pcd_t = o3d.geometry.PointCloud()
        pcd_t.points = o3d.utility.Vector3dVector(pts)
        if pcd.has_colors():
            pcd_t.colors = pcd.colors
        return pcd_t

    @staticmethod
    def write_matrix_to_file(filename, matrix):
        """将矩阵写入文件，每行格式化为字符串，先清除原有内容"""
        try:
            with open(filename, 'w') as file:  # 'w'模式会自动清空文件内容
                for row in matrix:
                    # 将每行数字转换为字符串并用逗号连接（保持原有格式）
                    line = ','.join(f"{num:.6f}" for num in row)
                    file.write(line + '\n')
            print(f"成功写入文件：{filename}")
        except Exception as e:
            print(f"写入文件 {filename} 时发生错误：{e}")

    @staticmethod
    def write_matrix_to_file_empty(filename, matrix):
        """将矩阵写入文件，每行格式化为字符串"""
        try:
            with open(filename, 'w') as file:
                for row in matrix:
                    # 将每行数字转换为字符串并用空格连接
                    line = ' '.join(map(str, row))
                    file.write(line + '\n')
            print(f"成功写入文件：{filename}")
        except Exception as e:
            print(f"写入文件 {filename} 时发生错误：{e}")

    def save_transforms_along_path(self, pcd, path_pts, mesh_direction_pts, output_txt="transforms.txt"):
        """
        保存每一帧的平移 (x,y,z) 和旋转欧拉角 (rx,ry,rz) 到 txt 文件
        旋转欧拉角单位：度，使用 ZYX 顺序（机械臂常用）
        """
        original_pcd = copy.deepcopy(pcd)

        v1 = np.asarray(mesh_direction_pts[0])
        v2 = np.asarray(mesh_direction_pts[1])
        model_forward = v2 - v1
        model_forward /= np.linalg.norm(model_forward)
        mesh_center = np.asarray(mesh_direction_pts[2])

        directions = self.compute_directions(path_pts)

        transforms = []

        for i, (pos, dir_vec) in enumerate(zip(path_pts, directions)):
            if i == 0:
                # 第一帧没有旋转和平移
                translation = np.zeros(3)
                euler_deg = np.zeros(3)
            else:
                R_mat = self.rotation_from_vector_to_vector(model_forward, dir_vec)
                translation = pos - mesh_center
                euler_deg = R.from_matrix(R_mat).as_euler("zyx", degrees=True)

            transforms.append(np.hstack([translation, euler_deg]))

        # 保存到 txt
        np.savetxt(output_txt, np.array(transforms), fmt="%.6f")
        print(f"位姿数据已保存到: {output_txt}")

    def save_pointcloud_along_path_as_ply(self, pcd, path_pts, mesh_direction_pts, output_folder="saved_ply"):
        os.makedirs(output_folder, exist_ok=True)
        original_pcd = copy.deepcopy(pcd)
        v1 = np.asarray(mesh_direction_pts[0])
        v2 = np.asarray(mesh_direction_pts[1])
        model_forward = v2 - v1
        model_forward /= np.linalg.norm(model_forward)
        mesh_center = np.asarray(mesh_direction_pts[2])
        directions = self.compute_directions(path_pts)

        for i, (pos, dir_vec) in enumerate(zip(path_pts, directions)):
            tmp_pcd = copy.deepcopy(original_pcd)
            if i == 0:
                filename = os.path.join(output_folder, f"frame_{i:04d}.ply")
                o3d.io.write_point_cloud(filename, tmp_pcd)
                print(f"Saved {filename}")
                continue
            R = self.rotation_from_vector_to_vector(model_forward, dir_vec)
            tmp_pcd = self.rotate_pointcloud(tmp_pcd, R, mesh_center)
            translation = pos - mesh_center
            tmp_pcd = self.translate_pointcloud(tmp_pcd, translation)
            filename = os.path.join(output_folder, f"frame_{i:04d}.ply")
            o3d.io.write_point_cloud(filename, tmp_pcd)
            print(f"Saved {filename}")


    def save_pointcloud_along_path_as_plyv2(self, pcd, path_pts, mesh_direction_pts, mesh_direction_ids,
                                            output_folder="saved_ply"):
        """
        mesh_direction_pts: 旋转轴和中心的三个点坐标
        mesh_direction_ids: 对应点在点云中的索引，用于配准
        """
        import math
        import os
        import copy
        import numpy as np
        from scipy.spatial.transform import Rotation as R
        import open3d as o3d

        os.makedirs(output_folder, exist_ok=True)
        original_pcd = copy.deepcopy(pcd)

        v1 = np.asarray(mesh_direction_pts[0])
        v2 = np.asarray(mesh_direction_pts[1])
        model_forward = v2 - v1
        model_forward /= np.linalg.norm(model_forward)
        mesh_center = np.asarray(mesh_direction_pts[2])

        directions = self.compute_directions(path_pts)

        transform_file = os.path.join(output_folder, "transforms.txt")
        transform_file = "D:\\hand_eye_calibration\\saved_ply\\transforms.txt"
        try:
            with open(transform_file, "w") as f:
                for i, (pos, dir_vec) in enumerate(zip(path_pts, directions)):
                    tmp_pcd = copy.deepcopy(original_pcd)

                    # if i == 0:
                    #     filename = os.path.join(output_folder, f"frame_{i:04d}.ply")
                    #     # o3d.io.write_point_cloud(filename, tmp_pcd)
                    #     f.write("0 0 0 0 0 0\n")
                    #     f.flush()  # Ensure immediate write
                    #     continue

                    # 先基于 mesh_center 旋转
                    R_local = self.rotation_from_vector_to_vector(model_forward, dir_vec)
                    tmp_pcd = self.rotate_pointcloud(tmp_pcd, R_local, mesh_center)

                    # 平移
                    translation = pos - mesh_center
                    tmp_pcd = self.translate_pointcloud(tmp_pcd, translation)

                    # 保存 ply
                    # filename = os.path.join(output_folder, f"frame_{i:04d}.ply")
                    # o3d.io.write_point_cloud(filename, tmp_pcd)

                    # ---- 从对应点计算全局刚体变换 ----
                    src_pts = np.asarray(original_pcd.points)[mesh_direction_ids]
                    dst_pts = np.asarray(tmp_pcd.points)[mesh_direction_ids]
                    R_global, t_global = self.compute_rigid_transform(src_pts, dst_pts)  # 基于原点
                    t_global = translation

                    # 欧拉角（度）
                    rzlocal, rylocal, rxlocal = R.from_matrix(R_local).as_euler("zyx", degrees=True)
                    rz, ry, rx = R.from_matrix(R_global).as_euler("zyx", degrees=True)


                    print("current id is:",i)
                    print("rzlocal :",rzlocal)
                    print("rylocal :",rylocal)
                    print("rxlocal :",rxlocal)


                    print("rz :",rz)
                    print("ry :",ry)
                    print("rx :",rx)

                    # 写入并立即刷新
                    transform_line = f"{t_global[0]:.6f} {t_global[1]:.6f} {t_global[2]:.6f} {rx:.6f} {ry:.6f} {rz:.6f}\n"
                    f.write(transform_line)
                    f.flush()

            print(f"Transforms successfully saved to {transform_file}")
        except IOError as e:
            print(f"Error saving transforms.txt: {e}")
            raise


    def save_pointcloud_along_path_as_plyv21(self, pcd, path_pts, mesh_direction_pts, mesh_direction_ids,
                                             output_folder="saved_ply",typeid=7):
        """
        mesh_direction_pts: 旋转轴和中心的三个点坐标
        mesh_direction_ids: 对应点在点云中的索引，用于配准
        """
        import math
        import os
        import copy
        import numpy as np
        from scipy.spatial.transform import Rotation as R
        import open3d as o3d

        os.makedirs(output_folder, exist_ok=True)
        original_pcd = copy.deepcopy(pcd)

        v1 = np.asarray(mesh_direction_pts[0])
        v2 = np.asarray(mesh_direction_pts[1])
        model_forward = v2 - v1
        model_forward /= np.linalg.norm(model_forward)
        mesh_center = np.asarray(mesh_direction_pts[2])

        path_pts = np.asarray(path_pts)
        n_pts = len(path_pts)
        # 每个路径点一条方向：差分得到 N-1 条，最后一点沿用上一段的切向，使 transform 行数与点数一致
        if n_pts == 0:
            directions = np.zeros((0, 3))
        elif n_pts == 1:
            directions = model_forward.reshape(1, -1).copy()
        else:
            directions = self.compute_directions(path_pts)
            if len(directions) == n_pts - 1:
                directions = np.vstack([directions, directions[-1:]])
            elif len(directions) != n_pts:
                raise ValueError(
                    f"path_pts 点数 {n_pts} 与 directions 行数 {len(directions)} 不匹配，且无法自动补齐"
                )

        transform_file = os.path.join(output_folder, "transforms.txt")
        # transform_file = "D:\\UsmileProject\\hand_eye_calibration\\saved_ply\\transforms.txt"

        transform_data = []  # 用于存储每一行的 numpy 数组
        rz_first = 0
        try:
            with open(transform_file, "w") as f:
                for i, (pos, dir_vec) in enumerate(zip(path_pts, directions)):
                    tmp_pcd = copy.deepcopy(original_pcd)

                    # 先基于 mesh_center 旋转
                    R_local = self.rotation_from_vector_to_vector(model_forward, dir_vec)
                    tmp_pcd = self.rotate_pointcloud(tmp_pcd, R_local, mesh_center)

                    # 平移
                    translation = pos - mesh_center
                    tmp_pcd = self.translate_pointcloud(tmp_pcd, translation)

                    # ---- 从对应点计算全局刚体变换 ----
                    src_pts = np.asarray(original_pcd.points)[mesh_direction_ids]
                    dst_pts = np.asarray(tmp_pcd.points)[mesh_direction_ids]
                    R_global, t_global = self.compute_rigid_transform(src_pts, dst_pts)  # 基于原点
                    t_global = translation

                    # 欧拉角（度）
                    rz, ry, rx = R.from_matrix(R_global).as_euler("zyx", degrees=True)

                    rx = 0
                    ry = 0
                    if (typeid == 6):
                        rx = 0
                        ry = 0
                        rz = 0

                    if (i == 0 and typeid != 7 and typeid != 6 and typeid !=0 and typeid !=1):
                        if (typeid == 3 or typeid == 4):
                            rz_first = rz + 5
                            rz = -5
                        else:
                            rz_first = rz - 5
                            rz = 5
                    else:
                        if(typeid == 0 or typeid ==1):
                            rz = rz +0
                        else:
                            rz -= rz_first

                    # rx = 0
                    # ry = 0
                    # rz = 0


                    # 写入并立即刷新
                    transform_line = f"{t_global[0]:.6f} {t_global[1]:.6f} {t_global[2]:.6f} {rx:.6f} {ry:.6f} {rz:.6f}\n"
                    f.write(transform_line)
                    f.flush()

                    # 将当前的转换矩阵保存到 numpy 数组中
                    transform_data.append([t_global[0], t_global[1], t_global[2], rx, ry, rz])

            # 将列表转换为 numpy 数组
            transform_array = np.array(transform_data)

            print(f"Transforms successfully saved to {transform_file}")
            return transform_array  # 返回 numpy 数组

        except IOError as e:
            print(f"Error saving transforms.txt: {e}")
            raise

    def interactive_translate_rotate_pathv8(self, pcd, path_pts, model_pcd=None, mesh_direction_pts=None,
                                            mesh_direction_ids=None, step=1.0):
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(window_name="动态演示刷牙轨迹")

        # 主点云
        vis.add_geometry(pcd)

        # 初始化路径线
        path_line = o3d.geometry.LineSet()
        path_line.points = o3d.utility.Vector3dVector(path_pts)
        path_line.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(path_pts) - 1)])
        path_line.colors = o3d.utility.Vector3dVector([[1, 0, 0]] * (len(path_pts) - 1))
        vis.add_geometry(path_line)

        # 添加模型点云（如果传入）
        model = None
        path_pts_adjust = []
        if model_pcd is not None and len(path_pts) >= 3:
            model = copy.deepcopy(model_pcd)  # 深度复制传入的点云
            # 将模型从第三个点平移到 path_pts[0] 位置
            move_vector = path_pts[0] - mesh_direction_pts[2]
            model.translate(move_vector)
            vis.add_geometry(model)

        # 定义旋转中心和方向
        mesh_center = np.asarray(mesh_direction_pts[2]) if mesh_direction_pts is not None else np.zeros(3)
        v1 = np.asarray(mesh_direction_pts[0]) if mesh_direction_pts is not None else np.zeros(3)
        v2 = np.asarray(mesh_direction_pts[1]) if mesh_direction_pts is not None else np.zeros(3)
        model_forward = v2 - v1
        model_forward /= np.linalg.norm(model_forward)

        # 计算方向
        directions = self.compute_directions(path_pts)

        # 定义回调函数
        def move(dx=0, dy=0, dz=0, rotate_angle=0):
            nonlocal path_pts, model

            # 获取当前视角参数
            ctr = vis.get_view_control()
            params = ctr.convert_to_pinhole_camera_parameters()

            translation = np.array([dx, dy, dz])
            path_pts += translation

            # 更新路径线
            path_line.points = o3d.utility.Vector3dVector(path_pts)
            vis.update_geometry(path_line)

            # 如果存在模型且路径至少有3个点，更新模型位置
            if model is not None and len(path_pts) >= 3:
                # 移除之前的模型
                vis.remove_geometry(model, reset_bounding_box=False)
                # 创建新的模型副本
                model = copy.deepcopy(model_pcd)
                # 计算模型需要移动的向量（对齐到path_pts[0]）
                move_vector = path_pts[0] - mesh_direction_pts[2]
                model.translate(move_vector)

                vis.add_geometry(model, reset_bounding_box=False)

            # 恢复视角参数
            ctr.convert_from_pinhole_camera_parameters(params)

            print(f"平移: dx={dx}, dy={dy}, dz={dz}, 旋转角度={rotate_angle}")

        def animate_model():
            nonlocal model, path_pts, is_animating

            if len(path_pts) < 3 or model is None:
                return

            is_animating = True
            total_points = len(path_pts)
            current_index = 0

            # 获取初始位置
            initial_position = mesh_direction_pts[2]

            while current_index < total_points and is_animating:
                # 计算移动向量
                target_position = path_pts[current_index]
                move_vector = target_position - initial_position

                # 更新模型位置
                vis.remove_geometry(model, reset_bounding_box=False)
                model = copy.deepcopy(model_pcd)
                model.translate(move_vector)

                # 更新旋转
                if mesh_direction_pts is not None:
                    prev_direction = mesh_direction_pts[1] - mesh_direction_pts[0]
                    current_direction = directions[current_index]
                    # print("current_direction :", current_direction)
                    rotation_matrix = self.rotation_from_vector_to_vector(prev_direction, current_direction)
                    # print("rotation_matrix :",rotation_matrix)
                    model = self.rotate_pointcloud(model, rotation_matrix, path_pts[current_index])

                vis.add_geometry(model, reset_bounding_box=False)

                current_index += 1
                time.sleep(0.00005)
                vis.poll_events()
                vis.update_renderer()
                vis.update_geometry(model)

            is_animating = False

        # 添加全局变量
        is_animating = False

        # 注册按键回调
        vis.register_key_callback(ord("S"), lambda v: move(dz=-step))
        vis.register_key_callback(ord("W"), lambda v: move(dz=step))
        vis.register_key_callback(ord("A"), lambda v: move(dx=-step))
        vis.register_key_callback(ord("D"), lambda v: move(dx=step))
        vis.register_key_callback(ord("Q"), lambda v: move(dy=step))
        vis.register_key_callback(ord("E"), lambda v: move(dy=-step))
        vis.register_key_callback(ord("R"), lambda v: move(rotate_angle=step))  # R键旋转
        vis.register_key_callback(ord("P"), lambda v: animate_model())  # P键触发动画
        vis.register_key_callback(256, lambda v: vis.close())  # ESC退出

        vis.run()
        vis.destroy_window()
        return path_pts, path_pts_adjust

    # @@@@@@@@@@@@@@@@@@@@@@@@@@@@改动的地方@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    # def interactive_translate_rotate_pathv18(self, pcd, path_pts, model_pcd=None, mesh_direction_pts=None,
    #                                         mesh_direction_ids=None, step=1.0):
    #     vis = o3d.visualization.VisualizerWithKeyCallback()
    #     vis.create_window(window_name="动态演示刷牙轨迹")
    #
    #     # 主点云
    #     vis.add_geometry(pcd)
    #
    #     # 初始化路径线
    #     path_line = o3d.geometry.LineSet()
    #     path_line.points = o3d.utility.Vector3dVector(path_pts)
    #     path_line.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(path_pts) - 1)])
    #     path_line.colors = o3d.utility.Vector3dVector([[1, 0, 0]] * (len(path_pts) - 1))
    #     vis.add_geometry(path_line)
    #
    #     # 轨迹纠正marker和刷牙向量初始化
    #     model = None
    #     path_pts_adjust = copy.deepcopy(path_pts)
    #     brushing_vectors = copy.deepcopy(path_pts)
    #     adjusted_last_vertexs = copy.deepcopy(path_pts)
    #     adjusted_second_last_vertexs = copy.deepcopy(path_pts)
    #
    #     if model_pcd is not None and len(path_pts) >= 3:
    #         model = copy.deepcopy(model_pcd)  # 深度复制传入的点云
    #         # 将模型从第三个点平移到 path_pts[0] 位置
    #         move_vector = path_pts[0] - mesh_direction_pts[2]
    #         model.translate(move_vector)
    #         vis.add_geometry(model)
    #
    #     # 定义旋转中心和方向
    #     mesh_center = np.asarray(mesh_direction_pts[2]) if mesh_direction_pts is not None else np.zeros(3)
    #     v1 = np.asarray(mesh_direction_pts[0]) if mesh_direction_pts is not None else np.zeros(3)
    #     v2 = np.asarray(mesh_direction_pts[1]) if mesh_direction_pts is not None else np.zeros(3)
    #     model_forward = v2 - v1
    #     model_forward /= np.linalg.norm(model_forward)
    #
    #     # 计算方向
    #     directions = self.compute_directions(path_pts)
    #
    #     # 计算路径点的几何中心
    #     def compute_center_of_points(points):
    #         return np.mean(np.array(points), axis=0)
    #
    #     # 定义回调函数
    #     def move(dx=0, dy=0, dz=0, scale_factor=1.0):
    #         nonlocal path_pts, model
    #
    #         # 获取当前视角参数
    #         ctr = vis.get_view_control()
    #         params = ctr.convert_to_pinhole_camera_parameters()
    #
    #         translation = np.array([dx, dy, dz])
    #         path_pts += translation
    #
    #         # 更新路径线
    #         path_line.points = o3d.utility.Vector3dVector(path_pts)
    #         vis.update_geometry(path_line)
    #
    #         # 如果存在模型且路径至少有3个点，更新模型位置
    #         if model is not None and len(path_pts) >= 3:
    #             # 移除之前的模型
    #             vis.remove_geometry(model, reset_bounding_box=False)
    #             # 创建新的模型副本
    #             model = copy.deepcopy(model_pcd)
    #             # 计算模型需要移动的向量（对齐到path_pts[0]）
    #             move_vector = path_pts[0] - mesh_direction_pts[2]
    #             model.translate(move_vector)
    #
    #             vis.add_geometry(model, reset_bounding_box=False)
    #
    #         # 恢复视角参数
    #         ctr.convert_from_pinhole_camera_parameters(params)
    #
    #         print(f"平移: dx={dx}, dy={dy}, dz={dz}, 缩放因子={scale_factor}")
    #
    #     def move_forward():
    #         nonlocal path_pts
    #         if len(path_pts) < 2:
    #             return
    #         direction = path_pts[1] - path_pts[0]  # 轨迹起点→第二点
    #         direction /= np.linalg.norm(direction)  # 归一化方向
    #         move(dx=direction[0] * step, dy=direction[1] * step, dz=direction[2] * step)
    #
    #     def move_backward():
    #         nonlocal path_pts
    #         if len(path_pts) < 2:
    #             return
    #         direction = path_pts[0] - path_pts[1]  # 往反方向走
    #         direction /= np.linalg.norm(direction)
    #         move(dx=direction[0] * step, dy=direction[1] * step, dz=direction[2] * step)
    #
    #
    #     def scale_along_center(scale_factor):
    #         nonlocal path_pts
    #
    #         # 轨迹中心
    #         center = compute_center_of_points(path_pts)
    #
    #         # ---🌟真正几何比例缩放，不变形---
    #         for i, pt in enumerate(path_pts):
    #             path_pts[i] = center + (pt - center) * scale_factor
    #
    #         # 更新渲染
    #         path_line.points = o3d.utility.Vector3dVector(path_pts)
    #         vis.update_geometry(path_line)
    #         print(f"路径整体等比缩放 × {scale_factor}")
    #
    #     def animate_model():
    #         # 添加path_pts_adjust变量和brushing_vectors变量
    #         nonlocal model, path_pts, is_animating, path_pts_adjust ,brushing_vectors,adjusted_last_vertexs,adjusted_second_last_vertexs
    #
    #         if len(path_pts) < 3 or model is None:
    #             return
    #
    #         is_animating = True
    #         total_points = len(path_pts)-1
    #         current_index = 0
    #
    #         # 获取初始位置
    #         initial_position = mesh_direction_pts[2]
    #
    #         while current_index < total_points and is_animating:
    #             # 计算移动向量
    #             target_position = path_pts[current_index]
    #             move_vector = target_position - initial_position
    #
    #             # 更新模型位置
    #             vis.remove_geometry(model, reset_bounding_box=False)
    #             model = copy.deepcopy(model_pcd)
    #             model.translate(move_vector)
    #
    #             # 更新旋转
    #             if mesh_direction_pts is not None:
    #                 prev_direction = mesh_direction_pts[1] - mesh_direction_pts[0]
    #                 current_direction = directions[current_index]
    #                 rotation_matrix = self.rotation_from_vector_to_vector(prev_direction, current_direction)
    #
    #                 # 旋转模型
    #                 model = self.rotate_pointcloud(model, rotation_matrix, path_pts[current_index])
    #
    #                 # 处理mesh_direction_ids最后一个点的位置
    #                 if mesh_direction_ids is not None and len(mesh_direction_ids) > 0:
    #                     # last_vertex_ids = mesh_direction_ids[-3]
    #                     last_vertex_ids = mesh_direction_ids[2]
    #                     adjusted_vertex = np.asarray(model.points)[last_vertex_ids]
    #                     path_pts_adjust[current_index] = adjusted_vertex
    #
    #                 #处理刷牙向量
    #                 if mesh_direction_ids is not None and len(mesh_direction_ids) > 0:
    #                     last_vertex_id = mesh_direction_ids[-1]
    #                     second_last_vertex_id = mesh_direction_ids[-2] if len(mesh_direction_ids) > 1 else None
    #
    #                     # 获取倒数第一点和倒数第二点的坐标
    #                     adjusted_last_vertex = np.asarray(model.points)[last_vertex_id]
    #                     adjusted_second_last_vertex = np.asarray(model.points)[
    #                         second_last_vertex_id] if second_last_vertex_id is not None else adjusted_last_vertex
    #
    #                     # 计算倒数第一点和倒数第二点之间的向量（刷牙向量）
    #                     brushing_vector = adjusted_last_vertex - adjusted_second_last_vertex
    #                     brushing_vectors[current_index] = brushing_vector  # 保存刷牙向量
    #                     adjusted_last_vertexs[current_index] = adjusted_last_vertex
    #                     adjusted_second_last_vertexs[current_index] = adjusted_second_last_vertex
    #
    #
    #
    #             # 添加更新后的模型
    #             vis.add_geometry(model, reset_bounding_box=False)
    #
    #             current_index += 1
    #             time.sleep(0.00005)
    #             vis.poll_events()
    #             vis.update_renderer()
    #             vis.update_geometry(model)
    #
    #         is_animating = False
    #
    #     # 添加全局变量
    #     is_animating = False
    #     vis.register_key_callback(ord("S"), lambda v: move(dx=0.017294, dy=0.016502, dz=0.999714))
    #     vis.register_key_callback(ord("W"), lambda v: move(dx=-0.017294, dy=-0.016502, dz=-0.999714))
    #     vis.register_key_callback(ord("A"), lambda v: move(dx=0.827884, dy=0.560404, dz=-0.023572))
    #     vis.register_key_callback(ord("D"), lambda v: move(dx=-0.827884, dy=-0.560404, dz=0.023572))
    #
    #     vis.register_key_callback(ord("Z"), lambda v: move(dx=0.560633, dy=-0.828055, dz=0.003970))  # 沿轨迹方向整体前推
    #     vis.register_key_callback(ord("X"), lambda v: move(dx=-0.560633, dy=0.828055, dz=-0.003970))  # 沿轨迹方向整体后退
    #
    #     vis.register_key_callback(ord("Q"), lambda v: scale_along_center(1.05))  # 放大5%
    #     vis.register_key_callback(ord("E"), lambda v: scale_along_center(0.95))  # 缩小5%
    #     vis.register_key_callback(ord("R"), lambda v: move(rotate_angle=step))  # R键旋转
    #     vis.register_key_callback(ord("P"), lambda v: animate_model())  # P键触发动画
    #     vis.register_key_callback(256, lambda v: vis.close())  # ESC退出
    #
    #     vis.run()
    #     vis.destroy_window()
    #     return path_pts ,path_pts_adjust ,brushing_vectors ,adjusted_last_vertexs ,adjusted_second_last_vertexs
    def interactive_translate_rotate_pathv18(self, pcd, path_pts, model_pcd=None, mesh_direction_pts=None,
                                             mesh_direction_ids=None, step=1.0):
        # 在创建可视化窗口之前，先保存初始状态的合并点云
        initial_combined_pcd = o3d.geometry.PointCloud()

        # 复制主点云
        pcd_copy = copy.deepcopy(pcd)
        initial_combined_pcd += pcd_copy

        # 如果存在模型点云，也添加到合并点云中
        if model_pcd is not None:
            model_copy = copy.deepcopy(model_pcd)
            # 如果提供了mesh_direction_pts，将模型平移到path_pts[0]位置（与可视化中的初始位置一致）
            if mesh_direction_pts is not None and len(path_pts) >= 3:
                move_vector = path_pts[0] - mesh_direction_pts[2]
                model_copy.translate(move_vector)
            initial_combined_pcd += model_copy

        # 保存合并的点云为PLY文件
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"D:/UsmileProject/hand_eye_calibration/initial_combined_cloud.ply"
        o3d.io.write_point_cloud(filename, initial_combined_pcd)
        print(f"已保存初始状态合并点云: {filename}")

        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(window_name="动态演示刷牙轨迹")

        # 主点云
        vis.add_geometry(pcd)

        # 初始化路径线
        path_line = o3d.geometry.LineSet()
        path_line.points = o3d.utility.Vector3dVector(path_pts)
        path_line.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(path_pts) - 1)])
        path_line.colors = o3d.utility.Vector3dVector([[1, 0, 0]] * (len(path_pts) - 1))
        vis.add_geometry(path_line)

        # 轨迹纠正marker和刷牙向量初始化
        model = None
        path_pts_adjust = copy.deepcopy(path_pts)
        brushing_vectors = copy.deepcopy(path_pts)
        adjusted_last_vertexs = copy.deepcopy(path_pts)
        adjusted_second_last_vertexs = copy.deepcopy(path_pts)

        if model_pcd is not None and len(path_pts) >= 3:
            model = copy.deepcopy(model_pcd)  # 深度复制传入的点云
            # 将模型从第三个点平移到 path_pts[0] 位置
            move_vector = path_pts[0] - mesh_direction_pts[2]
            model.translate(move_vector)
            vis.add_geometry(model)

        # 定义旋转中心和方向
        mesh_center = np.asarray(mesh_direction_pts[2]) if mesh_direction_pts is not None else np.zeros(3)
        v1 = np.asarray(mesh_direction_pts[0]) if mesh_direction_pts is not None else np.zeros(3)
        v2 = np.asarray(mesh_direction_pts[1]) if mesh_direction_pts is not None else np.zeros(3)
        model_forward = v2 - v1
        model_forward /= np.linalg.norm(model_forward)

        # 计算方向
        directions = self.compute_directions(path_pts)

        # 计算路径点的几何中心
        def compute_center_of_points(points):
            return np.mean(np.array(points), axis=0)

        # 定义回调函数
        def move(dx=0, dy=0, dz=0, scale_factor=1.0):
            nonlocal path_pts, model

            # 获取当前视角参数
            ctr = vis.get_view_control()
            params = ctr.convert_to_pinhole_camera_parameters()

            translation = np.array([dx, dy, dz])
            path_pts += translation

            # 更新路径线
            path_line.points = o3d.utility.Vector3dVector(path_pts)
            vis.update_geometry(path_line)

            # 如果存在模型且路径至少有3个点，更新模型位置
            if model is not None and len(path_pts) >= 3:
                # 移除之前的模型
                vis.remove_geometry(model, reset_bounding_box=False)
                # 创建新的模型副本
                model = copy.deepcopy(model_pcd)
                # 计算模型需要移动的向量（对齐到path_pts[0]）
                move_vector = path_pts[0] - mesh_direction_pts[2]
                model.translate(move_vector)

                vis.add_geometry(model, reset_bounding_box=False)

            # 恢复视角参数
            ctr.convert_from_pinhole_camera_parameters(params)

            print(f"平移: dx={dx}, dy={dy}, dz={dz}, 缩放因子={scale_factor}")

        def move_forward():
            nonlocal path_pts
            if len(path_pts) < 2:
                return
            direction = path_pts[1] - path_pts[0]  # 轨迹起点→第二点
            direction /= np.linalg.norm(direction)  # 归一化方向
            move(dx=direction[0] * step, dy=direction[1] * step, dz=direction[2] * step)

        def move_backward():
            nonlocal path_pts
            if len(path_pts) < 2:
                return
            direction = path_pts[0] - path_pts[1]  # 往反方向走
            direction /= np.linalg.norm(direction)
            move(dx=direction[0] * step, dy=direction[1] * step, dz=direction[2] * step)

        def scale_along_center(scale_factor):
            nonlocal path_pts

            # 轨迹中心
            center = compute_center_of_points(path_pts)

            # ---🌟真正几何比例缩放，不变形---
            for i, pt in enumerate(path_pts):
                path_pts[i] = center + (pt - center) * scale_factor

            # 更新渲染
            path_line.points = o3d.utility.Vector3dVector(path_pts)
            vis.update_geometry(path_line)
            print(f"路径整体等比缩放 × {scale_factor}")

        def animate_model():
            # 添加path_pts_adjust变量和brushing_vectors变量
            nonlocal model, path_pts, is_animating, path_pts_adjust, brushing_vectors, adjusted_last_vertexs, adjusted_second_last_vertexs

            if len(path_pts) < 3 or model is None:
                return

            is_animating = True
            total_points = len(path_pts) - 1
            current_index = 0

            # 获取初始位置
            initial_position = mesh_direction_pts[2]

            while current_index < total_points and is_animating:
                # 计算移动向量
                target_position = path_pts[current_index]
                move_vector = target_position - initial_position

                # 更新模型位置
                vis.remove_geometry(model, reset_bounding_box=False)
                model = copy.deepcopy(model_pcd)
                model.translate(move_vector)

                # 更新旋转
                if mesh_direction_pts is not None:
                    prev_direction = mesh_direction_pts[1] - mesh_direction_pts[0]
                    current_direction = directions[current_index]
                    rotation_matrix = self.rotation_from_vector_to_vector(prev_direction, current_direction)

                    # 旋转模型
                    model = self.rotate_pointcloud(model, rotation_matrix, path_pts[current_index])

                    # 处理mesh_direction_ids最后一个点的位置
                    if mesh_direction_ids is not None and len(mesh_direction_ids) > 0:
                        # last_vertex_ids = mesh_direction_ids[-3]
                        last_vertex_ids = mesh_direction_ids[2]
                        adjusted_vertex = np.asarray(model.points)[last_vertex_ids]
                        path_pts_adjust[current_index] = adjusted_vertex

                    # 处理刷牙向量
                    if mesh_direction_ids is not None and len(mesh_direction_ids) > 0:
                        last_vertex_id = mesh_direction_ids[-1]
                        second_last_vertex_id = mesh_direction_ids[-2] if len(mesh_direction_ids) > 1 else None

                        # 获取倒数第一点和倒数第二点的坐标
                        adjusted_last_vertex = np.asarray(model.points)[last_vertex_id]
                        adjusted_second_last_vertex = np.asarray(model.points)[
                            second_last_vertex_id] if second_last_vertex_id is not None else adjusted_last_vertex

                        # 计算倒数第一点和倒数第二点之间的向量（刷牙向量）
                        brushing_vector = adjusted_last_vertex - adjusted_second_last_vertex
                        brushing_vectors[current_index] = brushing_vector  # 保存刷牙向量
                        adjusted_last_vertexs[current_index] = adjusted_last_vertex
                        adjusted_second_last_vertexs[current_index] = adjusted_second_last_vertex

                # 添加更新后的模型
                vis.add_geometry(model, reset_bounding_box=False)

                current_index += 1
                time.sleep(0.00005)
                vis.poll_events()
                vis.update_renderer()
                vis.update_geometry(model)

            is_animating = False

        # 添加全局变量
        is_animating = False
        vis.register_key_callback(ord("S"), lambda v: move(dx=0.017294, dy=0.016502, dz=0.999714))
        vis.register_key_callback(ord("W"), lambda v: move(dx=-0.017294, dy=-0.016502, dz=-0.999714))
        vis.register_key_callback(ord("A"), lambda v: move(dx=0.827884, dy=0.560404, dz=-0.023572))
        vis.register_key_callback(ord("D"), lambda v: move(dx=-0.827884, dy=-0.560404, dz=0.023572))

        vis.register_key_callback(ord("Z"), lambda v: move(dx=0.560633, dy=-0.828055, dz=0.003970))  # 沿轨迹方向整体前推
        vis.register_key_callback(ord("X"), lambda v: move(dx=-0.560633, dy=0.828055, dz=-0.003970))  # 沿轨迹方向整体后退

        vis.register_key_callback(ord("Q"), lambda v: scale_along_center(1.05))  # 放大5%
        vis.register_key_callback(ord("E"), lambda v: scale_along_center(0.95))  # 缩小5%
        vis.register_key_callback(ord("R"), lambda v: move(rotate_angle=step))  # R键旋转
        vis.register_key_callback(ord("P"), lambda v: animate_model())  # P键触发动画
        vis.register_key_callback(256, lambda v: vis.close())  # ESC退出

        vis.run()
        vis.destroy_window()
        return path_pts, path_pts_adjust, brushing_vectors, adjusted_last_vertexs, adjusted_second_last_vertexs

    def save_pointcloud_along_path_as_plyv3(self, pcd, path_pts, mesh_direction_pts, mesh_direction_ids,
                                            output_folder="saved_ply"):
        """
        mesh_direction_pts: 旋转轴和中心的三个点坐标
        mesh_direction_ids: 对应点在点云中的索引，用于配准
        """
        import math

        os.makedirs(output_folder, exist_ok=True)
        original_pcd = copy.deepcopy(pcd)

        v1 = np.asarray(mesh_direction_pts[0])
        v2 = np.asarray(mesh_direction_pts[1])
        model_forward = v2 - v1
        model_forward /= np.linalg.norm(model_forward)
        mesh_center = np.asarray(mesh_direction_pts[2])

        directions = self.compute_directions(path_pts)

        transforms = []  # 保存 [tx, ty, tz, rx, ry, rz]

        for i, (pos, dir_vec) in enumerate(zip(path_pts, directions)):
            tmp_pcd = copy.deepcopy(pcd)
            if i == 0:
                filename = os.path.join(output_folder, f"frame_{i:04d}.ply")
                o3d.io.write_point_cloud(filename, tmp_pcd)
                transforms.append([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                continue

            # 基于 mesh_center 旋转
            R_local = self.rotation_from_vector_to_vector(model_forward, dir_vec)
            tmp_pcd = self.rotate_pointcloud(tmp_pcd, R_local, mesh_center)

            # 平移
            translation = pos - mesh_center
            tmp_pcd = self.translate_pointcloud(tmp_pcd, translation)

            # 保存 ply
            filename = os.path.join(output_folder, f"frame_{i:04d}.ply")
            o3d.io.write_point_cloud(filename, tmp_pcd)

            # 刚体变换
            src_pts = np.asarray(original_pcd.points)[mesh_direction_ids]
            dst_pts = np.asarray(tmp_pcd.points)[mesh_direction_ids]
            R_global, t_global = self.compute_rigid_transform(src_pts, dst_pts)
            t_global = translation

            rx, ry, rz = R.from_matrix(R_global).as_euler("zyx", degrees=True)
            transforms.append([t_global[0], t_global[1], t_global[2], rx, ry, rz])

        # === 去掉原来的第一行 ===
        transforms = transforms[1:]

        # === 后三列做差分（旋转差分）===
        new_transforms = []
        prev_r = None
        for i, (tx, ty, tz, rx, ry, rz) in enumerate(transforms):
            if i == 0:
                # 第一行保持原旋转
                new_transforms.append([tx, ty, tz, rx, ry, rz])
            else:
                drx = rx - prev_r[0]
                dry = ry - prev_r[1]
                drz = rz - prev_r[2]
                new_transforms.append([tx, ty, tz, drx, dry, drz])
            prev_r = (rx, ry, rz)

        # 写文件
        transform_file = os.path.join(output_folder, "transforms.txt")
        # with open(transform_file, "w") as f:
        #     for tx, ty, tz, rx, ry, rz in new_transforms:
        #         f.write(f"{tx:.6f} {ty:.6f} {tz:.6f} {rx:.6f} {ry:.6f} {rz:.6f}\n")
        with open(transform_file, "w") as f:
            for i, (tx, ty, tz, rx, ry, rz) in enumerate(new_transforms):
                if i == 0:
                    # 第一行的特殊处理
                    f.write(f"{tx:.6f} {ty:.6f} {tz:.6f} {0:.6f} {0:.6f} {0:.6f}\n")
                else:
                    # 其它行的处理
                    f.write(f"{tx:.6f} {ty:.6f} {tz:.6f} {rx:.6f} {ry:.6f} {rz:.6f}\n")
                    # f.write(f"{tx:.6f} {ty:.6f} {tz:.6f} {0.6:.6f} {ry:.6f} {rz:.6f}\n")

        print(f"Transforms saved to {transform_file}")


    def save_pointcloud_along_path_as_plyv6(self, pcd, path_pts, mesh_direction_pts, mesh_direction_ids,
                                            output_folder="saved_ply"):
        """
        mesh_direction_pts: 旋转轴和中心的三个点坐标
        mesh_direction_ids: 对应点在点云中的索引，用于配准
        """
        import math

        os.makedirs(output_folder, exist_ok=True)
        original_pcd = copy.deepcopy(pcd)

        v1 = np.asarray(mesh_direction_pts[0])
        v2 = np.asarray(mesh_direction_pts[1])
        model_forward = v2 - v1
        model_forward /= np.linalg.norm(model_forward)
        mesh_center = np.asarray(mesh_direction_pts[2])

        directions = self.compute_directions(path_pts)

        transforms = []  # 保存 [tx, ty, tz, rx, ry, rz]

        for i, (pos, dir_vec) in enumerate(zip(path_pts, directions)):
            tmp_pcd = copy.deepcopy(pcd)
            if i == 0:
                filename = os.path.join(output_folder, f"frame_{i:04d}.ply")
                o3d.io.write_point_cloud(filename, tmp_pcd)
                transforms.append([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                continue

            # 基于 mesh_center 旋转
            R_local = self.rotation_from_vector_to_vector(model_forward, dir_vec)
            tmp_pcd = self.rotate_pointcloud(tmp_pcd, R_local, mesh_center)

            # 平移
            translation = pos - mesh_center
            tmp_pcd = self.translate_pointcloud(tmp_pcd, translation)

            # 保存 ply
            filename = os.path.join(output_folder, f"frame_{i:04d}.ply")
            o3d.io.write_point_cloud(filename, tmp_pcd)

            # 刚体变换
            src_pts = np.asarray(original_pcd.points)[mesh_direction_ids]
            dst_pts = np.asarray(tmp_pcd.points)[mesh_direction_ids]
            R_global, t_global = self.compute_rigid_transform(src_pts, dst_pts)
            t_global = translation

            rx, ry, rz = R.from_matrix(R_global).as_euler("zyx", degrees=True)
            transforms.append([t_global[0], t_global[1], t_global[2], rx, ry, rz])

        # === 去掉原来的第一行 ===
        transforms = transforms[1:]

        # === 后三列做差分（旋转差分）===
        new_transforms = []
        prev_r = None
        for i, (tx, ty, tz, rx, ry, rz) in enumerate(transforms):
            if i == 0:
                # 第一行保持原旋转
                new_transforms.append([tx, ty, tz, rx, ry, rz])
            else:
                drx = rx - prev_r[3]
                dry = ry - prev_r[4]
                drz = rz - prev_r[5]
                dtx = rx - prev_r[0]
                dty = ry - prev_r[1]
                dtz = rz - prev_r[2]
                new_transforms.append([dtx, dty, dtz, drx, dry, drz])
            prev_r = (tx, ty, tz,rx, ry, rz)

        # 写文件
        transform_file = os.path.join(output_folder, "transforms6.txt")

        with open(transform_file, "w") as f:
            for i, (tx, ty, tz, rx, ry, rz) in enumerate(new_transforms):
                f.write(f"{tx:.6f} {ty:.6f} {tz:.6f} {rx:.6f} {ry:.6f} {rz:.6f}\n")

        print(f"Transforms saved to {transform_file}")

    def compute_rigid_transform(self, src, dst):
        """
        用 Open3D 的 PointToPoint 直接计算刚体变换 (R, t)
        src, dst: np.ndarray(N, 3) 一一对应的点
        """
        import open3d as o3d
        import numpy as np

        assert src.shape == dst.shape, "src 和 dst 必须形状相同"

        # 构造点云
        src_pcd = o3d.geometry.PointCloud()
        dst_pcd = o3d.geometry.PointCloud()
        src_pcd.points = o3d.utility.Vector3dVector(src)
        dst_pcd.points = o3d.utility.Vector3dVector(dst)

        # 构造对应关系（0→0, 1→1, ..., N→N）
        corr = np.array([[i, i] for i in range(src.shape[0])], dtype=np.int32)

        # 计算变换矩阵
        p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        trans_init = p2p.compute_transformation(
            src_pcd, dst_pcd,
            o3d.utility.Vector2iVector(corr)
        )

        # 拆出旋转和平移
        R = trans_init[:3, :3]
        t = trans_init[:3, 3]

        return R, t

    def save_model_along_path_as_objs_custom(self, mesh, path_pts, mesh_direction_pts, output_folder="saved_objs"):
        os.makedirs(output_folder, exist_ok=True)
        original_mesh = copy.deepcopy(mesh)
        v1 = np.asarray(mesh_direction_pts[0])
        v2 = np.asarray(mesh_direction_pts[1])
        model_forward = v2 - v1
        model_forward /= np.linalg.norm(model_forward)
        mesh_center = np.asarray(mesh_direction_pts[2])
        directions = self.compute_directions(path_pts)

        for i, (pos, dir_vec) in enumerate(zip(path_pts, directions)):
            tmp_mesh = copy.deepcopy(original_mesh)
            if i == 0:
                filename = os.path.join(output_folder, f"frame_{i:04d}.obj")
                o3d.io.write_triangle_mesh(filename, tmp_mesh)
                print(f"Saved {filename}")
                continue
            R = self.rotation_from_vector_to_vector(model_forward, dir_vec)
            tmp_mesh.rotate(R, center=mesh_center)
            tmp_mesh.translate(pos - mesh_center, relative=True)
            filename = os.path.join(output_folder, f"frame_{i:04d}.obj")
            o3d.io.write_triangle_mesh(filename, tmp_mesh)
            print(f"Saved {filename}")

    def apply_transforms_and_save_pointclouds(
            self, base_pcd_path, transforms_txt, output_folder="./transformed_pcds"
    ):
        """
        根据 transforms.txt 文件，对加载的点云依次做刚性变换并保存
        transforms.txt 每行: tx ty tz rx ry rz (角度，ZYX顺序)
        """
        os.makedirs(output_folder, exist_ok=True)

        # 1. 加载点云
        _, base_pcd = self.load_pointcloud(base_pcd_path)
        base_points = np.asarray(base_pcd.points)
        base_colors = np.asarray(base_pcd.colors) if base_pcd.has_colors() else None

        # 2. 读取 transforms.txt
        transforms = np.loadtxt(transforms_txt)
        if transforms.ndim == 1:
            transforms = transforms[np.newaxis, :]  # 只有一行的情况

        for i, tf in enumerate(transforms):
            tx, ty, tz, rx, ry, rz = tf

            # 3. 构造旋转矩阵
            R_mat = R.from_euler("zyx", [rx, ry, rz], degrees=True).as_matrix()

            # 4. 应用刚性变换
            transformed_points = (base_points @ R_mat.T) + np.array([tx, ty, tz])

            # 5. 保存
            transformed_pcd = o3d.geometry.PointCloud()
            transformed_pcd.points = o3d.utility.Vector3dVector(transformed_points)
            if base_colors is not None:
                transformed_pcd.colors = o3d.utility.Vector3dVector(base_colors)

            filename = os.path.join(output_folder, f"frame_{i:04d}.ply")
            o3d.io.write_point_cloud(filename, transformed_pcd)
            print(f"Saved {filename}")

class FaithfulPCDRegistrar:
    """
    支持 p1, p2 都为点云；
    - p1 使用沿某轴的渐变色（默认沿 z 轴），
    - p2 使用部分染色（前 color_n 个点），
    - 保持交互选点流程与之前一致。

    用法:
      r = FaithfulPCDRegistrar(pcd1, pcd2, save_debug_dir="debug")
      r.run_interactive()
    """

    def __init__(self, pcd1, pcd2, save_debug_dir=None,
                 nb_neighbors=16, std_ratio=2.0, color_n=10000,
                 icp_thresh=0.30, refine_thresh=1.0, gradient_axis=2):
        self.pcd1 = pcd1  # ptoothscan 点云
        self.pcd2 = pcd2  # ptooth 点云
        self.save_debug_dir = save_debug_dir
        if save_debug_dir:
            os.makedirs(save_debug_dir, exist_ok=True)

        self.nb_neighbors = nb_neighbors
        self.std_ratio = std_ratio
        self.color_n = color_n
        self.icp_thresh = float(icp_thresh)
        self.refine_thresh = float(refine_thresh)
        # gradient_axis: 0=x,1=y,2=z
        self.gradient_axis = int(gradient_axis)

        # placeholders
        self.ind1 = None
        self.ind2 = None
        self.shift_to_align = None
        self.pcd2_shifted = None

    # ---------- minimal helper: color first n points ----------
    @staticmethod
    def color_partial(pcd, color, n=1000):
        points = np.asarray(pcd.points)
        N = len(points)
        if pcd.has_colors():
            colors = np.asarray(pcd.colors)
            if len(colors) != N:
                colors = np.ones((N, 3), dtype=float) * 0.7
        else:
            colors = np.ones((N, 3), dtype=float) * 0.7
        colors[:min(n, N)] = color
        pcd.colors = o3d.utility.Vector3dVector(colors)
        return pcd

    # ---------- gradient color for a pointcloud ----------
    def apply_gradient(self, pcd, axis=2):
        pts = np.asarray(pcd.points)
        if pts.size == 0:
            return pcd
        vals = pts[:, axis]
        vmin = vals.min()
        vmax = vals.max()
        if np.isclose(vmin, vmax):
            t = np.zeros_like(vals)
        else:
            t = (vals - vmin) / (vmax - vmin)
        # simple colormap: blue -> cyan -> yellow -> red (approx)
        # map t in [0,1] to RGB
        r = np.minimum(2 * t, 1.0)
        g = np.minimum(2 * np.abs(t - 0.5), 1.0)
        b = np.minimum(2 * (1 - t), 1.0)
        colors = np.vstack([r, g, b]).T
        pcd.colors = o3d.utility.Vector3dVector(colors)
        return pcd

    # ---------- load and filter points directly from pcd objects ----------
    def load_and_filter(self):
        # print("处理点云 p1, p2 ...")
        # print(f"原始点数 pcd1: {len(self.pcd1.points)}, pcd2: {len(self.pcd2.points)}")

        # 对 p1 使用渐变色（沿指定轴）
        self.apply_gradient(self.pcd2, axis=self.gradient_axis)

        # 对 p2 使用前 color_n 个点染色为蓝色
        self.color_partial(self.pcd1, [0, 0, 1], n=self.color_n)

    # ---------- initial shift ----------
    def make_initial_shift(self):
        center1 = self.pcd1.get_center()
        center2 = self.pcd2.get_center()
        self.shift_to_align = center1 - center2
        self.pcd2_shifted = o3d.geometry.PointCloud(self.pcd2)
        self.pcd2_shifted.translate(self.shift_to_align, relative=True)
        self.pcd2_shifted.translate([30, 0, 0], relative=True)
        # print("center1:", center1)
        # print("center2:", center2)
        # print("shift_to_align:", self.shift_to_align)

        # if self.save_debug_dir:
        #     np.savetxt(os.path.join(self.save_debug_dir, "center1.txt"), np.array(center1).reshape(1, 3))
        #     np.savetxt(os.path.join(self.save_debug_dir, "center2.txt"), np.array(center2).reshape(1, 3))
        #     np.savetxt(os.path.join(self.save_debug_dir, "shift_to_align.txt"),
        #                np.array(self.shift_to_align).reshape(1, 3))

    def make_initial_shiftzero(self):
        center1 = self.pcd1.get_center()
        center2 = self.pcd2.get_center()
        self.shift_to_align = center1 - center2
        self.pcd2_shifted = o3d.geometry.PointCloud(self.pcd2)
        self.pcd2_shifted.translate(self.shift_to_align, relative=True)
        self.pcd2_shifted.translate([0, 0, 0], relative=True)

    # ---------- interactive pick ----------
    def pick_correspondences(self, window_name="Pick Correspondences"):
        # fused used for picking should be pcd1 + pcd2_shifted so indices are coherent
        fused = self.pcd1 + self.pcd2_shifted

        print("打开点云选择窗口，请按原来习惯选点（先 pcd1 的点，然后 pcd2 的点），完成后按 Q。")
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name=window_name)
        vis.add_geometry(fused)
        vis.run()
        vis.destroy_window()
        picked_ids = vis.get_picked_points()
        print("Picked indices:", picked_ids)
        if self.save_debug_dir:
            np.savetxt(os.path.join(self.save_debug_dir, "picked_indices.txt"), np.array(picked_ids, dtype=np.int32),
                       fmt="%d")
        return picked_ids, fused

    # ---------- compute initial transform ----------
    def compute_trans_init_from_picks(self, picked_ids, fused,mode):
        if len(picked_ids) == 0:
            raise RuntimeError("未选取点 (picked_ids 为空)。")

        N = len(picked_ids) // 2
        ids1 = picked_ids[:N]
        ids2 = picked_ids[N:2 * N]
        points = np.asarray(fused.points)

        target_points = points[ids1]
        # 还原 pcd2 的原始坐标：减去 X 轴平移(100) 与 center 对齐移动(shift_to_align)
        if(mode ==0):
            source_points = points[ids2] - np.array([30, 0, 0]) - self.shift_to_align
        else:
            source_points = points[ids2] - np.array([0, 0, 0]) - self.shift_to_align
        src_corr = o3d.geometry.PointCloud()
        tgt_corr = o3d.geometry.PointCloud()
        src_corr.points = o3d.utility.Vector3dVector(source_points)
        tgt_corr.points = o3d.utility.Vector3dVector(target_points)

        corr = np.zeros((N, 2), dtype=np.int32)
        corr[:, 0] = np.arange(N, dtype=np.int32)
        corr[:, 1] = np.arange(N, dtype=np.int32)

        p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        trans_init = p2p.compute_transformation(src_corr, tgt_corr, o3d.utility.Vector2iVector(corr))

        # if self.save_debug_dir:
        #     np.savetxt(os.path.join(self.save_debug_dir, "trans_init.txt"), trans_init.reshape(4, 4))
        return trans_init

    # ---------- ICP ----------
    def run_icp_with_init(self, trans_init):
        reg_p2p = o3d.pipelines.registration.registration_icp(
            self.pcd2, self.pcd1, self.icp_thresh, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )

        pcd2_aligned = copy.deepcopy(self.pcd2)
        pcd2_aligned.transform(reg_p2p.transformation)

        # 不再使用这个后面的优化
        reg_refine = o3d.pipelines.registration.registration_icp(
            pcd2_aligned, self.pcd1, self.refine_thresh, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )

        pcd2_aligned_refined = copy.deepcopy(pcd2_aligned)
        pcd2_aligned_refined.transform(reg_refine.transformation)

        # fused_final = pcd1_aligned_refined + self.pcd2
        # o3d.visualization.draw_geometries([fused_final], window_name="Fused Point Cloud (After ICP Refinement)")

        return pcd2_aligned_refined
        # return pcd2_aligned

    def run_multiple_icp(self, trans_init, num_iterations=3):
        pcd2_aligned = copy.deepcopy(self.pcd2)

        # 初始配准
        reg_p2p = o3d.pipelines.registration.registration_icp(
            self.pcd2, self.pcd1, self.icp_thresh, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )

        pcd2_aligned.transform(reg_p2p.transformation)
        last_transformation = reg_p2p.transformation

        for i in range(num_iterations):
            print(f"Running ICP iteration {i + 1}...")

            # 每次迭代都用上次的变换结果作为初始化
            reg_refine = o3d.pipelines.registration.registration_icp(
                pcd2_aligned, self.pcd1, self.refine_thresh, last_transformation,
                o3d.pipelines.registration.TransformationEstimationPointToPoint()
            )

            pcd2_aligned.transform(reg_refine.transformation)
            last_transformation = reg_refine.transformation

        # 最终配准后的点云
        pcd2_aligned_refined = copy.deepcopy(pcd2_aligned)
        pcd2_aligned_refined.transform(last_transformation)

        # 返回最终配准的点云
        return pcd2_aligned_refined

    # ---------- one-line runner ----------
    def run_interactive(self,mode=0):
        self.load_and_filter()
        if(mode == 0):
            self.make_initial_shift()
        else:
            self.make_initial_shiftzero()
        picked_ids, fused = self.pick_correspondences()
        trans_init = self.compute_trans_init_from_picks(picked_ids, fused,mode)

        # print("trans init",trans_init)
        # reg_p2p, reg_refine = self.run_icp_with_init(trans_init)
        reg_refine = self.run_icp_with_init(trans_init)
        # reg_refine = self.run_multiple_icp(trans_init,3)
        # print("全部完成。若仍与原始脚本结果不同，请把 debug 文件夹内内容贴给我方便排查。")
        # return {
        #     "picked_indices": picked_ids,
        #     "trans_init": trans_init,
        #     "reg_p2p": getattr(reg_p2p, "transformation", None),
        #     "reg_refine": getattr(reg_refine, "transformation", None)
        # }
        return reg_refine




def scantobasesinglev2(ply_path, gripper_pose):
    """
    将3D点坐标从相机坐标系转换到机械臂基坐标系（不使用平移），
    同时保留原始点云的颜色信息，并只保留 z 坐标在 [-60, 60] 范围内的点云。

    参数：
        ply_path: str，PLY文件路径
        gripper_pose: 机械臂末端姿态 [x, y, z, rx, ry, rz] (单位 mm 和 deg)

    返回：
        o3d.geometry.PointCloud，变换后的点云（保留颜色并过滤z范围外的点）
    """

    # 1. 加载 PLY 文件并获取点云数据
    pcd = o3d.io.read_point_cloud(ply_path)
    pts_3d = np.asarray(pcd.points)  # 点云的 3D 坐标
    colors = np.asarray(pcd.colors)  # 点云的颜色信息

    if len(pts_3d) == 0:
        return o3d.geometry.PointCloud()

    # 2. 如果存在 nan 点，先过滤
    valid_mask = ~np.isnan(pts_3d).any(axis=1)
    pts_3d = pts_3d[valid_mask]
    colors = colors[valid_mask]  # 过滤后的颜色数据

    # 3. 相机坐标到 gripper 坐标的旋转（手眼标定）
    R_cam2gripper = np.array([
        [0.09884656, -0.99501953, -0.0128642],
        [0.99358876, 0.09940076, -0.05385968],
        [0.05487015, -0.00745788, 0.99846565]
    ])
    T_cam2gripper = np.eye(4)
    T_cam2gripper[:3, :3] = R_cam2gripper

    # 4. gripper 坐标到 base 坐标的旋转（末端姿态）
    r = R.from_euler('xyz', gripper_pose[3:], degrees=True)
    R_gripper2base = r.as_matrix()
    T_gripper2base = np.eye(4)
    T_gripper2base[:3, :3] = R_gripper2base

    # 5. 总变换矩阵
    T_cam2base = T_gripper2base @ T_cam2gripper

    # 6. 应用变换
    pts_h = np.hstack([pts_3d, np.ones((pts_3d.shape[0], 1))])  # 变为齐次坐标
    transformed_pts = (T_cam2base @ pts_h.T).T[:, :3]

    # 7. 只保留 z 坐标在 [-60, 60] 范围内的点
    z_mask = (transformed_pts[:, 2] >= -180) & (transformed_pts[:, 2] <= -120)
    transformed_pts = transformed_pts[z_mask]
    transformed_colors = colors[z_mask]

    # 8. 返回 Open3D 点云对象并保持颜色信息
    transformed_pcd = o3d.geometry.PointCloud()
    transformed_pcd.points = o3d.utility.Vector3dVector(transformed_pts)
    transformed_pcd.colors = o3d.utility.Vector3dVector(transformed_colors)

    return transformed_pcd
# ============ 示例调用 ===============
def save_raw_path_pts_to_npy(raw_path_pts, file_path):
    np.save(file_path, raw_path_pts)  # 保存为 .npy 格式
    print(f"Path points saved to {file_path}")

def load_raw_path_pts_from_npy(file_path):
    raw_path_pts = np.load(file_path)  # 加载 .npy 文件
    print(f"Path points loaded from {file_path}")
    return raw_path_pts


def register_pointclouds_interactive(pcd1, pcd2, translation_step=0.05, rotation_step=0.05):
    """
    通过交互式操作将pcd2配准到pcd1

    参数:
    pcd1: 目标点云
    pcd2: 需要变换的点云
    translation_step: 平移步长
    rotation_step: 旋转步长(角度)

    返回:
    transformed_pcd2: 变换后的pcd2
    transformation: 应用的变换矩阵
    """

    # 复制点云以避免修改原始数据
    pcd1_copy = pcd1
    pcd2_copy = pcd2

    # 为区分两个点云，设置不同颜色
    # pcd1_copy.paint_uniform_color([1, 0, 0])  # 红色 - 目标点云
    # pcd2_copy.paint_uniform_color([0, 1, 0])  # 绿色 - 待变换点云

    # 初始变换矩阵（单位矩阵）
    transformation = np.eye(4)

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="点云配准 - 按ESC退出")

    # 添加点云到可视化
    vis.add_geometry(pcd1_copy)
    vis.add_geometry(pcd2_copy)

    def update_pcd2():
        """更新pcd2的变换"""
        pcd2_copy.transform(transformation)
        vis.update_geometry(pcd2_copy)
        # 重置变换矩阵为单位矩阵，因为点云已经应用了变换
        transformation[:] = np.eye(4)

    def get_rotation_matrix(axis, angle_degrees):
        """获取绕指定轴的旋转矩阵"""
        angle_rad = np.radians(angle_degrees)
        if axis == 'x':
            R = np.array([[1, 0, 0],
                          [0, np.cos(angle_rad), -np.sin(angle_rad)],
                          [0, np.sin(angle_rad), np.cos(angle_rad)]])
        elif axis == 'y':
            R = np.array([[np.cos(angle_rad), 0, np.sin(angle_rad)],
                          [0, 1, 0],
                          [-np.sin(angle_rad), 0, np.cos(angle_rad)]])
        elif axis == 'z':
            R = np.array([[np.cos(angle_rad), -np.sin(angle_rad), 0],
                          [np.sin(angle_rad), np.cos(angle_rad), 0],
                          [0, 0, 1]])
        return R

    # --- 平移函数 ---
    def translate(dx=0, dy=0, dz=0):
        nonlocal transformation
        T = np.eye(4)
        T[:3, 3] = [dx, dy, dz]
        transformation = T @ transformation
        update_pcd2()

    # --- 旋转函数 ---
    def rotate(axis, direction=1):
        nonlocal transformation
        R = get_rotation_matrix(axis, rotation_step * direction)
        T = np.eye(4)
        T[:3, :3] = R
        transformation = T @ transformation
        update_pcd2()

    # --- 重置函数 ---
    def reset_transform():
        nonlocal transformation
        # 重置pcd2到初始状态
        pcd2_copy.points = pcd2.points
        transformation = np.eye(4)
        vis.update_geometry(pcd2_copy)
        print("已重置变换")

    # --- 键盘绑定 ---

    # 平移绑定：WASD + RF (上下)
    vis.register_key_callback(ord("D"), lambda vis: translate(dx=translation_step))  # +X
    vis.register_key_callback(ord("A"), lambda vis: translate(dx=-translation_step))  # -X
    vis.register_key_callback(ord("W"), lambda vis: translate(dy=translation_step))  # +Y
    vis.register_key_callback(ord("S"), lambda vis: translate(dy=-translation_step))  # -Y
    vis.register_key_callback(ord("Q"), lambda vis: translate(dz=translation_step))  # +Z
    vis.register_key_callback(ord("E"), lambda vis: translate(dz=-translation_step))  # -Z

    # 旋转绑定：J/L (绕Z), I/K (绕Y), U/O (绕X)
    vis.register_key_callback(ord("J"), lambda vis: rotate('z', direction=1))  # 绕Z+
    vis.register_key_callback(ord("L"), lambda vis: rotate('z', direction=-1))  # 绕Z-
    vis.register_key_callback(ord("I"), lambda vis: rotate('y', direction=1))  # 绕Y+
    vis.register_key_callback(ord("K"), lambda vis: rotate('y', direction=-1))  # 绕Y-
    vis.register_key_callback(ord("U"), lambda vis: rotate('x', direction=1))  # 绕X+
    vis.register_key_callback(ord("O"), lambda vis: rotate('x', direction=-1))  # 绕X-

    # 重置绑定：T
    vis.register_key_callback(ord("T"), lambda vis: reset_transform())

    print("=" * 50)
    print("点云配准控制说明:")
    print("平移: D/A (X轴), W/S (Y轴), R/F (Z轴)")
    print("旋转: J/L (绕Z轴), I/K (绕Y轴), U/O (绕X轴)")
    print("重置: T")
    print("退出: ESC")
    print("=" * 50)

    vis.run()
    vis.destroy_window()

    # 应用最终的变换到原始pcd2
    final_pcd2 = pcd2
    final_pcd2.transform(transformation)

    return final_pcd2, transformation

def save_mesh_direction_ids(mesh_direction_ids, save_path="mesh_direction_ids.txt"):
    mesh_direction_ids = np.array(mesh_direction_ids, dtype=np.int32)
    np.savetxt(save_path, mesh_direction_ids, fmt="%d")
    print(f"[INFO] mesh_direction_ids 已保存到 {save_path}")

def load_mesh_direction_ids(load_path="mesh_direction_ids.txt"):
    mesh_direction_np = np.loadtxt(load_path, dtype=np.int32)
    print(f"[INFO] 已从 {load_path} 加载 mesh_direction_ids")
    return mesh_direction_np

# def load_picked_path_by_keyboard():
#     print("请选择要加载的轨迹点路径:")
#     print("1: 牙外侧" ,"2: 牙上侧" ,"3: 牙内侧")
#
#
#     choice = input("请输入数字 (1/2/3): ").strip()
#
#     # ===== 三个轨迹路径 =====
#     base_dir = "D:/UsmileProject/hand_eye_calibration"
#
#     path_map = {
#         "1": os.path.join(base_dir, "outer", "picked_path.npy"),     # 牙外侧
#         "2": os.path.join(base_dir, "upper", "picked_path.npy"),     # 牙上侧
#         "3": os.path.join(base_dir, "inner", "picked_path.npy")      # 牙内侧
#     }
#
#     if choice not in path_map:
#         raise ValueError("输入错误！只能输入 1 / 2 / 3")
#
#     path_file = path_map[choice]
#
#     if not os.path.exists(path_file):
#         raise FileNotFoundError(f"文件不存在: {path_file}")
#
#     print(f"加载轨迹文件: {path_file}")
#
#     picked_path = np.load(path_file)
#     return picked_path

def load_picked_path_by_keyboard(param=None):
    """
    加载轨迹点路径
    Args:
        param: 整数参数，0-6
              0,1: 加载upper
              2,3: 加载outer
              4,5,6: 加载inner
    """
    if param is not None:
        # 根据参数值确定要加载的路径
        if param in [0, 1]:
            path_type = "upper"
            choice_display = "2"
        elif param in [2, 3 ,7]:
            path_type = "outer"
            choice_display = "1"
        elif param in [4, 5, 6]:
            path_type = "inner"
            choice_display = "3"
        else:
            raise ValueError(f"参数错误！参数 {param} 不在有效范围 0-6 内")

        print(f"根据参数 {param} 自动选择加载: {path_type}")
        choice = choice_display
    else:
        # 如果没有参数，则使用交互式输入
        print("请选择要加载的轨迹点路径:")
        print("1: 牙外侧 (outer)", "2: 牙上侧 (upper)", "3: 牙内侧 (inner)")
        choice = input("请输入数字 (1/2/3): ").strip()

    # ===== 三个轨迹路径 =====
    base_dir = "D:/UsmileProject/hand_eye_calibration"

    path_map = {
        "1": os.path.join(base_dir, "outer", "picked_path.npy"),  # 牙外侧
        "2": os.path.join(base_dir, "upper", "picked_path.npy"),  # 牙上侧
        "3": os.path.join(base_dir, "inner", "picked_path.npy")  # 牙内侧
    }

    if choice not in path_map:
        raise ValueError("输入错误！只能输入 1 / 2 / 3")

    path_file = path_map[choice]

    if not os.path.exists(path_file):
        raise FileNotFoundError(f"文件不存在: {path_file}")

    print(f"加载轨迹文件: {path_file}")

    picked_path = np.load(path_file)
    return picked_path
# ============ 示例调用 ===============

def pick_three_paths(processor, selected):
    """
    连续执行三次点选 + 拟合 + 保存
    保存路径分别为 outer / upper / inner
    """

    names = ["outer", "upper", "inner"]
    base_dir = "D:/UsmileProject/hand_eye_calibration/"

    for name in names:
        print(f"\n====== 现在开始选择: {name} 点 ======")

        # 1. 选点
        raw_path_pts, picked_ids = processor.pick_vertex_indices_from_pointcloud(selected)
        print(f"[{name}] 选点数量: {len(raw_path_pts)}")

        if len(raw_path_pts) < 3:
            print(f"[{name}] 选点太少，跳过拟合！")
            continue

        # 2. 拟合曲线
        picked_path = processor.fit_poly_curve_xys(raw_path_pts, num_samples=24)

        # 3. 保存路径
        save_path = f"{base_dir}/{name}/picked_path.npy"
        np.save(save_path, picked_path)
        print(f"[{name}] 已保存到: {save_path}")

    print("\n=== 已完成 outer / upper / inner 三条路径的选择并保存 ===")


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
    pcd2 = "D:/UsmileProject/hand_eye_calibration/mergeredoldboard.ply"
    pcd2 = o3d.io.read_point_cloud(pcd2)

    rotation_param = None
    subfolder = ""  # 默認空文件夾

    if len(sys.argv) > 1:
        try:
            param = int(sys.argv[1])
            rotation_param = param
            # 獲取對應的子文件夾名稱
            subfolder = get_subfolder_by_param(param)
            print(f"Using rotation parameter: {param}, subfolder: {subfolder}")
        except ValueError:
            print("參數必須是整數")
            sys.exit(1)

    # 是否需要旋轉
    selectedbrush2 = rotate_point_cloud_with_selected_axis(
        "D:/UsmileProject/hand_eye_calibration/Initmergeredbrush.ply",
        rotation_param=rotation_param
    )

    processor = BrushPathProcessor()
    import sys

    if len(sys.argv) > 1:
        try:
            param = int(sys.argv[1])
            picked_path = load_picked_path_by_keyboard(param)
        except ValueError:
            print("參數必須是整數")
            sys.exit(1)
    else:
        picked_path = load_picked_path_by_keyboard()

    picked_path = processor.interactive_pick_path_points_v3(pcd2, picked_path)
    if (int(sys.argv[1]) != 7):
        picked_path = processor.smooth_and_sample_path(picked_path, smooth_factor=0.6, num_samples=36)
        # picked_path = processor.smooth_and_sample_pathv21(picked_path, num_samples=36)
    else:
        picked_path = processor.smooth_and_sample_path(picked_path, smooth_factor=0.6, num_samples=36)
    mesh_direction_ids = load_mesh_direction_ids("D:/UsmileProject/hand_eye_calibration/mesh_direction_ids.txt")
    points_np = np.asarray(selectedbrush2.points)
    mesh_direction_pts = points_np[mesh_direction_ids]

    # 根據參數創建不同的保存目錄
    if subfolder:
        save_dir = f"{DEFAULTCONFIG_DIR}/{subfolder}/"
    else:
        save_dir = "D:/UsmileProject/hand_eye_calibration/"

    os.makedirs(save_dir, exist_ok=True)
    print(f"Saving files to: {save_dir}")

    if mesh_direction_pts is not None:
        mesh_pts_path = os.path.join(save_dir, 'mesh_direction_pts.npy')
        np.save(mesh_pts_path, mesh_direction_pts)
    else:
        print("mesh_direction_pts is None, cannot save")

    if picked_path is not None:
        picked_path_path = os.path.join(save_dir, 'picked_path.npy')
        np.save(picked_path_path, picked_path)
        if(int(sys.argv[1]) == 7):
            picked_path_path2 = os.path.join(save_dir, 'picked_path.txt')
            np.savetxt(picked_path_path2, picked_path)
    else:
        print("picked_path is None, cannot save")

    adjusted_path = picked_path

    # print("adjusted_path",adjusted_path)

    ply_path_we = "D:/UsmileProject/hand_eye_calibration/mergeredoldboard.ply"
    app = BrushBuilderGUI(picked_path, ply_path_we,subfolder)
    app.run()

    # 上右(rightup=0)/上左(leftup=1) 不需要选择浮刷点(support_points)，跳过该步骤；
    # 其余区域(如侧面 side)仍正常选择浮刷点，避免影响其浮刷流程。
    if rotation_param not in (0, 1):
        ply_path_we = "D:/UsmileProject/hand_eye_calibration/mergeredoldboard.ply"
        app2 = SupportPointSelector(picked_path, ply_path_we, subfolder)
        app2.run()
    else:
        print(f"[INFO] 区域 {subfolder} 无需选择浮刷点(support_points)，已跳过。")

    # 根據參數設置輸出文件夾
    if subfolder:
        output_folder = save_dir
    else:
        output_folder = "saved_ply"

    typeid_value = int(sys.argv[1])

    transformArray = processor.save_pointcloud_along_path_as_plyv21(selectedbrush2, adjusted_path, mesh_direction_pts,
                                                                    mesh_direction_ids,
                                                                    output_folder=output_folder,typeid = typeid_value)