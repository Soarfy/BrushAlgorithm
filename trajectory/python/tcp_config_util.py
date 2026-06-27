import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trajectory_paths import DEFAULTCONFIG_DIR

# 新标定 TCP 默认值（无 tcp.json 或 json 为 0,0,0 时使用）
DEFAULT_TCP_X = -9.352824
DEFAULT_TCP_Y = -186.998296
DEFAULT_TCP_Z = 224.724733

TCP_JSON_PATH = os.path.join(DEFAULTCONFIG_DIR, "tcp.json")


def _is_unset_tcp(x, y, z):
    return abs(x) < 1e-9 and abs(y) < 1e-9 and abs(z) < 1e-9


def load_brush_tcp_xyz(json_path=None):
    """从 tcp.json 读取 x/y/z；文件不存在、读取失败或为 0,0,0 时使用 DEFAULT_TCP_*。"""
    path = json_path or TCP_JSON_PATH
    x, y, z = DEFAULT_TCP_X, DEFAULT_TCP_Y, DEFAULT_TCP_Z
    try:
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            jx = float(data.get("x", x))
            jy = float(data.get("y", y))
            jz = float(data.get("z", z))
            if not _is_unset_tcp(jx, jy, jz):
                x, y, z = jx, jy, jz
    except Exception:
        pass
    return x, y, z
