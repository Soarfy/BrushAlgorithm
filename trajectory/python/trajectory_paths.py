"""轨迹脚本与 defaultconfig 路径（相对本仓库根目录）。"""
import os

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(_SCRIPT_DIR))
DEFAULTCONFIG_DIR = os.path.join(REPO_ROOT, "defaultconfig")

# PLY/标定辅助数据目录（默认原 hand_eye_calibration，可用环境变量 HAND_EYE_DATA_DIR 覆盖）
HAND_EYE_DATA_DIR = os.environ.get(
    "HAND_EYE_DATA_DIR", r"D:/UsmileProject/hand_eye_calibration"
)


def region_config_dir(subfolder: str) -> str:
    return os.path.join(DEFAULTCONFIG_DIR, subfolder)
