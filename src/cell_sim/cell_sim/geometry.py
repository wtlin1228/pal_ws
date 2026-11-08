"""The twin's own 4x4 helpers (NumPy + SciPy). The C++ side has the same functions in cell_planning/geometry.hpp."""
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Point, Quaternion, Transform


def mat_from_pose(p: Pose) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]).as_matrix()
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T


def mat_from_tf(t: Transform) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R.from_quat([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]).as_matrix()
    T[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return T


def pose_from_mat(T: np.ndarray) -> Pose:
    q = R.from_matrix(T[:3, :3]).as_quat()      # x, y, z, w
    return Pose(position=Point(x=float(T[0, 3]), y=float(T[1, 3]), z=float(T[2, 3])),
                orientation=Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3])))


def trans(x=0.0, y=0.0, z=0.0) -> np.ndarray:
    T = np.eye(4); T[:3, 3] = [x, y, z]; return T


def rot(axis: str, angle: float) -> np.ndarray:
    T = np.eye(4); T[:3, :3] = R.from_euler(axis, angle).as_matrix(); return T
