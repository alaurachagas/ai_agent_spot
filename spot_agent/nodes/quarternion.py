import math
import numpy as np
from math import radians, sin, cos, acos, sqrt

def normalize(v, tolerance=0.00001):
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = sqrt(mag2)
        v = tuple(n / mag for n in v)
    return np.array(v)

class Quaternion:

    def from_axisangle(theta, v):
        theta = theta
        v = normalize(v)

        new_quaternion = Quaternion()
        new_quaternion._axisangle_to_q(theta, v)
        return new_quaternion

    def from_value(value):
        new_quaternion = Quaternion()
        new_quaternion._val = value
        return new_quaternion

    def _axisangle_to_q(self, theta, v):
        x = v[0]
        y = v[1]
        z = v[2]

        w = cos(theta/2.)
        x = x * sin(theta/2.)
        y = y * sin(theta/2.)
        z = z * sin(theta/2.)

        self._val = np.array([w, x, y, z])

    def __mul__(self, b):

        if isinstance(b, Quaternion):
            return self._multiply_with_quaternion(b)
        elif isinstance(b, (list, tuple, np.ndarray)):
            if len(b) != 3:
                raise Exception(f"Input vector has invalid length {len(b)}")
            return self._multiply_with_vector(b)
        else:
            raise Exception(f"Multiplication with unknown type {type(b)}")

    def _multiply_with_quaternion(self, q2):
        w1, x1, y1, z1 = self._val
        w2, x2, y2, z2 = q2._val
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2

        result = Quaternion.from_value(np.array((w, x, y, z)))
        return result

    def _multiply_with_vector(self, v):
        q2 = Quaternion.from_value(np.append((0.0), v))
        return (self * q2 * self.get_conjugate())._val[1:]

    def get_conjugate(self):
        w, x, y, z = self._val
        result = Quaternion.from_value(np.array((w, -x, -y, -z)))
        return result

    def __repr__(self):
        theta, v = self.get_axisangle()
        return f"((%.6f; %.6f, %.6f, %.6f))"%(theta, v[0], v[1], v[2])

    def get_axisangle(self):
        w, v = self._val[0], self._val[1:]
        theta = acos(w) * 2.0

        return theta, normalize(v)

    def tolist(self):
        return self._val.tolist()

    def vector_norm(self):
        w, v = self.get_axisangle()
        return np.linalg.norm(v)
    

def main():
    # Exemple of implementation

    x_axis_unit = (1, 0, 0)
    y_axis_unit = (0, 1, 0)
    z_axis_unit = (0, 0, 1)

    r1 = Quaternion.from_axisangle(np.pi / 2, x_axis_unit)
    r2 = Quaternion.from_axisangle(np.pi / 2, y_axis_unit)
    r3 = Quaternion.from_axisangle(np.pi / 2, z_axis_unit)

    # Quaternion - vector multiplication
    v = r1 * y_axis_unit
    v = r2 * v
    v = r3 * v

    print(v)

    # Quaternion - quaternion multiplication
    r_total = r3 * r2 * r1
    v = r_total * y_axis_unit

    print(v)


def calculate_turn_pose(pose: dict, yaw_deg: float) -> dict:
    """
    Rotate the robot about the +Z axis by yaw_deg (in degrees).
    Keeps the same position, only updates orientation.
    """
    # Extract current orientation (ROS order → Quaternion order)
    qx = float(pose["orientation"]["x"])
    qy = float(pose["orientation"]["y"])
    qz = float(pose["orientation"]["z"])
    qw = float(pose["orientation"]["w"])
    q_curr = Quaternion.from_value(np.array([qw, qx, qy, qz]))

    # Create rotation quaternion about +Z and compose
    q_delta = Quaternion.from_axisangle(math.radians(yaw_deg), (0.0, 0.0, 1.0))
    q_new = q_curr * q_delta

    # Convert back to ROS order
    w, x, y, z = q_new.tolist()
    return {
        "position": dict(pose["position"]),
        "orientation": {"x": x, "y": y, "z": z, "w": w},
    }


def calculate_move_forward_pose(pose: dict, dist_m: float) -> dict:
    """
    Move the robot forward along its local +X axis by dist_m meters.
    Orientation remains unchanged.
    """
    # Extract position
    px = float(pose["position"]["x"])
    py = float(pose["position"]["y"])
    pz = float(pose["position"]["z"])

    # Extract orientation (ROS order → Quaternion order)
    qx = float(pose["orientation"]["x"])
    qy = float(pose["orientation"]["y"])
    qz = float(pose["orientation"]["z"])
    qw = float(pose["orientation"]["w"])
    q = Quaternion.from_value(np.array([qw, qx, qy, qz]))

    # Rotate the robot's +X vector into the world frame
    fwd_world = q * (1.0, 0.0, 0.0)  # rotated +X direction
    nx = px + dist_m * float(fwd_world[0])
    ny = py + dist_m * float(fwd_world[1])
    nz = pz  # keep same height for ground robot

    # Return updated pose
    return {
        "position": {"x": nx, "y": ny, "z": nz},
        "orientation": dict(pose["orientation"]),  # unchanged
    }
    