import math

import numpy as np
import pytest

from legged_control.kinematics import (
    D_LAT,
    forward_kinematics,
    inverse_kinematics,
    projected_gravity_from_quat,
    quat_rotate_inverse,
)


class TestQuatRotateInverse:
    def test_identity_leaves_vector_unchanged(self):
        v = quat_rotate_inverse(0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 3.0)
        np.testing.assert_allclose(v, [1.0, 2.0, 3.0], atol=1e-9)

    def test_yaw_90_maps_world_x_to_body_minus_y(self):
        # body yawed +90deg about z; world +x velocity reads as body -y
        s = math.sin(math.pi / 4)
        v = quat_rotate_inverse(0.0, 0.0, s, s, 1.0, 0.0, 0.0)
        np.testing.assert_allclose(v, [0.0, -1.0, 0.0], atol=1e-9)

    def test_consistent_with_projected_gravity(self):
        # projecting world gravity [0,0,-1] into body == projected_gravity_from_quat
        q = (0.1, -0.2, 0.05, 0.97)
        n = math.sqrt(sum(c * c for c in q))
        q = tuple(c / n for c in q)
        g = quat_rotate_inverse(*q, 0.0, 0.0, -1.0)
        np.testing.assert_allclose(g, projected_gravity_from_quat(*q), atol=1e-9)


@pytest.mark.parametrize(
    "leg,y_sign", [("FL", 1.0), ("FR", -1.0), ("RL", 1.0), ("RR", -1.0)]
)
def test_fk_of_ik_recovers_target(leg: str, y_sign: float):
    xs = [-0.04, 0.00, 0.04]
    ys = [y_sign * (D_LAT + delta) for delta in (0.01, 0.04)]
    zs = [-0.22, -0.26, -0.30]

    for x in xs:
        for y in ys:
            for z in zs:
                target = (x, y, z)
                joints = inverse_kinematics(leg, target)
                assert joints is not None, f"IK unexpectedly failed for {leg} {target}"
                recovered = forward_kinematics(leg, joints)
                np.testing.assert_allclose(recovered, target, atol=1e-4)


def test_inverse_kinematics_returns_none_outside_workspace():
    assert inverse_kinematics("FL", (0.0, 0.0, -0.05)) is None


def test_invalid_leg_raises():
    with pytest.raises(ValueError):
        inverse_kinematics("XX", (0.0, D_LAT + 0.02, -0.25))


@pytest.mark.parametrize(
    "leg,joints",
    [
        ("FL", (0.14, 0.34, -0.72)),
        ("FR", (-0.14, 0.34, -0.72)),
        ("RL", (0.10, 0.46, -0.82)),
        ("RR", (-0.10, 0.46, -0.82)),
    ],
)
def test_training_default_pose_roundtrips(leg: str, joints: tuple[float, float, float]):
    foot = forward_kinematics(leg, joints)
    recovered = inverse_kinematics(leg, foot)
    assert recovered is not None
    np.testing.assert_allclose(recovered, joints, atol=1e-4)
