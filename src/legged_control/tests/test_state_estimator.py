import math
import numpy as np
import pytest
from legged_control.kinematics import (
    projected_gravity_from_quat,
    yaw_rotation_matrix,
    leg_kinematic_velocity,
)
from legged_control.processing.state_estimator_node import _accept_gravity


class TestAcceptGravity:
    def test_accepts_unit_vector_pointing_down(self):
        assert _accept_gravity(np.array([0.0, 0.0, -1.0]))

    def test_accepts_unit_vector_when_heavily_tilted(self):
        # previously rejected by the gz < -0.1 gate; must now be accepted
        assert _accept_gravity(np.array([0.998, 0.0, 0.05]))

    def test_rejects_non_unit_magnitude(self):
        assert not _accept_gravity(np.array([0.0, 0.0, -0.5]))
        assert not _accept_gravity(np.array([0.0, 0.0, 0.0]))


def test_projected_gravity_identity():
    g = projected_gravity_from_quat(0.0, 0.0, 0.0, 1.0)
    np.testing.assert_allclose(g, [0.0, 0.0, -1.0], atol=1e-6)


def test_projected_gravity_pitch_90():
    s = math.sin(math.pi / 4)
    g = projected_gravity_from_quat(0.0, s, 0.0, s)
    assert g[0] > 0.9, f"Expected x>0.9, got {g}"
    assert abs(g[2]) < 0.1, f"Expected z≈0, got {g}"


def test_projected_gravity_unit_length():
    g = projected_gravity_from_quat(0.1, 0.2, 0.3, 0.9)
    np.testing.assert_allclose(np.linalg.norm(g), 1.0, atol=1e-5)


def test_yaw_rotation_identity():
    R = yaw_rotation_matrix(0.0, 0.0, 0.0, 1.0)
    np.testing.assert_allclose(R, np.eye(3), atol=1e-6)


def test_yaw_rotation_90():
    s = math.sin(math.pi / 4)
    R = yaw_rotation_matrix(0.0, 0.0, s, s)
    v = R @ np.array([1.0, 0.0, 0.0])
    np.testing.assert_allclose(v, [0.0, 1.0, 0.0], atol=1e-5)


def test_kinematic_velocity_zero_dq():
    v = leg_kinematic_velocity("FL", (0.0, 0.7, -1.2), (0.0, 0.0, 0.0))
    np.testing.assert_allclose(v, [0.0, 0.0, 0.0], atol=1e-6)


def test_kinematic_velocity_shape():
    v = leg_kinematic_velocity("FR", (0.0, 0.5, -1.0), (0.1, 0.2, -0.1))
    assert v.shape == (3,)
