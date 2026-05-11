import numpy as np
import pytest


def _build_height_scan(points_base_link: np.ndarray) -> np.ndarray:
    from legged_control.height_scan_node import _build_height_scan as _f
    return _f(points_base_link)


def test_empty_point_cloud_returns_zeros():
    points = np.zeros((0, 3), dtype=np.float32)
    hs = _build_height_scan(points)
    assert hs.shape == (325,)
    np.testing.assert_array_equal(hs, np.zeros(325, dtype=np.float32))


def test_flat_ground_at_stance_height():
    xs = np.linspace(0.10, 1.30, 25)
    ys = np.linspace(-0.30, 0.30, 13)
    xv, yv = np.meshgrid(xs, ys)
    pts = np.stack([xv.ravel(), yv.ravel(), np.full(325, -0.28)], axis=1).astype(np.float32)
    hs = _build_height_scan(pts)
    np.testing.assert_allclose(hs, np.full(325, 0.28, dtype=np.float32), atol=1e-4)


def test_obstacle_clips_to_negative():
    pts = np.array([[0.70, 0.0, 0.80]], dtype=np.float32)
    hs = _build_height_scan(pts)
    # Cell for x=0.70, y=0.00: x_idx=12, y_idx=6, idx=6*25+12=162
    assert hs[162] == pytest.approx(-1.0, abs=1e-4)


def test_out_of_range_points_ignored():
    pts = np.array([[0.0, 0.0, -0.3], [2.0, 0.0, -0.3]], dtype=np.float32)
    hs = _build_height_scan(pts)
    np.testing.assert_array_equal(hs, np.zeros(325, dtype=np.float32))


def test_multiple_hits_use_highest_point():
    x, y = 0.50, 0.00
    pts = np.array([[x, y, -0.30], [x, y, -0.10]], dtype=np.float32)
    hs = _build_height_scan(pts)
    # x_idx=8, y_idx=6, idx=158
    assert hs[158] == pytest.approx(0.10, abs=1e-4)


def test_output_dtype_float32():
    hs = _build_height_scan(np.zeros((0, 3), dtype=np.float32))
    assert hs.dtype == np.float32
