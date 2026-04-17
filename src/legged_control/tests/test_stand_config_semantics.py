import yaml


def test_robot_sim_has_nonzero_default_stand_pose():
    cfg = yaml.safe_load(
        open(
            "/home/grayerd/Desktop/Projects/rc/legged_ws/src/legged_control/config/robot_sim.yaml"
        )
    )
    defaults = [float(j["default_q"]) for j in cfg["joints"]]
    assert any(abs(q) > 1e-6 for q in defaults)
