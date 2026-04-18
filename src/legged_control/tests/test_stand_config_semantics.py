import yaml


def test_robot_sim_has_nonzero_default_stand_pose():
    cfg = yaml.safe_load(
        open(
            "/home/grayerd/Desktop/Projects/rc/legged_ws/src/legged_control/config/robot_sim.yaml"
        )
    )
    defaults = [float(j["default_q"]) for j in cfg["joints"]]
    assert any(abs(q) > 1e-6 for q in defaults)


def test_robot_configs_define_posture_toggle_button_and_lie_down_duration():
    for path in (
        "/home/grayerd/Desktop/Projects/rc/legged_ws/src/legged_control/config/robot.yaml",
        "/home/grayerd/Desktop/Projects/rc/legged_ws/src/legged_control/config/robot_sim.yaml",
    ):
        cfg = yaml.safe_load(open(path))
        assert "btn_posture_toggle" in cfg["teleop"]
        assert float(cfg["standup"]["lie_down_duration"]) > 0.0
