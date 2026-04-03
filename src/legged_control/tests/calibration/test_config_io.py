import pytest, textwrap, pathlib, tempfile
from legged_control.calibration.config_io import ConfigIO

YAML = textwrap.dedent("""\
    # Top comment
    control:
      kp: 20.0  # gain
      kd: 0.5
    joints:
      - name: FR_hip
        motor_id: 0
        default_q: 0.0
""")

@pytest.fixture
def tmp_yaml(tmp_path):
    p = tmp_path / 'robot.yaml'
    p.write_text(YAML)
    return p

def test_read_returns_dict(tmp_yaml):
    cfg = ConfigIO(tmp_yaml).read()
    assert cfg['control']['kp'] == 20.0

def test_patch_preserves_comments(tmp_yaml):
    io = ConfigIO(tmp_yaml)
    io.patch({'control': {'kp': 30.0}})
    text = tmp_yaml.read_text()
    assert '# Top comment' in text
    assert '# gain' in text
    assert '30.0' in text

def test_patch_joint_field(tmp_yaml):
    io = ConfigIO(tmp_yaml)
    io.patch_joint('FR_hip', 'default_q', 0.42)
    cfg = ConfigIO(tmp_yaml).read()
    assert abs(cfg['joints'][0]['default_q'] - 0.42) < 1e-6

def test_patch_joint_motor_id(tmp_yaml):
    io = ConfigIO(tmp_yaml)
    io.patch_joint('FR_hip', 'motor_id', 7)
    cfg = ConfigIO(tmp_yaml).read()
    assert cfg['joints'][0]['motor_id'] == 7
