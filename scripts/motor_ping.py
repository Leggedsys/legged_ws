#!/usr/bin/env python3
"""motor_ping — RS485 总线健康检查:零力矩逐电机 ping,统计应答率。

用途:电机"连不上"时区分 电机死了 / 总线丢包 / 串口问题。
  一次扫描:  /usr/bin/python3 scripts/motor_ping.py --port /dev/ttyUSB0 --ids 0,1,2,3,4,5 2>/dev/null
  摇线定位:  加 --watch,每秒刷新应答率;此时逐个轻晃各段线缆/接插件,
             哪一晃应答率跳水,坏点就在哪(经典 wiggle test)。Ctrl-C 退出。

kp=kd=tau=0,纯读,任何状态下运行都安全(但别和 motor_bus 节点同时开)。
2026-07-13 实录:走机 FL_calf"连不上",实测是全总线丢包
(FL_hip 18%/FL_calf 32%/RL_calf 0%,截断帧 13~15/16 字节 = 信号完整性)。
"""
import argparse
import sys
import time

sys.path.insert(
    0,
    "/home/shijue/legged_ws/install/unitree_actuator_sdk/lib/python3.10/site-packages",
)
from unitree_motor_ros2.sdk_loader import load_sdk  # noqa: E402

NAMES = {0: "FR_hip", 1: "FR_thigh", 2: "FR_calf",
         3: "FL_hip", 4: "FL_thigh", 5: "FL_calf",
         6: "RR_hip", 7: "RR_thigh", 11: "RR_calf",
         9: "RL_hip", 10: "RL_thigh", 8: "RL_calf"}


def scan(sdk, serial, ids, n):
    cmd, data = sdk.MotorCmd(), sdk.MotorData()
    out = {}
    for mid in ids:
        ok = 0
        q = temp = None
        for _ in range(n):
            # 每次都重设——sendRecv 可能覆写这些字段(motor_bus 同款坑)
            data.motorType = sdk.MotorType.GO_M8010_6
            cmd.motorType = sdk.MotorType.GO_M8010_6
            cmd.mode = sdk.queryMotorMode(sdk.MotorType.GO_M8010_6, sdk.MotorMode.FOC)
            cmd.id = mid
            cmd.kp = cmd.kd = cmd.q = cmd.dq = cmd.tau = 0.0
            data.correct = False
            data.motor_id = 255
            serial.sendRecv(cmd, data)
            if data.correct and int(data.motor_id) == mid:
                ok += 1
                q, temp = float(data.q), int(data.temp)
            time.sleep(0.001)
        out[mid] = (ok, q, temp)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--ids", default="0,1,2,3,4,5",
                    help="逗号分隔;后总线用 6,7,11,9,10,8")
    ap.add_argument("-n", type=int, default=50, help="每电机 ping 次数")
    ap.add_argument("--watch", action="store_true", help="循环刷新(摇线定位)")
    args = ap.parse_args()

    sdk = load_sdk()
    serial = sdk.SerialPort(args.port)
    ids = [int(x) for x in args.ids.split(",")]
    n = 10 if args.watch else args.n

    while True:
        res = scan(sdk, serial, ids, n)
        line = time.strftime("%H:%M:%S") if args.watch else f"{args.port}"
        parts = []
        for mid in ids:
            ok, q, temp = res[mid]
            pct = 100 * ok // n
            tag = NAMES.get(mid, f"id{mid}")
            parts.append(f"{tag} {pct:3d}%")
        print(f"[{line}] " + "  ".join(parts), flush=True)
        if not args.watch:
            for mid in ids:
                ok, q, temp = res[mid]
                extra = f" q={q:.2f} temp={temp}C" if q is not None else "  <-- 无任何应答"
                print(f"  id {mid:2d} {NAMES.get(mid, ''):9s} {ok}/{n}{extra}")
            break


if __name__ == "__main__":
    main()
