#!/usr/bin/env python3

import socket
import msgpack

PORT = 5609
BUFFER_SIZE = 512


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PORT))

    print(f"Listening for IMU MessagePack packets on port {PORT}")

    while True:
        data, _ = sock.recvfrom(BUFFER_SIZE)
        if not data:
            continue

        try:
            values = msgpack.unpackb(data, raw=False)

            if not isinstance(values, list) or len(values) != 12:
                print(f"Unexpected packet length: {len(values)}", flush=True)
                continue

            seq = int(values[0])
            timestamp = values[1]

            ax, ay, az = values[2], values[3], values[4]
            gx, gy, gz_val = values[5], values[6], values[7]
            qw, qx, qy, qz = values[8], values[9], values[10], values[11]

            print(f"SEQ: {seq}  time: {timestamp}")
            print(f"Accel: {ax} {ay} {az}")
            print(f"Gyro:  {gx} {gy} {gz_val}")
            print(f"Quat:  {qw} {qx} {qy} {qz}")
            print("-------------------------")

        except Exception as e:
            print(f"Failed to unpack MessagePack packet: {e}")


if __name__ == "__main__":
    main()
