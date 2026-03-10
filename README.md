# IMU UDP Gazebo Plugin

A lightweight Gazebo sensor plugin that streams IMU measurements over UDP using the MessagePack serialization format. The repository also contains a small receiver utility and the vendored MessagePack headers that are used by both the plugin and the sample application.

---

## Features
- Streams linear acceleration, angular velocity, and orientation sampled from any Gazebo IMU sensor.
- Publishes packets over UDP in MessagePack format for easy consumption in C++, Python, or any other language with a MessagePack client.
- Includes a reference receiver that prints decoded packets to the console.
- Self-contained MessagePack dependency under `third_party/msgpack`.

## Repository Layout
- `src/ImuUDPPlugin.cc` – main Gazebo `SensorPlugin` implementation that gathers IMU data and pushes it to a background networking thread.
- `examples/model.sdf` – minimal sensor definition that demonstrates how to attach the plugin to an IMU inside an SDF model.
- `examples/receiver.cpp` – simple console program that subscribes to the UDP stream and logs the decoded values.
- `third_party/msgpack` – vendored MessagePack C++ headers used by both the plugin and the example receiver.

## Requirements
- Gazebo (Classic) with development headers (`gazebo`, `gazebo-dev`). Tested with Gazebo 9+.
- CMake ≥ 3.10.
- A C++14 capable compiler (GCC 9+, Clang 10+, or Apple Clang 12+).
- POSIX sockets (Linux/macOS). The plugin currently targets Unix-like systems.

The repository already ships with MessagePack headers, so no additional package installation is required for that dependency.

## Building the Plugin
```
mkdir -p build && cd build
cmake ..
make -j
```
The shared library `libImuUDPPlugin.so` will be placed in `build/`. Point Gazebo to the build folder via `GAZEBO_PLUGIN_PATH` (or by copying the library into a directory that is already on that path):
```
export GAZEBO_PLUGIN_PATH=$PWD:${GAZEBO_PLUGIN_PATH}
```

## Using the Plugin in an SDF Model
Attach the plugin to any IMU sensor by adding the following snippet to your model file:
```xml
<sensor name="imu_sensor" type="imu">
  <update_rate>1000</update_rate>
  <plugin name="imu_udp_plugin" filename="libImuUDPPlugin.so">
    <address>127.0.0.1</address>   <!-- Destination IPv4 address -->
    <port>5005</port>               <!-- Destination UDP port -->
  </plugin>
</sensor>
```
Only the `address` and `port` tags are currently parsed. The example `model.sdf` in this repository shows the same structure with additional placeholder tags that are ignored by the plugin.

> **Tip:** You can embed the sensor inside any robot model or create a dedicated world file that includes the above snippet. Launch Gazebo with that world after exporting `GAZEBO_PLUGIN_PATH`.

## Running the Sample Receiver
The receiver is a straightforward console application located under `examples/receiver.cpp`. Compile it after building the plugin so the MessagePack headers are already available:
```
cd examples
c++ receiver.cpp -I../third_party/msgpack/include -o imu_receiver
./imu_receiver
```
By default, the plugin sends packets to `127.0.0.1:5005`, so running the receiver on the same host will immediately show the streamed data:
```
SEQ: 42  time: 12345.678
Accel: -0.01 0.00 9.80
Gyro:  0.00 -0.00 0.00
Quat:  1.00 0.00 0.00 0.00
-------------------------
```

## MessagePack Schema
Each UDP datagram holds a 12-element MessagePack array in the following order:
1. `uint64 seq`
2. `double timestamp` (simulation time in seconds)
3. `double ax, ay, az` – linear acceleration (m/s²)
4. `double gx, gy, gz` – angular velocity (rad/s)
5. `double qw, qx, qy, qz` – orientation quaternion

Consumers written in other languages simply need a MessagePack client to unpack this fixed-size array.

## Configuration
Tag | Description | Default
----|-------------|--------
`address` | Destination IPv4 address for outgoing UDP packets | `127.0.0.1`
`port` | Destination UDP port | `5005`

(Additional tags shown in `examples/model.sdf` are currently ignored.)

## Developing & Testing
- The plugin keeps a small in-memory queue and a dedicated networking thread to ship packets without stalling the Gazebo update callback.
- Message serialization uses `msgpack::packer` for zero-copy writes directly into the UDP buffer.
- To inspect or modify the message contents, edit `ImuUDPPlugin::OnUpdate()` in `src/ImuUDPPlugin.cc`.

Pull requests and bug reports are welcome. Please include environment details (Gazebo version, OS, compiler) along with reproduction steps when filing issues.
