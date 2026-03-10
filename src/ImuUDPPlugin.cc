#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <unistd.h>

#include <thread>
#include <mutex>
#include <queue>
#include <atomic>

#include <msgpack.hpp>

namespace gazebo
{

struct ImuPacket
{
  uint64_t seq;
  double timestamp;

  double ax, ay, az;
  double gx, gy, gz;
  double qw, qx, qy, qz;
};

class ImuUDPPlugin : public SensorPlugin
{

public:

  ImuUDPPlugin() {}

  ~ImuUDPPlugin()
  {
    running = false;

    if (netThread.joinable())
      netThread.join();

    if (sock >= 0)
      close(sock);
  }

  void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
  {
    imuSensor =
      std::dynamic_pointer_cast<sensors::ImuSensor>(_sensor);

    if (!imuSensor)
    {
      gzerr << "ImuUDPPlugin requires IMU sensor\n";
      return;
    }

    address = "127.0.0.1";
    port = 5005;

    if (_sdf->HasElement("address"))
      address = _sdf->Get<std::string>("address");

    if (_sdf->HasElement("port"))
      port = _sdf->Get<int>("port");

    SetupSocket();

    running = true;

    netThread = std::thread(&ImuUDPPlugin::NetworkLoop, this);

    updateConnection =
      imuSensor->ConnectUpdated(
        std::bind(&ImuUDPPlugin::OnUpdate, this));

    imuSensor->SetActive(true);
  }

private:

  void SetupSocket()
  {
    sock = socket(AF_INET, SOCK_DGRAM, 0);

    fcntl(sock, F_SETFL, O_NONBLOCK);

    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(port);

    inet_pton(AF_INET, address.c_str(),
              &destAddr.sin_addr);

    gzmsg << "IMU UDP MessagePack streaming to "
          << address << ":" << port << "\n";
  }

  void OnUpdate()
  {
    ignition::math::Vector3d accel =
      imuSensor->LinearAcceleration();

    ignition::math::Vector3d gyro =
      imuSensor->AngularVelocity();

    ignition::math::Quaterniond orient =
      imuSensor->Orientation();

    ImuPacket pkt;

    pkt.seq = seq++;
    pkt.timestamp =
      imuSensor->LastMeasurementTime().Double();

    pkt.ax = accel.X();
    pkt.ay = accel.Y();
    pkt.az = accel.Z();

    pkt.gx = gyro.X();
    pkt.gy = gyro.Y();
    pkt.gz = gyro.Z();

    pkt.qw = orient.W();
    pkt.qx = orient.X();
    pkt.qy = orient.Y();
    pkt.qz = orient.Z();

    {
      std::lock_guard<std::mutex> lock(queueMutex);

      queue.push(pkt);

      if (queue.size() > 200)
        queue.pop();
    }
  }

  void NetworkLoop()
  {
    while (running)
    {
      ImuPacket pkt;
      bool hasData = false;

      {
        std::lock_guard<std::mutex> lock(queueMutex);

        if (!queue.empty())
        {
          pkt = queue.front();
          queue.pop();
          hasData = true;
        }
      }

      if (hasData)
      {
        msgpack::sbuffer buffer;
        msgpack::packer<msgpack::sbuffer> pk(&buffer);

        pk.pack_array(12);

        pk.pack(pkt.seq);
        pk.pack(pkt.timestamp);

        pk.pack(pkt.ax);
        pk.pack(pkt.ay);
        pk.pack(pkt.az);

        pk.pack(pkt.gx);
        pk.pack(pkt.gy);
        pk.pack(pkt.gz);

        pk.pack(pkt.qw);
        pk.pack(pkt.qx);
        pk.pack(pkt.qy);
        pk.pack(pkt.qz);

        sendto(sock,
               buffer.data(),
               buffer.size(),
               0,
               (sockaddr*)&destAddr,
               sizeof(destAddr));
      }
      else
      {
        std::this_thread::sleep_for(
          std::chrono::microseconds(200));
      }
    }
  }

private:

  sensors::ImuSensorPtr imuSensor;

  event::ConnectionPtr updateConnection;

  std::string address;
  int port;

  int sock;
  sockaddr_in destAddr;

  std::thread netThread;

  std::mutex queueMutex;
  std::queue<ImuPacket> queue;

  std::atomic<bool> running{false};
  std::atomic<uint64_t> seq{0};
};

GZ_REGISTER_SENSOR_PLUGIN(ImuUDPPlugin)

}