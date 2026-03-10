#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/Sensor.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/imu.pb.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include <msgpack.hpp>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{

struct ImuPacket
{
  uint64_t seq{0};
  double timestamp{0.0};

  double ax{0.0}, ay{0.0}, az{0.0};
  double gx{0.0}, gy{0.0}, gz{0.0};
  double qw{1.0}, qx{0.0}, qy{0.0}, qz{0.0};
};

class ImuUDPPlugin : public System,
                     public ISystemConfigure,
                     public ISystemPostUpdate
{
public:
  ImuUDPPlugin() = default;

  ~ImuUDPPlugin() override
  {
    running = false;

    if (netThread.joinable())
      netThread.join();

    if (sock >= 0)
      close(sock);
  }

  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &/*_eventMgr*/) override
  {
    if (!_ecm.Component<components::Sensor>(_entity) ||
        !_ecm.Component<components::Imu>(_entity))
    {
      gzerr << "ImuUDPPlugin must be attached to an IMU sensor entity\n";
      return;
    }

    entity = _entity;
    address = "127.0.0.1";
    port = 5005;

    if (_sdf && _sdf->HasElement("address"))
      address = _sdf->Get<std::string>("address");

    if (_sdf && _sdf->HasElement("port"))
      port = _sdf->Get<int>("port");

    socketReady = SetupSocket();
    if (!socketReady)
      return;

    gzmsg << "IMU UDP plugin waiting for sensor topic before subscribing" << std::endl;
  }

  void PostUpdate(const UpdateInfo & /*_info*/,
                  const EntityComponentManager &_ecm) override
  {
    if (!socketReady || subscriptionStarted || subscriptionFailed ||
        entity == kNullEntity)
    {
      return;
    }

    Sensor sensor(entity);
    auto topic = sensor.Topic(_ecm);
    if (!topic)
      return;

    if (!node.Subscribe(*topic, &ImuUDPPlugin::OnImuMsg, this))
    {
      gzerr << "Failed to subscribe to IMU topic ['" << *topic << "']"
            << std::endl;
      subscriptionFailed = true;
      return;
    }

    subscriptionStarted = true;
    running = true;
    netThread = std::thread(&ImuUDPPlugin::NetworkLoop, this);

    gzmsg << "IMU UDP MessagePack streaming to "
          << address << ":" << port << " from topic "
          << *topic << std::endl;
  }

private:
  bool SetupSocket()
  {
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
      gzerr << "Unable to open UDP socket" << std::endl;
      return false;
    }

    if (fcntl(sock, F_SETFL, O_NONBLOCK) < 0)
    {
      gzerr << "Failed to set socket non-blocking" << std::endl;
      return false;
    }

    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(port);

    if (inet_pton(AF_INET, address.c_str(), &destAddr.sin_addr) != 1)
    {
      gzerr << "Invalid IPv4 address: " << address << std::endl;
      return false;
    }

    return true;
  }

  void OnImuMsg(const gz::msgs::IMU &_msg)
  {
    if (!running.load())
      return;

    ImuPacket pkt;
    pkt.seq = seq++;
    pkt.timestamp = TimeFromMsg(_msg);  // seconds

    const auto &accel = _msg.linear_acceleration();
    pkt.ax = accel.x();
    pkt.ay = accel.y();
    pkt.az = accel.z();

    const auto &gyro = _msg.angular_velocity();
    pkt.gx = gyro.x();
    pkt.gy = gyro.y();
    pkt.gz = gyro.z();

    const auto &quat = _msg.orientation();
    pkt.qw = quat.w();
    pkt.qx = quat.x();
    pkt.qy = quat.y();
    pkt.qz = quat.z();

    std::lock_guard<std::mutex> lock(queueMutex);
    queue.push(pkt);
    if (queue.size() > maxQueueDepth)
      queue.pop();
  }

  static double TimeFromMsg(const gz::msgs::IMU &_msg)
  {
    if (_msg.has_header() && _msg.header().has_stamp())
    {
      const auto &stamp = _msg.header().stamp();
      return static_cast<double>(stamp.sec()) +
            static_cast<double>(stamp.nsec()) * 1e-9;
    }

    return 0.0;
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
               reinterpret_cast<sockaddr*>(&destAddr),
               sizeof(destAddr));
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::microseconds(200));
      }
    }
  }

private:
  static constexpr std::size_t maxQueueDepth{200};

  gz::transport::Node node;

  std::string address{"127.0.0.1"};
  int port{5005};

  int sock{-1};
  sockaddr_in destAddr{};

  std::thread netThread;
  std::mutex queueMutex;
  std::queue<ImuPacket> queue;

  Entity entity{kNullEntity};
  bool socketReady{false};
  bool subscriptionStarted{false};
  bool subscriptionFailed{false};

  std::atomic<bool> running{false};
  std::atomic<uint64_t> seq{0};
};

}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(gz::sim::ImuUDPPlugin,
              gz::sim::System,
              gz::sim::ISystemConfigure)

GZ_ADD_PLUGIN_ALIAS(gz::sim::ImuUDPPlugin, "imu_udp_plugin")