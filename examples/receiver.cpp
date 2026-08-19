#include <iostream>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <vector>

#include <msgpack.hpp>

int main()
{
    const int PORT = 5609;

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
        perror("socket");
        return 1;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (sockaddr*)&addr, sizeof(addr)) < 0)
    {
        perror("bind");
        return 1;
    }

    std::cout << "Listening for IMU MessagePack packets on port " << PORT << std::endl;

    const size_t BUFFER_SIZE = 512;
    char buffer[BUFFER_SIZE];

    while (true)
    {
        ssize_t bytes = recvfrom(
            sock,
            buffer,
            BUFFER_SIZE,
            0,
            nullptr,
            nullptr);

        if (bytes <= 0)
            continue;

        try
        {
            // Unpack the MessagePack buffer
            msgpack::object_handle oh = msgpack::unpack(buffer, bytes);
            msgpack::object obj = oh.get();

            // Convert to a vector of doubles
            std::vector<double> values;
            obj.convert(values);

            if (values.size() != 12)
            {
                std::cerr << "Unexpected packet length: " << values.size() << std::endl;
                continue;
            }

            uint64_t seq = static_cast<uint64_t>(values[0]);
            double timestamp = values[1];

            double ax = values[2], ay = values[3], az = values[4];
            double gx = values[5], gy = values[6], gz = values[7];
            double qw = values[8], qx = values[9], qy = values[10], qz = values[11];

            // Print nicely
            std::cout << "SEQ: " << seq << "  time: " << timestamp << std::endl;
            std::cout << "Accel: " << ax << " " << ay << " " << az << std::endl;
            std::cout << "Gyro:  " << gx << " " << gy << " " << gz << std::endl;
            std::cout << "Quat:  " << qw << " " << qx << " " << qy << " " << qz << std::endl;
            std::cout << "-------------------------" << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to unpack MessagePack packet: " << e.what() << std::endl;
        }
    }

    close(sock);
}