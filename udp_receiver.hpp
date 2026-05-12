// udp_receiver.hpp
// Tyler's component — drop this header into your OpenGL project.
//
// Listens on a UDP port for packets from udp_bridge.py and exposes
// roll/pitch/yaw, x/y/z position, and raw IMU values.
//
// Packet format expected (from udp_bridge.py):
//   "roll,pitch,yaw,x,y,z,ax,ay,az,gx,gy,gz\n"
//   Angles in degrees. Position in metres. Accel in g. Gyro in deg/s.
//
// Usage (in your OpenGL project)
// --------------------------------
//   #include "udp_receiver.hpp"
//
//   UDPReceiver imu(5005);   // port must match --udp-port in udp_bridge.py
//   imu.start();             // call once before render loop
//
//   // Inside render loop:
//   IMUData d = imu.getData();
//   if (d.valid) {
//       // d.roll, d.pitch, d.yaw  → orientation (degrees)
//       // d.x, d.y, d.z           → position (metres from start)
//   }
//
//   imu.stop();   // call on shutdown
//
// Dependencies: none (uses platform sockets + C++11 threads)
// Windows: links against Ws2_32 automatically via #pragma comment

#pragma once

#include <atomic>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>

#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #pragma comment(lib, "Ws2_32.lib")
  typedef int socklen_t;
#else
  #include <arpa/inet.h>
  #include <netinet/in.h>
  #include <sys/socket.h>
  #include <unistd.h>
  #define SOCKET          int
  #define INVALID_SOCKET  (-1)
  #define SOCKET_ERROR    (-1)
  #define closesocket     close
#endif

// ── Data struct passed to OpenGL render loop ──────────────────────────────────

struct IMUData {
    float roll  = 0.0f;   // degrees  (+right tilt, -left tilt)
    float pitch = 0.0f;   // degrees  (+forward, -backward)
    float yaw   = 0.0f;   // degrees  (heading; drifts without magnetometer)
    float x     = 0.0f;   // metres  (world X from start position)
    float y     = 0.0f;   // metres  (world Y from start position)
    float z     = 0.0f;   // metres  (world Z from start position)
    float ax    = 0.0f;   // g
    float ay    = 0.0f;   // g
    float az    = 0.0f;   // g
    float gx    = 0.0f;   // deg/s
    float gy    = 0.0f;   // deg/s
    float gz    = 0.0f;   // deg/s
    bool  valid = false;  // true once at least one packet has been received
};

// ── UDP Receiver ──────────────────────────────────────────────────────────────

class UDPReceiver {
public:
    explicit UDPReceiver(int port = 5005)
        : port_(port), sock_(INVALID_SOCKET), running_(false) {}

    ~UDPReceiver() { stop(); }

    // ── start() ──────────────────────────────────────────────────────────────
    // Call once at startup (before your render loop).
    // Returns true on success.
    bool start() {
#ifdef _WIN32
        WSADATA wsa;
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
            fprintf(stderr, "[UDP] WSAStartup failed\n");
            return false;
        }
#endif
        sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock_ == INVALID_SOCKET) {
            fprintf(stderr, "[UDP] socket() failed\n");
            return false;
        }

        // Non-blocking receive — render loop never stalls waiting for data
#ifdef _WIN32
        u_long nonblock = 1;
        ioctlsocket(sock_, FIONBIO, &nonblock);
#else
        struct timeval tv { 0, 500 };   // 0.5 ms timeout
        setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif

        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port        = htons(static_cast<uint16_t>(port_));

        if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr))
                == SOCKET_ERROR) {
            fprintf(stderr, "[UDP] bind() failed on port %d\n", port_);
            closesocket(sock_);
            sock_ = INVALID_SOCKET;
            return false;
        }

        running_ = true;
        thread_  = std::thread(&UDPReceiver::recvLoop, this);
        fprintf(stdout, "[UDP] Listening on port %d\n", port_);
        return true;
    }

    // ── getData() ─────────────────────────────────────────────────────────────
    // Call every frame inside your render loop.
    // Returns a snapshot of the most recently received orientation.
    IMUData getData() {
        std::lock_guard<std::mutex> lock(mutex_);
        return latest_;
    }

    // ── stop() ────────────────────────────────────────────────────────────────
    // Call on application shutdown.
    void stop() {
        running_ = false;
        if (sock_ != INVALID_SOCKET) {
            closesocket(sock_);
            sock_ = INVALID_SOCKET;
        }
        if (thread_.joinable())
            thread_.join();
#ifdef _WIN32
        WSACleanup();
#endif
    }

private:
    int               port_;
    SOCKET            sock_;
    std::atomic<bool> running_;
    std::thread       thread_;
    std::mutex        mutex_;
    IMUData           latest_;

    // Background thread: drains the socket as fast as packets arrive.
    void recvLoop() {
        char buf[256];
        while (running_) {
            int n = recv(sock_, buf, static_cast<int>(sizeof(buf)) - 1, 0);
            if (n <= 0) {
                std::this_thread::sleep_for(std::chrono::microseconds(200));
                continue;
            }
            buf[n] = '\0';

            IMUData d;
            int matched = sscanf(buf,
                "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                &d.roll,  &d.pitch, &d.yaw,
                &d.x,     &d.y,     &d.z,
                &d.ax,    &d.ay,    &d.az,
                &d.gx,    &d.gy,    &d.gz);

            if (matched == 12) {
                d.valid = true;
                std::lock_guard<std::mutex> lock(mutex_);
                latest_ = d;
            }
        }
    }
};

// ── Quick usage example (remove before shipping) ──────────────────────────────
//
//  int main() {
//      UDPReceiver imu(5005);
//      if (!imu.start()) return 1;
//
//      // --- OpenGL render loop ---
//      while (running) {
//          IMUData d = imu.getData();
//          if (d.valid) {
//              glMatrixMode(GL_MODELVIEW);
//              glLoadIdentity();
//
//              // Position: translate box in world space (metres → scale as needed)
//              float scale = 5.0f;   // tune this so movement feels natural on screen
//              glTranslatef(d.x * scale, d.y * scale, d.z * scale);
//
//              // Orientation: rotate box
//              glRotatef(d.yaw,   0, 1, 0);   // yaw   around Y
//              glRotatef(d.pitch, 1, 0, 0);   // pitch around X
//              glRotatef(d.roll,  0, 0, 1);   // roll  around Z
//
//              drawBox();
//          }
//          swapBuffers();
//      }
//      imu.stop();
//  }
