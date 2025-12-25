#include <iostream>
#include <eigen3/Eigen/Dense>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <boost/asio.hpp>

struct Packet {
  uint32_t t;
  float ax, ay, az;
  float gx, gy, gz;
};


int main() {
  boost::asio::io_service io;
  boost::asio::serial_port port(io, "COM3");

  port.set_option(boost::asio::serial_port_base::baud_rate(115200));

  Packet p;

  while (true) {
    boost::asio::read(port, boost::asio::buffer(&p, sizeof(Packet)));

    std::cout << p.t << " "
              << p.ax << " "
              << p.ay << " "
              << p.az << " "
              << p.gx << " "
              << p.gy << " "
              << p.gz << "\n";

    // EKF update here (Eigen)
  }
}
