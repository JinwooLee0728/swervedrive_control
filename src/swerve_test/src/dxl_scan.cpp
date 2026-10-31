// Standalone Dynamixel bus scanner.
// Usage:
//   ros2 run swerve_test dxl_scan                       # defaults below
//   ros2 run swerve_test dxl_scan /dev/ttyUSB1
//   ros2 run swerve_test dxl_scan /dev/ttyUSB1 1000000  # single baud
//
// Scans ID 0..30 at each baudrate in BAUDS unless one is specified.
// Reports any Dynamixel that replies to a Protocol 2.0 ping.

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace
{
constexpr double kProtocol = 2.0;
const std::vector<int> kBauds = {57600, 115200, 1000000, 2000000, 3000000, 4000000};
constexpr int kMaxId = 30;
}  // namespace

int main(int argc, char ** argv)
{
  std::string port = (argc > 1) ? argv[1] : "/dev/ttyUSB1";
  std::vector<int> bauds = kBauds;
  if (argc > 2) {
    bauds = {std::atoi(argv[2])};
  }

  auto * port_handler = dynamixel::PortHandler::getPortHandler(port.c_str());
  auto * packet_handler = dynamixel::PacketHandler::getPacketHandler(kProtocol);

  if (!port_handler->openPort()) {
    std::fprintf(stderr, "Failed to open port: %s\n", port.c_str());
    return 1;
  }

  bool any_found = false;
  for (int baud : bauds) {
    if (!port_handler->setBaudRate(baud)) {
      std::fprintf(stderr, "  [skip] Failed to set baud %d\n", baud);
      continue;
    }
    std::printf("Scanning %s @ %d ...\n", port.c_str(), baud);
    for (int id = 0; id <= kMaxId; ++id) {
      uint16_t model = 0;
      uint8_t err = 0;
      int result = packet_handler->ping(port_handler, id, &model, &err);
      if (result == COMM_SUCCESS) {
        std::printf("  FOUND ID=%d  model=%u  err=%u\n", id, model, err);
        any_found = true;
      }
    }
  }

  port_handler->closePort();
  if (!any_found) {
    std::printf(
      "\nNo Dynamixels replied on %s at any of %zu baudrates.\n"
      "Check: (1) motor 12-24V power supply, (2) RS-485 wiring/termination, "
      "(3) port path (try /dev/ttyUSB0 or /dev/serial/by-id/...).\n",
      port.c_str(), bauds.size());
    return 2;
  }
  return 0;
}
