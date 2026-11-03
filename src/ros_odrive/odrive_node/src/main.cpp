#include "odrive_can_node.hpp"
#include "epoll_event_loop.hpp"
#include "socket_can.hpp"
#include <thread>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    EpollEventLoop event_loop;
    auto can_node = std::make_shared<ODriveCanNode>("ODriveCanNode");

    if (!can_node->init(&event_loop)) return -1;

    std::thread can_event_loop([&event_loop]() { event_loop.run_until_empty(); });
    rclcpp::spin(can_node);
    can_node->deinit();
    // Every fd is deregistered now, but the loop thread stays blocked in
    // epoll_wait (nothing wakes it), so it can't be joined. Destroying a
    // joinable std::thread calls std::terminate -- that was the exit code -6
    // on every CTRL-C. Detach it; process exit reclaims it.
    can_event_loop.detach();
    rclcpp::shutdown();
    return 0;
}
