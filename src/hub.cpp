#include "middleware.hpp"
#include "rosbridge_server.hpp"

#include <csignal>

int main() {
  std::signal(SIGPIPE, SIG_IGN);
  autorob::Bus bus;
  autorob::RosbridgeServer gateway(bus);
  return gateway.run("127.0.0.1", 9095) ? 0 : 1;
}
