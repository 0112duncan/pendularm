#pragma once

#include "middleware.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace autorob {

class RosbridgeServer {
 public:
  explicit RosbridgeServer(Bus& bus);
  ~RosbridgeServer();
  RosbridgeServer(const RosbridgeServer&) = delete;
  RosbridgeServer& operator=(const RosbridgeServer&) = delete;

  bool run(const std::string& address = "127.0.0.1", std::uint16_t port = 9095);
  void stop();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace autorob
