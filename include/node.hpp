#pragma once

#include "middleware.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace autorob {

// This is the only class normal node files need to use.
class Node {
 public:
  using TopicCallback = std::function<void(const Json& message)>;
  using ServiceCallback = std::function<ServiceResult(const Json& args)>;

  explicit Node(std::string name, std::string host = "127.0.0.1",
                std::uint16_t port = 9095);
  ~Node();
  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  void advertise(const std::string& topic, const std::string& type);
  void unadvertise(const std::string& topic);
  void publish(const std::string& topic, const Json& message);

  void subscribe(const std::string& topic, const std::string& type,
                 TopicCallback callback);
  void unsubscribe(const std::string& topic);

  void advertise_service(const std::string& service, const std::string& type,
                         ServiceCallback callback);
  void unadvertise_service(const std::string& service);
  ServiceResult call_service(
      const std::string& service, const Json& args,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(3000));

  // Keeps a node process alive while callbacks run in the background.
  void wait();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace autorob
