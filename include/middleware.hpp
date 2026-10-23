#pragma once

#include "json.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

namespace autorob {

struct ServiceResult {
  bool result = false;
  Json values = Json::Object{};
  std::string status;
};

class Bus {
 public:
  using TopicCallback = std::function<void(const Json&)>;
  using ServiceHandler = std::function<ServiceResult(const Json&)>;

  std::uint64_t subscribe(const std::string& topic, TopicCallback callback);
  void unsubscribe(std::uint64_t token);
  void publish(const std::string& topic, const Json& message);
  void advertise_service(const std::string& service, ServiceHandler handler);
  std::optional<ServiceResult> call_service(const std::string& service, const Json& args) const;

 private:
  mutable std::mutex mutex_;
  std::uint64_t next_token_ = 1;
  std::unordered_map<std::string, std::map<std::uint64_t, TopicCallback>> topic_callbacks_;
  std::unordered_map<std::uint64_t, std::string> token_topics_;
  std::unordered_map<std::string, ServiceHandler> services_;
};

}  // namespace autorob
