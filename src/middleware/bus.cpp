#include "middleware.hpp"

#include <exception>
#include <utility>
#include <vector>

namespace autorob {

std::uint64_t Bus::subscribe(const std::string& topic, TopicCallback callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::uint64_t token = next_token_++;
  topic_callbacks_[topic].emplace(token, std::move(callback));
  token_topics_[token] = topic;
  return token;
}

void Bus::unsubscribe(std::uint64_t token) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto token_it = token_topics_.find(token);
  if (token_it == token_topics_.end()) return;
  auto topic_it = topic_callbacks_.find(token_it->second);
  if (topic_it != topic_callbacks_.end()) {
    topic_it->second.erase(token);
    if (topic_it->second.empty()) topic_callbacks_.erase(topic_it);
  }
  token_topics_.erase(token_it);
}

void Bus::publish(const std::string& topic, const Json& message) {
  std::vector<TopicCallback> callbacks;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = topic_callbacks_.find(topic);
    if (it != topic_callbacks_.end()) {
      callbacks.reserve(it->second.size());
      for (const auto& entry : it->second) callbacks.push_back(entry.second);
    }
  }
  for (const TopicCallback& callback : callbacks) callback(message);
}

void Bus::advertise_service(const std::string& service, ServiceHandler handler) {
  std::lock_guard<std::mutex> lock(mutex_);
  services_[service] = std::move(handler);
}

std::optional<ServiceResult> Bus::call_service(const std::string& service,
                                               const Json& args) const {
  ServiceHandler handler;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = services_.find(service);
    if (it == services_.end()) return std::nullopt;
    handler = it->second;
  }
  try {
    return handler(args);
  } catch (const std::exception& error) {
    return ServiceResult{false, Json::Object{}, error.what()};
  }
}

}  // namespace autorob
