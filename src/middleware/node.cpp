#include "node.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>

namespace autorob {

struct Node::Impl {
  struct PendingCall {
    std::mutex mutex;
    std::condition_variable ready;
    bool done = false;
    ServiceResult reply{false, Json::Object{}, "connection closed"};
  };

  Impl(std::string node_name, const std::string& host, std::uint16_t port)
      : name(std::move(node_name)) {
    connect_to_hub(host, port);
    receiver = std::thread([this] { receive_loop(); });
  }

  ~Impl() {
    running = false;
    int socket_fd = fd.exchange(-1);
    if (socket_fd >= 0) {
      ::shutdown(socket_fd, SHUT_RDWR);
      ::close(socket_fd);
    }
    if (receiver.joinable() && receiver.get_id() != std::this_thread::get_id()) {
      receiver.join();
    }
  }

  void connect_to_hub(const std::string& host, std::uint16_t port) {
    sockaddr_in endpoint{};
    endpoint.sin_family = AF_INET;
    endpoint.sin_port = htons(port);
    if (::inet_pton(AF_INET, host.c_str(), &endpoint.sin_addr) != 1) {
      throw std::runtime_error("invalid hub address: " + host);
    }

    // A short retry lets scripts start the hub and nodes at nearly the same time.
    for (int attempt = 0; attempt < 100; ++attempt) {
      int candidate = ::socket(AF_INET, SOCK_STREAM, 0);
      if (candidate >= 0 &&
          ::connect(candidate, reinterpret_cast<sockaddr*>(&endpoint), sizeof(endpoint)) == 0) {
        fd = candidate;
        return;
      }
      if (candidate >= 0) ::close(candidate);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw std::runtime_error("could not connect to the hub at " + host + ':' +
                             std::to_string(port));
  }

  void send(const Json& message) {
    std::string line = message.dump() + '\n';
    std::lock_guard<std::mutex> lock(send_mutex);
    int socket_fd = fd.load();
    if (socket_fd < 0) throw std::runtime_error("node is disconnected");
    std::size_t sent = 0;
    while (sent < line.size()) {
      ssize_t count = ::send(socket_fd, line.data() + sent, line.size() - sent, MSG_NOSIGNAL);
      if (count < 0) {
        if (errno == EINTR) continue;
        throw std::runtime_error(std::string("send failed: ") + std::strerror(errno));
      }
      sent += static_cast<std::size_t>(count);
    }
  }

  void receive_loop() {
    std::string input;
    char buffer[8192];
    while (running) {
      int socket_fd = fd.load();
      if (socket_fd < 0) break;
      ssize_t count = ::recv(socket_fd, buffer, sizeof(buffer), 0);
      if (count <= 0) {
        if (count < 0 && errno == EINTR) continue;
        break;
      }
      input.append(buffer, static_cast<std::size_t>(count));
      for (;;) {
        std::size_t newline = input.find('\n');
        if (newline == std::string::npos) break;
        std::string line = input.substr(0, newline);
        input.erase(0, newline + 1);
        if (line.empty()) continue;
        try {
          handle(Json::parse(line));
        } catch (const std::exception&) {
          // Ignore malformed input. The hub and autograder always send valid JSON.
        }
      }
    }
    running = false;
    int socket_fd = fd.exchange(-1);
    if (socket_fd >= 0) ::close(socket_fd);

    std::lock_guard<std::mutex> lock(state_mutex);
    for (auto& entry : pending_calls) {
      auto& pending = entry.second;
      {
        std::lock_guard<std::mutex> pending_lock(pending->mutex);
        pending->done = true;
      }
      pending->ready.notify_all();
    }
  }

  void handle(const Json& message) {
    const Json* op = message.get("op");
    if (!op || !op->is_string()) return;

    if (op->as_string() == "publish") {
      const Json* topic = message.get("topic");
      const Json* body = message.get("msg");
      if (!topic || !topic->is_string() || !body) return;
      TopicCallback callback;
      {
        std::lock_guard<std::mutex> lock(state_mutex);
        auto it = topic_callbacks.find(topic->as_string());
        if (it != topic_callbacks.end()) callback = it->second;
      }
      if (callback) callback(*body);
    } else if (op->as_string() == "call_service") {
      const Json* service = message.get("service");
      const Json* id = message.get("id");
      const Json* args = message.get("args");
      if (!service || !service->is_string() || !id || !args) return;
      ServiceCallback callback;
      {
        std::lock_guard<std::mutex> lock(state_mutex);
        auto it = service_callbacks.find(service->as_string());
        if (it != service_callbacks.end()) callback = it->second;
      }
      if (!callback) return;

      // A service callback gets its own thread. This lets an A* callback call
      // /heap_sort and wait while the receiver thread accepts the response.
      std::thread([this, callback, service_name = service->as_string(), request_id = *id,
                   arguments = *args] {
        ServiceResult result;
        try {
          result = callback(arguments);
        } catch (const std::exception& error) {
          result = {false, Json::Object{}, error.what()};
        }
        try {
          send(Json::Object{{"op", "service_response"}, {"service", service_name},
                            {"id", request_id}, {"values", result.values},
                            {"result", result.result}, {"status", result.status}});
        } catch (const std::exception&) {
        }
      }).detach();
    } else if (op->as_string() == "service_response") {
      const Json* id = message.get("id");
      const Json* values = message.get("values");
      const Json* result = message.get("result");
      if (!id || !id->is_string() || !values || !result || !result->is_bool()) return;
      std::shared_ptr<PendingCall> pending;
      {
        std::lock_guard<std::mutex> lock(state_mutex);
        auto it = pending_calls.find(id->as_string());
        if (it != pending_calls.end()) pending = it->second;
      }
      if (!pending) return;
      std::string status;
      const Json* status_value = message.get("status");
      if (status_value && status_value->is_string()) status = status_value->as_string();
      {
        std::lock_guard<std::mutex> lock(pending->mutex);
        pending->reply = {result->as_bool(), *values, status};
        pending->done = true;
      }
      pending->ready.notify_one();
    }
    // Status messages are informational, so this small client ignores them.
  }

  std::string name;
  std::atomic<int> fd{-1};
  std::atomic<bool> running{true};
  std::atomic<std::uint64_t> next_call_id{1};
  std::thread receiver;
  std::mutex send_mutex;
  std::mutex state_mutex;
  std::unordered_map<std::string, TopicCallback> topic_callbacks;
  std::unordered_map<std::string, ServiceCallback> service_callbacks;
  std::unordered_map<std::string, std::shared_ptr<PendingCall>> pending_calls;
};

Node::Node(std::string name, std::string host, std::uint16_t port)
    : impl_(std::make_unique<Impl>(std::move(name), host, port)) {}
Node::~Node() = default;

void Node::advertise(const std::string& topic, const std::string& type) {
  impl_->send(Json::Object{{"op", "advertise"}, {"topic", topic}, {"type", type}});
}

void Node::unadvertise(const std::string& topic) {
  impl_->send(Json::Object{{"op", "unadvertise"}, {"topic", topic}});
}

void Node::publish(const std::string& topic, const Json& message) {
  impl_->send(Json::Object{{"op", "publish"}, {"topic", topic}, {"msg", message}});
}

void Node::subscribe(const std::string& topic, const std::string& type,
                     TopicCallback callback) {
  {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    impl_->topic_callbacks[topic] = std::move(callback);
  }
  impl_->send(Json::Object{{"op", "subscribe"}, {"topic", topic}, {"type", type}});
}

void Node::unsubscribe(const std::string& topic) {
  {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    impl_->topic_callbacks.erase(topic);
  }
  impl_->send(Json::Object{{"op", "unsubscribe"}, {"topic", topic}});
}

void Node::advertise_service(const std::string& service, const std::string& type,
                             ServiceCallback callback) {
  {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    impl_->service_callbacks[service] = std::move(callback);
  }
  impl_->send(Json::Object{{"op", "advertise_service"}, {"service", service},
                           {"type", type}});
}

void Node::unadvertise_service(const std::string& service) {
  {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    impl_->service_callbacks.erase(service);
  }
  impl_->send(Json::Object{{"op", "unadvertise_service"}, {"service", service}});
}

ServiceResult Node::call_service(const std::string& service, const Json& args,
                                 std::chrono::milliseconds timeout) {
  std::string id = impl_->name + "-call-" + std::to_string(impl_->next_call_id++);
  auto pending = std::make_shared<Impl::PendingCall>();
  {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    impl_->pending_calls[id] = pending;
  }
  try {
    impl_->send(Json::Object{{"op", "call_service"}, {"service", service},
                             {"id", id}, {"args", args}});
  } catch (const std::exception& error) {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    impl_->pending_calls.erase(id);
    return {false, Json::Object{}, error.what()};
  }

  std::unique_lock<std::mutex> lock(pending->mutex);
  bool received = pending->ready.wait_for(lock, timeout, [&] { return pending->done; });
  ServiceResult reply = received
                            ? pending->reply
                            : ServiceResult{false, Json::Object{}, "service call timed out"};
  lock.unlock();
  {
    std::lock_guard<std::mutex> state_lock(impl_->state_mutex);
    impl_->pending_calls.erase(id);
  }
  return reply;
}

void Node::wait() {
  if (impl_->receiver.joinable()) impl_->receiver.join();
}

}  // namespace autorob
