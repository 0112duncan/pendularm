#include "rosbridge_server.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autorob {

struct RosbridgeServer::Impl {
  struct Client {
    Client(int socket_fd, std::uint64_t client_id) : fd(socket_fd), id(client_id) {}

    void close() {
      int socket_fd = fd.exchange(-1);
      if (socket_fd >= 0) {
        ::shutdown(socket_fd, SHUT_RDWR);
        ::close(socket_fd);
      }
    }

    bool send(const Json& message) {
      std::string line = message.dump() + '\n';
      std::lock_guard<std::mutex> lock(send_mutex);
      int socket_fd = fd.load();
      if (socket_fd < 0) return false;
      std::size_t sent = 0;
      while (sent < line.size()) {
        // Never let a client that stopped reading block the publisher thread.
        // Topic fan-out and service routing share these client threads, so
        // blocking here can otherwise freeze the whole runtime. A slow client
        // is disconnected and can reconnect instead.
        ssize_t count = ::send(socket_fd, line.data() + sent, line.size() - sent,
                               MSG_NOSIGNAL | MSG_DONTWAIT);
        if (count < 0) {
          if (errno == EINTR) continue;
          close();
          return false;
        }
        if (count == 0) {
          close();
          return false;
        }
        sent += static_cast<std::size_t>(count);
      }
      return true;
    }

    std::atomic<int> fd;
    std::uint64_t id;
    std::mutex send_mutex;
    std::mutex registration_mutex;
    std::unordered_set<std::string> publishers;
    std::unordered_set<std::string> subscriptions;
  };

  struct PendingService {
    std::weak_ptr<Client> caller;
    std::weak_ptr<Client> provider;
    Json caller_id;
    std::string service;
  };

  explicit Impl(Bus& bus_value) : bus(bus_value) {}
  ~Impl() { stop(); }

  static bool text_field(const Json& object, const std::string& key, std::string& value) {
    const Json* field = object.get(key);
    if (!field || !field->is_string()) return false;
    value = field->as_string();
    return true;
  }

  static void status(const std::shared_ptr<Client>& client, const std::string& level,
                     const std::string& message, const Json* id = nullptr) {
    Json::Object response{{"op", "status"}, {"level", level}, {"msg", message}};
    if (id) response["id"] = *id;
    client->send(Json(std::move(response)));
  }

  static Json service_response(const std::string& service, const Json& id,
                               const ServiceResult& reply) {
    return Json::Object{{"op", "service_response"}, {"service", service},
                        {"id", id}, {"values", reply.values},
                        {"result", reply.result}, {"status", reply.status}};
  }

  bool run(const std::string& address, std::uint16_t port) {
    listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
      std::cerr << "socket: " << std::strerror(errno) << '\n';
      return false;
    }
    int yes = 1;
    ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in endpoint{};
    endpoint.sin_family = AF_INET;
    endpoint.sin_port = htons(port);
    if (::inet_pton(AF_INET, address.c_str(), &endpoint.sin_addr) != 1 ||
        ::bind(listen_fd, reinterpret_cast<sockaddr*>(&endpoint), sizeof(endpoint)) < 0 ||
        ::listen(listen_fd, 64) < 0) {
      std::cerr << "cannot listen on " << address << ':' << port << ": "
                << std::strerror(errno) << '\n';
      stop();
      return false;
    }

    std::cerr << "ROS-like hub listening on " << address << ':' << port << '\n';
    while (!stopping) {
      int client_fd = ::accept(listen_fd, nullptr, nullptr);
      if (client_fd < 0) {
        if (errno == EINTR) continue;
        if (stopping) break;
        continue;
      }
      auto client = std::make_shared<Client>(client_fd, next_client_id++);
      {
        std::lock_guard<std::mutex> lock(mutex);
        clients[client->id] = client;
      }
      std::thread([this, client] { client_loop(client); }).detach();
    }
    return true;
  }

  void stop() {
    if (stopping.exchange(true)) return;
    int socket_fd = listen_fd.exchange(-1);
    if (socket_fd >= 0) {
      ::shutdown(socket_fd, SHUT_RDWR);
      ::close(socket_fd);
    }
    std::vector<std::shared_ptr<Client>> active;
    {
      std::lock_guard<std::mutex> lock(mutex);
      for (auto& entry : clients) active.push_back(entry.second);
    }
    for (const auto& client : active) client->close();
  }

  void client_loop(const std::shared_ptr<Client>& client) {
    constexpr std::size_t maximum_line_size = 16 * 1024 * 1024;
    std::string input;
    char buffer[8192];
    for (;;) {
      int socket_fd = client->fd.load();
      if (socket_fd < 0) break;
      ssize_t count = ::recv(socket_fd, buffer, sizeof(buffer), 0);
      if (count == 0) break;
      if (count < 0) {
        if (errno == EINTR) continue;
        break;
      }
      input.append(buffer, static_cast<std::size_t>(count));
      if (input.size() > maximum_line_size && input.find('\n') == std::string::npos) break;
      for (;;) {
        std::size_t newline = input.find('\n');
        if (newline == std::string::npos) break;
        std::string line = input.substr(0, newline);
        input.erase(0, newline + 1);
        if (line.empty()) continue;
        try {
          handle(client, Json::parse(line));
        } catch (const std::exception& error) {
          status(client, "error", std::string("bad JSON: ") + error.what());
        }
      }
    }
    client->close();
    remove_client(client);
  }

  void handle(const std::shared_ptr<Client>& client, const Json& request) {
    const Json* op_value = request.get("op");
    const Json* request_id = request.get("id");
    if (!op_value || !op_value->is_string()) {
      status(client, "error", "request needs a string op", request_id);
      return;
    }
    const std::string& op = op_value->as_string();

    if (op == "advertise" || op == "unadvertise") {
      std::string topic;
      if (!text_field(request, "topic", topic)) {
        status(client, "error", op + " needs topic", request_id);
        return;
      }
      {
        std::lock_guard<std::mutex> lock(client->registration_mutex);
        if (op == "advertise") client->publishers.insert(topic);
        else client->publishers.erase(topic);
      }
      status(client, "info", op + "d " + topic, request_id);
    } else if (op == "subscribe" || op == "unsubscribe") {
      std::string topic;
      if (!text_field(request, "topic", topic)) {
        status(client, "error", op + " needs topic", request_id);
        return;
      }
      {
        std::lock_guard<std::mutex> lock(client->registration_mutex);
        if (op == "subscribe") client->subscriptions.insert(topic);
        else client->subscriptions.erase(topic);
      }
      std::string message = op == "subscribe" ? "subscribed to " : "unsubscribed from ";
      status(client, "info", message + topic, request_id);
    } else if (op == "publish") {
      publish(client, request, request_id);
    } else if (op == "advertise_service") {
      advertise_service(client, request, request_id);
    } else if (op == "unadvertise_service") {
      unadvertise_service(client, request, request_id);
    } else if (op == "call_service") {
      call_service(client, request);
    } else if (op == "service_response") {
      receive_service_response(client, request);
    } else {
      status(client, "error", "unknown op: " + op, request_id);
    }
  }

  void publish(const std::shared_ptr<Client>& sender, const Json& request,
               const Json* request_id) {
    (void)sender;
    std::string topic;
    const Json* message = request.get("msg");
    if (!text_field(request, "topic", topic) || !message) {
      status(sender, "error", "publish needs topic and msg", request_id);
      return;
    }

    bus.publish(topic, *message);
    std::vector<std::shared_ptr<Client>> subscribers;
    {
      std::lock_guard<std::mutex> lock(mutex);
      for (const auto& entry : clients) {
        const auto& possible_subscriber = entry.second;
        std::lock_guard<std::mutex> registration_lock(possible_subscriber->registration_mutex);
        if (possible_subscriber->subscriptions.count(topic)) {
          subscribers.push_back(possible_subscriber);
        }
      }
    }
    Json outgoing = Json::Object{{"op", "publish"}, {"topic", topic}, {"msg", *message}};
    for (const auto& subscriber : subscribers) subscriber->send(outgoing);
  }

  void advertise_service(const std::shared_ptr<Client>& client, const Json& request,
                         const Json* request_id) {
    std::string service;
    if (!text_field(request, "service", service)) {
      status(client, "error", "advertise_service needs service", request_id);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex);
      service_providers[service] = client;
    }
    status(client, "info", "advertised service " + service, request_id);
  }

  void unadvertise_service(const std::shared_ptr<Client>& client, const Json& request,
                           const Json* request_id) {
    std::string service;
    if (!text_field(request, "service", service)) {
      status(client, "error", "unadvertise_service needs service", request_id);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex);
      auto it = service_providers.find(service);
      if (it != service_providers.end()) {
        auto owner = it->second.lock();
        if (!owner || owner->id == client->id) service_providers.erase(it);
      }
    }
    status(client, "info", "unadvertised service " + service, request_id);
  }

  void call_service(const std::shared_ptr<Client>& caller, const Json& request) {
    std::string service;
    const Json* caller_id = request.get("id");
    const Json* args = request.get("args");
    if (!caller_id || !args || !text_field(request, "service", service)) {
      status(caller, "error", "call_service needs service, id, and args", caller_id);
      return;
    }

    if (auto internal = bus.call_service(service, *args)) {
      caller->send(service_response(service, *caller_id, *internal));
      return;
    }

    std::shared_ptr<Client> provider;
    std::string provider_id;
    {
      std::lock_guard<std::mutex> lock(mutex);
      auto it = service_providers.find(service);
      if (it != service_providers.end()) provider = it->second.lock();
      if (provider) {
        provider_id = "hub-call-" + std::to_string(next_service_id++);
        pending_services[provider_id] = {caller, provider, *caller_id, service};
      }
    }
    if (!provider) {
      caller->send(service_response(
          service, *caller_id, {false, Json::Object{}, "service has no provider"}));
      return;
    }
    provider->send(Json::Object{{"op", "call_service"}, {"service", service},
                                {"id", provider_id}, {"args", *args}});
  }

  void receive_service_response(const std::shared_ptr<Client>& provider,
                                const Json& request) {
    std::string service;
    const Json* provider_id = request.get("id");
    const Json* values = request.get("values");
    const Json* result = request.get("result");
    if (!provider_id || !provider_id->is_string() || !values || !result ||
        !result->is_bool() || !text_field(request, "service", service)) {
      status(provider, "error", "invalid service_response", provider_id);
      return;
    }

    PendingService pending;
    bool found = false;
    {
      std::lock_guard<std::mutex> lock(mutex);
      auto it = pending_services.find(provider_id->as_string());
      if (it != pending_services.end()) {
        auto expected_provider = it->second.provider.lock();
        if (expected_provider && expected_provider->id == provider->id &&
            it->second.service == service) {
          pending = it->second;
          pending_services.erase(it);
          found = true;
        }
      }
    }
    if (!found) {
      status(provider, "error", "unknown service response id", provider_id);
      return;
    }

    auto caller = pending.caller.lock();
    if (!caller) return;
    std::string response_status;
    const Json* status_value = request.get("status");
    if (status_value && status_value->is_string()) response_status = status_value->as_string();
    caller->send(service_response(
        service, pending.caller_id, {result->as_bool(), *values, response_status}));
  }

  void remove_client(const std::shared_ptr<Client>& client) {
    struct FailedCall {
      std::shared_ptr<Client> caller;
      Json caller_id;
      std::string service;
    };
    std::vector<FailedCall> failed_calls;
    {
      std::lock_guard<std::mutex> lock(mutex);
      clients.erase(client->id);
      for (auto it = service_providers.begin(); it != service_providers.end();) {
        auto provider = it->second.lock();
        if (!provider || provider->id == client->id) it = service_providers.erase(it);
        else ++it;
      }
      for (auto it = pending_services.begin(); it != pending_services.end();) {
        auto caller = it->second.caller.lock();
        auto provider = it->second.provider.lock();
        if (!caller || caller->id == client->id) {
          it = pending_services.erase(it);
        } else if (!provider || provider->id == client->id) {
          failed_calls.push_back({caller, it->second.caller_id, it->second.service});
          it = pending_services.erase(it);
        } else {
          ++it;
        }
      }
    }
    for (const auto& failed : failed_calls) {
      failed.caller->send(service_response(
          failed.service, failed.caller_id,
          {false, Json::Object{}, "service provider disconnected"}));
    }
  }

  Bus& bus;
  std::atomic<int> listen_fd{-1};
  std::atomic<bool> stopping{false};
  std::atomic<std::uint64_t> next_client_id{1};
  std::uint64_t next_service_id = 1;
  std::mutex mutex;
  std::unordered_map<std::uint64_t, std::shared_ptr<Client>> clients;
  std::unordered_map<std::string, std::weak_ptr<Client>> service_providers;
  std::unordered_map<std::string, PendingService> pending_services;
};

RosbridgeServer::RosbridgeServer(Bus& bus) : impl_(std::make_unique<Impl>(bus)) {}
RosbridgeServer::~RosbridgeServer() = default;

bool RosbridgeServer::run(const std::string& address, std::uint16_t port) {
  return impl_->run(address, port);
}

void RosbridgeServer::stop() { impl_->stop(); }

}  // namespace autorob
