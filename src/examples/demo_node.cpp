#include "node.hpp"

#include <iostream>

int main() {
  autorob::Node node("demo_node");

  // Publisher template: advertise once, then publish whenever you have data.
  node.advertise("/demo/output", "demo/Number");

  // Subscriber template: this callback runs whenever /demo/input is published.
  node.subscribe("/demo/input", "demo/Number", [&node](const autorob::Json& message) {
    std::cout << "received: " << message.dump() << '\n';
    node.publish("/demo/output", message);
  });

  // Service-server template: return result, values, and an optional status message.
  node.advertise_service("/demo/add", "demo/Add", [](const autorob::Json& args) {
    const autorob::Json* a = args.get("a");
    const autorob::Json* b = args.get("b");
    if (!a || !b || !a->is_number() || !b->is_number()) {
      return autorob::ServiceResult{false, autorob::Json::Object{},
                                    "args must contain numeric a and b"};
    }
    return autorob::ServiceResult{
        true, autorob::Json::Object{{"sum", a->as_number() + b->as_number()}}, ""};
  });

  node.wait();
}
