#!/usr/bin/env python3
"""Small autograder-style test for the hub and the C++ Node API."""

import json
import socket
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ADDRESS = ("127.0.0.1", 9095)


class Client:
    def __init__(self):
        self.socket = socket.create_connection(ADDRESS, timeout=2)
        self.socket.settimeout(2)
        self.buffer = b""

    def send(self, message):
        line = json.dumps(message, separators=(",", ":")) + "\n"
        self.socket.sendall(line.encode())

    def receive(self):
        while b"\n" not in self.buffer:
            data = self.socket.recv(65536)
            if not data:
                raise RuntimeError("gateway closed the connection")
            self.buffer += data
        line, self.buffer = self.buffer.split(b"\n", 1)
        return json.loads(line)

    def call(self, service, arguments, request_id):
        self.send({"op": "call_service", "service": service,
                   "id": request_id, "args": arguments})
        while True:
            response = self.receive()
            if response.get("op") == "service_response" and response.get("id") == request_id:
                return response

    def close(self):
        self.socket.close()


def wait_for_hub():
    for _ in range(100):
        probe = socket.socket()
        if probe.connect_ex(ADDRESS) == 0:
            probe.close()
            return
        probe.close()
        time.sleep(0.02)
    raise RuntimeError("hub did not start")


def main():
    hub = subprocess.Popen([ROOT / "build/autorob_hub"])
    demo = None
    clients = []
    try:
        wait_for_hub()
        demo = subprocess.Popen([ROOT / "build/demo_node"])

        # Wait until the C++ demo node has advertised its service.
        caller = Client()
        clients.append(caller)
        for attempt in range(50):
            reply = caller.call("/demo/add", {"a": 2, "b": 5}, f"try-{attempt}")
            if reply["result"]:
                break
            time.sleep(0.02)
        assert reply["values"] == {"sum": 7}

        # Check that the caller's opaque JSON ID comes back unchanged.
        opaque_id = {"client": 7, "request": [1, 2]}
        reply = caller.call("/demo/add", {"a": -1.5, "b": 0.5}, opaque_id)
        assert reply["id"] == opaque_id
        assert reply["values"] == {"sum": -1}

        # Check C++ subscriber callback -> C++ publish -> external subscriber.
        listener = Client()
        publisher = Client()
        clients += [listener, publisher]
        listener.send({"op": "subscribe", "topic": "/demo/output",
                       "type": "demo/Number", "id": "sub-1"})
        assert listener.receive()["level"] == "info"
        message = {"value": 123}
        publisher.send({"op": "publish", "topic": "/demo/input", "msg": message})
        publication = listener.receive()
        assert publication == {"op": "publish", "topic": "/demo/output", "msg": message}

        # Different topic names must remain isolated.
        listener.socket.settimeout(0.2)
        publisher.send({"op": "publish", "topic": "/different", "msg": message})
        try:
            listener.receive()
            raise AssertionError("message leaked from another topic")
        except socket.timeout:
            pass

        print("PASS: topics, callbacks, services, and ID correlation work")
    finally:
        for client in clients:
            client.close()
        if demo is not None:
            demo.terminate()
            demo.wait(timeout=2)
        hub.terminate()
        hub.wait(timeout=2)


if __name__ == "__main__":
    main()
