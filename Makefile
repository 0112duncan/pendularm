CXX ?= g++
# GCC 13 emits a known false positive while moving std::variant values in the
# bundled Json type. Keep all other useful warnings enabled.
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic -Wno-maybe-uninitialized -pthread
CPPFLAGS ?= -Iinclude
SHELL := /bin/bash

NODE_CLIENT = src/middleware/node.cpp
HEADERS = $(wildcard include/*.hpp)
APP_NODES = arm_sim ik ik_action ik_trial integrator_step
APP_BINARIES = $(addprefix build/,$(APP_NODES))

.PHONY: build run web test test_integration test_project test_dynamics clean

build: build/autorob_hub $(APP_BINARIES)

build/autorob_hub: src/hub.cpp src/rosbridge_server.cpp src/middleware/bus.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/hub.cpp src/rosbridge_server.cpp \
		src/middleware/bus.cpp -o $@

build/integrator_step: src/nodes/integrator_step.cpp $(NODE_CLIENT) $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/nodes/integrator_step.cpp $(NODE_CLIENT) -o $@

build/arm_sim: src/nodes/arm_sim.cpp $(NODE_CLIENT) $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/nodes/arm_sim.cpp $(NODE_CLIENT) -o $@

build/ik: src/nodes/ik.cpp $(NODE_CLIENT) $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/nodes/ik.cpp $(NODE_CLIENT) -o $@

build/ik_action: src/nodes/ik_action.cpp $(NODE_CLIENT) $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/nodes/ik_action.cpp $(NODE_CLIENT) -o $@

build/ik_trial: src/nodes/ik_trial.cpp $(NODE_CLIENT) $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/nodes/ik_trial.cpp $(NODE_CLIENT) -o $@

build/integration_step_test: src/nodes/integration_step_test.cpp $(NODE_CLIENT) $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/nodes/integration_step_test.cpp $(NODE_CLIENT) -o $@

build/demo_node: src/examples/demo_node.cpp $(NODE_CLIENT) $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/examples/demo_node.cpp $(NODE_CLIENT) -o $@

run: build
	./scripts/run.sh

# In another terminal, after `make run`, serve the local browser UI at :8080.
web:
	node web/server.mjs

test: build/autorob_hub build/demo_node
	python3 tests/test_gateway.py

test_integration: build/autorob_hub build/integrator_step build/integration_step_test
	./build/autorob_hub & hub_pid=$$!; \
	trap 'kill $$hub_pid $$node_pid 2>/dev/null; wait $$hub_pid $$node_pid 2>/dev/null' EXIT; \
	./build/integrator_step & node_pid=$$!; \
	./build/integration_step_test

test_project: build
	python3 tests/test_project_protocol.py

test_dynamics: build
	python3 tests/test_dynamics_timeouts.py

clean:
	rm -rf build
