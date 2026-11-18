CXX      ?= g++
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra -pedantic

SRC_DIR   := src
TEST_DIR  := tests
BUILD_DIR := build

LIB_SRCS := $(SRC_DIR)/PathNode.cpp $(SRC_DIR)/Algo.cpp
LIB_HDRS := $(SRC_DIR)/PathNode.h $(SRC_DIR)/Algo.h

.PHONY: all run test check-headers clean

all: $(BUILD_DIR)/main

$(BUILD_DIR):
	mkdir -p $@

$(BUILD_DIR)/main: $(LIB_SRCS) $(LIB_HDRS) $(SRC_DIR)/main.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(SRC_DIR) $(LIB_SRCS) $(SRC_DIR)/main.cpp -o $@

$(BUILD_DIR)/tests: $(LIB_SRCS) $(LIB_HDRS) $(wildcard $(TEST_DIR)/*.cpp $(TEST_DIR)/*.h) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(SRC_DIR) -I$(TEST_DIR) $(LIB_SRCS) $(wildcard $(TEST_DIR)/*.cpp) -o $@

run: $(BUILD_DIR)/main
	./$(BUILD_DIR)/main

test: check-headers $(BUILD_DIR)/tests
	./$(BUILD_DIR)/tests

check-headers:
	@./scripts/check_headers.sh $(SRC_DIR)

clean:
	rm -rf $(BUILD_DIR)
