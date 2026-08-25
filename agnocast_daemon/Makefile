BUILD_DIR ?= build
TEST_BUILD_DIR ?= build_test

.PHONY: all test clean

all:
	cmake -S . -B $(BUILD_DIR)
	cmake --build $(BUILD_DIR) -j

test:
	cmake -S . -B $(TEST_BUILD_DIR) -DAGNOCAST_BUILD_TESTS=ON
	cmake --build $(TEST_BUILD_DIR) -j
	ctest --test-dir $(TEST_BUILD_DIR) --output-on-failure

clean:
	rm -rf $(BUILD_DIR) $(TEST_BUILD_DIR)
