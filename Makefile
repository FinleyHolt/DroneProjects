.PHONY: help check setup bootstrap clone-px4 sim clean clean-all test

# Default target
help:
	@echo "=========================================="
	@echo "LLMDrone Development Makefile"
	@echo "=========================================="
	@echo ""
	@echo "Setup targets:"
	@echo "  make check        - Check system requirements"
	@echo "  make bootstrap    - Install system dependencies"
	@echo "  make clone-px4    - Clone PX4-Autopilot v1.14.0 into .deps/"
	@echo "  make setup        - Run full setup (check + bootstrap + clone-px4)"
	@echo ""
	@echo "Simulation targets:"
	@echo "  make sim          - Build and launch PX4 SITL + Gazebo"
	@echo "  make sim-headless - Launch PX4 SITL without Gazebo GUI"
	@echo ""
	@echo "Cleanup targets:"
	@echo "  make clean        - Clean PX4 build artifacts"
	@echo "  make clean-all    - Remove all dependencies and state files"
	@echo ""
	@echo "Testing targets:"
	@echo "  make test         - Run PX4 SITL tests"
	@echo ""
	@echo "Before first use, run: make setup"
	@echo "Then source environment: source setup/env.sh"
	@echo ""

# Check system requirements
check:
	@bash setup/check_system.sh

# Install system dependencies
bootstrap:
	@bash setup/bootstrap.sh

# Clone PX4-Autopilot
clone-px4:
	@bash setup/clone_px4.sh

# Full setup sequence
setup: check bootstrap clone-px4
	@echo ""
	@echo "=========================================="
	@echo "Setup complete!"
	@echo "=========================================="
	@echo ""
	@echo "Next steps:"
	@echo "  1. Source environment: source setup/env.sh"
	@echo "  2. Launch simulation: make sim"
	@echo ""

# Build and launch PX4 SITL with Gazebo
sim:
	@if [ ! -d .deps/PX4-Autopilot ]; then \
		echo "Error: PX4-Autopilot not found. Run 'make setup' first."; \
		exit 1; \
	fi
	@echo "Building and launching PX4 SITL + Gazebo..."
	@echo "Press Ctrl+C to stop the simulation"
	@echo ""
	cd .deps/PX4-Autopilot && make px4_sitl gazebo

# Launch PX4 SITL without Gazebo GUI (headless mode)
sim-headless:
	@if [ ! -d .deps/PX4-Autopilot ]; then \
		echo "Error: PX4-Autopilot not found. Run 'make setup' first."; \
		exit 1; \
	fi
	@echo "Building and launching PX4 SITL (headless mode)..."
	@echo ""
	cd .deps/PX4-Autopilot && HEADLESS=1 make px4_sitl gazebo

# Clean PX4 build artifacts (keeps cloned repo)
clean:
	@if [ -d .deps/PX4-Autopilot ]; then \
		echo "Cleaning PX4 build artifacts..."; \
		cd .deps/PX4-Autopilot && make clean; \
		echo "Clean complete."; \
	else \
		echo "Nothing to clean (PX4 not installed)"; \
	fi

# Remove all dependencies and state (complete reset)
clean-all:
	@echo "Warning: This will remove all dependencies and state files."
	@read -p "Continue? [y/N] " -n 1 -r; \
	echo; \
	if [[ $$REPLY =~ ^[Yy]$$ ]]; then \
		echo "Removing .deps/ and .setup_state/..."; \
		rm -rf .deps .setup_state; \
		echo "Clean complete. Run 'make setup' to reinstall."; \
	else \
		echo "Cancelled."; \
	fi

# Run PX4 SITL tests
test:
	@if [ ! -d .deps/PX4-Autopilot ]; then \
		echo "Error: PX4-Autopilot not found. Run 'make setup' first."; \
		exit 1; \
	fi
	@echo "Running PX4 SITL tests..."
	@echo ""
	cd .deps/PX4-Autopilot && make px4_sitl_default gazebo-headless tests
