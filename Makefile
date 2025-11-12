CXX = g++
ifeq ($(origin TERMUX__PREFIX), environment)
    CXXFLAGS = --rtlib=compiler-rt -std=c++17 -O3 -march=native -DNDEBUG
    INCLUDES = -I$(TERMUX__PREFIX)/include/eigen3 -Iinclude
else
    CXXFLAGS = -std=c++17 -O3 -march=native -DNDEBUG #-Wall -Wextra -fsanitize=address 
    INCLUDES = -I/usr/include/eigen3 -Iinclude
endif
LIBS_SNDFILE = -lsndfile
LIBS_PORTAUDIO = -lportaudio

# LV2 Plugin configuration
PLUGIN_NAME = circuit_simulator
PLUGIN_SO = $(PLUGIN_NAME).so
LV2_BUNDLE = $(PLUGIN_NAME).lv2
USER_LV2_DIR = $(HOME)/.lv2
INSTALL_DIR = $(USER_LV2_DIR)/$(LV2_BUNDLE)
BIN_INSTALL_DIR = $(HOME)/bin

# LV2 compilation flags
LV2_CXXFLAGS = -fPIC -shared
LV2_INCLUDES = $(shell pkg-config --cflags lv2 2>/dev/null || echo "")

all: spicepedal spicepedal-stream

spicepedal: clean_spicepedal create_bin_folder
	$(CXX) $(CXXFLAGS) $(INCLUDES) src/spicepedal.cpp -o bin/spicepedal $(LIBS_SNDFILE) ${DEBUG}

spicepedal-stream: clean_spicepedal-stream create_bin_folder
	$(CXX) $(CXXFLAGS) $(INCLUDES) src/spicepedal_stream.cpp -o bin/spicepedal-stream $(LIBS_SNDFILE) $(LIBS_PORTAUDIO) ${DEBUG}

lv2: clean_lv2
	$(CXX) $(CXXFLAGS) $(LV2_CXXFLAGS) $(INCLUDES) $(LV2_INCLUDES) src/lv2_plugin.cpp -o lib/$(PLUGIN_SO) ${DEBUG}

install-lv2: lv2
	@mkdir -p $(INSTALL_DIR)
	@mkdir -p $(INSTALL_DIR)/circuits
	@cp lib/$(PLUGIN_SO) $(INSTALL_DIR)/
	@cp ttl/manifest.ttl $(INSTALL_DIR)/
	@cp ttl/circuit_simulator.ttl $(INSTALL_DIR)/
	@cp circuits/*.cir $(INSTALL_DIR)/circuits/
	@echo "Test with: jalv.gtk http://github.com/buzzobuono/circuit_simulator"

uninstall-lv2:
	rm -rf $(INSTALL_DIR)

# Test LV2 installation
test-lv2: install-lv2
	@echo ""
	@echo "Testing LV2 installation..."
	@if [ -d "$(INSTALL_DIR)" ]; then \
		echo "✓ Plugin directory exists"; \
	else \
		echo "✗ Plugin directory not found"; \
		exit 1; \
	fi
	@if command -v lv2ls >/dev/null 2>&1; then \
		if lv2ls | grep -q "circuit_simulator"; then \
			echo "✓ Plugin recognized by LV2"; \
		else \
			echo "⚠ Plugin not found in lv2ls"; \
		fi; \
	else \
		echo "⚠ lv2ls not available (install lilv-utils)"; \
	fi
	@echo "✓ Test complete"

install: all
	@mkdir -p $(BIN_INSTALL_DIR)
	@cp bin/spicepedal $(BIN_INSTALL_DIR)/
	@cp bin/spicepedal-stream $(BIN_INSTALL_DIR)/
	@chmod +x $(BIN_INSTALL_DIR)/spicepedal
	@chmod +x $(BIN_INSTALL_DIR)/spicepedal-stream
	@echo "✓ Binaries installed to ~/bin"
	@echo "✓ Make sure ~/bin is in your PATH"

# Uninstall binaries from ~/bin
uninstall:
	@rm -f $(BIN_INSTALL_DIR)/spicepedal
	@rm -f $(BIN_INSTALL_DIR)/spicepedal-stream
	@echo "✓ Binaries removed from ~/bin"

clean_spicepedal:
	@rm -f bin/pedal_spice

clean_spicepedal-stream:
	@rm -f bin/spicepedal-stream

clean_lv2:
	@rm -f lib/$(PLUGIN_SO)

create_bin_folder:
	@mkdir -p bin/ lib/
