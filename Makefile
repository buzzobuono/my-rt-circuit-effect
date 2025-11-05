CXX = g++
CXXFLAGS = -std=c++17 -O3 #-Wall -Wextra -fsanitize=address
INCLUDES = -I/usr/include/eigen3 -Iinclude
LIBS_SNDFILE = -lsndfile
LIBS_PORTAUDIO = -lportaudio

# LV2 Plugin configuration
PLUGIN_NAME = circuit_simulator
PLUGIN_SO = $(PLUGIN_NAME).so
LV2_BUNDLE = $(PLUGIN_NAME).lv2
USER_LV2_DIR = $(HOME)/.lv2
INSTALL_DIR = $(USER_LV2_DIR)/$(LV2_BUNDLE)

# LV2 compilation flags
LV2_CXXFLAGS = -fPIC -shared
LV2_INCLUDES = $(shell pkg-config --cflags lv2 2>/dev/null || echo "")

all: wav_processor sine_input_processor wav_streaming_processor dc_analisys lv2

wav_processor: clean_wav_processor create_bin_folder
	$(CXX) $(CXXFLAGS) $(INCLUDES) src/wav_processor.cpp -o bin/wav_processor $(LIBS_SNDFILE) ${DEBUG}

sine_input_processor: clean_sine_input_processor create_bin_folder
	$(CXX) $(CXXFLAGS) $(INCLUDES) src/sine_input_processor.cpp -o bin/sine_input_processor ${DEBUG}

wav_streaming_processor: clean_wav_streaming_processor create_bin_folder
	$(CXX) $(CXXFLAGS) $(INCLUDES) src/wav_streaming_processor.cpp -o bin/wav_streaming_processor $(LIBS_SNDFILE) $(LIBS_PORTAUDIO) ${DEBUG}

dc_analisys: clean_dc_analisys
	$(CXX) $(CXXFLAGS) $(INCLUDES) src/dc_analisys.cpp -o bin/dc_analisys ${DEBUG}

lv2: clean_lv2
	$(CXX) $(CXXFLAGS) $(LV2_CXXFLAGS) $(INCLUDES) $(LV2_INCLUDES) src/lv2_plugin.cpp -o lib/$(PLUGIN_SO) ${DEBUG}

install-lv2: lv2
	@mkdir -p $(INSTALL_DIR)
	@mkdir -p $(INSTALL_DIR)/circuits
	@cp lib/$(PLUGIN_SO) $(INSTALL_DIR)/
	@cp ttl/manifest.ttl $(INSTALL_DIR)/
	@cp ttl/circuit_simulator.ttl $(INSTALL_DIR)/
	@if [ -f circuits/bazz_fuss.cir ]; then \
		cp circuits/bazz_fuss.cir $(INSTALL_DIR)/circuits/; \
	fi
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

clean_wav_processor:
	@rm -f bin/wav_processor

clean_sine_input_processor:
	@rm -f bin/sine_input_processor

clean_wav_streaming_processor:
	@rm -f bin/wav_streaming_processor

clean_dc_analisys:
	@rm -f bin/dc_analisys

clean_lv2:
	@rm -f lib/$(PLUGIN_SO)

create_bin_folder:
	@mkdir -p bin/ lib/
