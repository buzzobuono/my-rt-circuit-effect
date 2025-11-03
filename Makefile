CXX = g++
CXXFLAGS = -std=c++17 -O3 #-Wall -Wextra -fsanitize=address
INCLUDES = -I/usr/include/eigen3 -Iinclude
LIBS_SNDFILE = -lsndfile
LIBS_PORTAUDIO = -lportaudio

all: wav_processor sine_input_processor wav_streaming_processor dc_analisys

wav_processor: clean_wav_processor create_bin_folder
	$(CXX) $(CXXFLAGS) $(INCLUDES) src/wav_processor.cpp -o bin/wav_processor $(LIBS_SNDFILE) ${DEBUG}

sine_input_processor: clean_sine_input_processor create_bin_folder
	$(CXX) $(CXXFLAGS) $(INCLUDES) src/sine_input_processor.cpp -o bin/sine_input_processor ${DEBUG}

wav_streaming_processor: clean_wav_streaming_processor create_bin_folder
	$(CXX) $(CXXFLAGS) $(INCLUDES) src/wav_streaming_processor.cpp -o bin/wav_streaming_processor $(LIBS_SNDFILE) $(LIBS_PORTAUDIO) ${DEBUG}

dc_analisys: clean_dc_analisys create_bin_folder
	$(CXX) $(CXXFLAGS) $(INCLUDES) src/dc_analisys.cpp -o bin/dc_analisys ${DEBUG}

clean_wav_processor:
	@rm -f bin/wav_processor

clean_sine_input_processor:
	@rm -f bin/sine_input_processor

clean_wav_streaming_processor:
	@rm -f bin/wav_streaming_processor

clean_dc_analisys:
	@rm -f bin/dc_analisys

create_bin_folder:
	@mkdir -p bin/