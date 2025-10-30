CXX = g++
CXXFLAGS = -std=c++17 -O3 #-Wall -Wextra -fsanitize=address
INCLUDES = -I/usr/include/eigen3 -Iinclude
LIBS_SNDFILE = -lsndfile
LIBS_PORTAUDIO = -lportaudio

all: wav_processor sine_input_processor wav_playback_processor

wav_processor: clean_wav_processor
	$(CXX) $(CXXFLAGS) $(INCLUDES) src/wav_processor.cpp -o wav_processor $(LIBS_SNDFILE) ${DEBUG}

sine_input_processor: clean_sine_input_processor
	$(CXX) $(CXXFLAGS) $(INCLUDES) src/sine_input_processor.cpp -o sine_input_processor ${DEBUG}

wav_playback_processor: clean_wav_processor
	$(CXX) $(CXXFLAGS) $(INCLUDES) src/wav_playback_processor.cpp -o wav_playback_processor $(LIBS_SNDFILE) $(LIBS_PORTAUDIO) ${DEBUG}

clean_wav_processor:
	rm -f wav_processor

clean_sine_input_processor:
	rm -f sine_input_processor

clean_wav_playback_processor:
	rm -f wav_playback_processor