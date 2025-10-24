CXX = g++
CXXFLAGS = -std=c++17 -O3 #-Wall -Wextra -fsanitize=address
INCLUDES = -I/usr/include/eigen3 -Iinclude
LIBS_SNDFILE = -lsndfile

all: wav_processor sine_input_processor

wav_processor: clean
	$(CXX) $(CXXFLAGS) $(INCLUDES) src/wav_processor.cpp -o wav_processor $(LIBS_SNDFILE) ${DEBUG}

sine_input_processor: clean
	$(CXX) $(CXXFLAGS) $(INCLUDES) src/sine_input_processor.cpp -o sine_input_processor ${DEBUG}

clean:
	rm -f $(TARGET)

