CXX = g++
CXXFLAGS = -std=c++17 -O3 #-Wall -Wextra -fsanitize=address
INCLUDES = -I/usr/include/eigen3
LIBS_SNDFILE = -lsndfile

all: wav_processor

wav_processor: wav_processor.cpp clean
	$(CXX) $(CXXFLAGS) $(INCLUDES) wav_processor.cpp -o wav_processor $(LIBS_SNDFILE) ${DEBUG}

clean:
	rm -f wav_processor
