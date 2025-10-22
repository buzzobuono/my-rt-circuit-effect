CXX = g++
CXXFLAGS = -std=c++17 -O3 #-Wall -Wextra -fsanitize=address
INCLUDES = -I/usr/include/eigen3 -Iinclude
LIBS_SNDFILE = -lsndfile

SRCS = $(wildcard src/*.cpp)
TARGET = wav_processor

all: ${TARGET}

$(TARGET): clean
	$(CXX) $(CXXFLAGS) $(INCLUDES) src/wav_processor.cpp -o $(TARGET) $(LIBS_SNDFILE) ${DEBUG}

clean:
	rm -f $(TARGET)

