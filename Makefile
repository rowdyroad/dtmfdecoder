CC=g++
CFLAGS=-std=c++11

.PHONY: all

all: example

example:
	mkdir -p build
	$(CC) example.cpp $(CFLAGS) -o build/example


