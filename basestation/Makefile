all: basestation

.phony: run

CFLAGS=-g `pkg-config --cflags --libs libmongoc-1.0` # libbson-1.0`
CC=clang

basestation: basestation.c db.c serial.c 

run: basestation
	./basestation
