#!/bin/bash

g++ webcam_stream_server.cpp -o server `pkg-config --cflags --libs opencv4`
