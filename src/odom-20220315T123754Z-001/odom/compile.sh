#!/bin/bash
g++ main.cpp common/*.cpp -Icommon -lzmq -pthread -std=c++14 -O0 -g -o odom -Wall
