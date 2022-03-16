#!/bin/bash

g++ *.cpp fm_planner/*.cpp -I . -I /home/paolo/applications/Clothoids/lib/include/ `pkg-config --libs --cflags opencv` -L /home/paolo/applications/Clothoids/lib/lib -lClothoids_linux -std=c++14 -pthread -lzmq -fopenmp
