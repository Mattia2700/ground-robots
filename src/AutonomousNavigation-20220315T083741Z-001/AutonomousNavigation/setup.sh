#!/bin/bash

curr_path=$(pwd)
cd ../Clothoids
make -j3
cd $curr_path
ln -sf ../../../Clothoids/lib/include libs/Clothoids/include
ln -sf ../../../Clothoids/lib/lib libs/Clothoids/lib

