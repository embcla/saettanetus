#!/bin/bash
stty -F /dev/ttyO3 115200
#modificare con esistenza di c_urg-config
export LD_LIBRARY_PATH=../librerie:/usr/local/lib
./main

