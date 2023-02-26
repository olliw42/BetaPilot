#!/bin/bash
# run in cygwin with ./z_ow_compile_copter_all.sh
echo hello
set -e #any subsequent commands which fail will cause the shell script to exit immediately
set -x #echo on

version=v0572
vehicle=copter

compile(){
  ./waf configure --board $1 #$1 is board
  ./waf $vehicle
  mv build/$1/bin/ardu$vehicle.apj bp_compiled_binaries/beta$vehicle-$version-$1.apj
}

#./waf clean

compile MatekH743
compile CubeOrange
compile MatekF765-Wing
compile fmuv3
compile omnibusf4 # for shuricus, needs special treatment for memory reason
compile omnibusf4pro
compile MatekF405-CAN # for patrick99

