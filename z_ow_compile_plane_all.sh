#!/bin/bash
echo hello
set -e #any subsequent commands which fail will cause the shell script to exit immediately
set -x #echo on

version=v055
vehicle=plane

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
compile omnibusf4 #too large
compile omnibusf4pro #too large

