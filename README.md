# BetaPilot 4.0

This is the BetaPilot (inlcuding BetaCopter, BetaPlane) fork of the ArduPilot project.

I use it for developing new features, mainly related to the [STorM32 gimbal controller](http://www.olliw.eu/2013/storm32bgc/) project.


## Usage and Release Notes ##

Please see http://www.olliw.eu/storm32bgc-wiki/Using_STorM32_with_ArduPilot for information on the usage and features of BetaPilot (BetaCopter/BetaPlane).


## Build, Compile Notes ##

In order to build BetaPilot 4.0 (BetaCopter/BetaPlane), do this:

- Follow the instructions in the ArduPilot wiki for installing the build environment.
- Clone this fork and checkout the desired BetaPilot (BetaCopter/BetaPlane) branch.
- Ensure that all git submodules are there. Run git submodule update --init --recursive.
- Copy the three files ardupilotmega.xml, common.xml, and storm32_4ap.xml from the \mavlink_withmostbasicadditions folder to the ardupilot\modules\mavlink\message_definitions\v1.0 folder. This will overwrite files existing there. (This step is crucuial since ArduPilot does not support the latest MAVLink standard, which STorM32 needs).
- Compile for your board follow the instructions in the ArduPilot wiki for compiling.


## Acknowledgements and License ##

BetaPilot (BetaCopter, BetaPlane) is based on the ArduPilot project, see: 

- ArduPilot home: http://ardupilot.com/ardupilot/index.html

- ArduPilot github repository: https://github.com/ArduPilot/ardupilot

BetaPilot (BetaCopter, BetaPlane) inherits the ArduPilot licence(s). ArduPilot is licensed under GNU GPL version 3, see:

- ArduPilot license, overview: http://ardupilot.org/dev/docs/license-gplv3.html

- Full text of license: https://github.com/ArduPilot/ardupilot/blob/master/COPYING.txt

