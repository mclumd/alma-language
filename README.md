# alma-language
ALMA language module using ROS

Speech recognition using CMUSphinx, which is then processed by ALMA, with ROS topic mechanisms for communication. As such, this has the following dependencies:
1) [PocketSphinx](https://github.com/cmusphinx/pocketsphinx), 5prealpha or newer
2) [ALMA](https://github.com/mclumd/Alma)
3) [SWI Prolog](http://www.swi-prolog.org/), required to compile and run ALMA
4) [ROS](http://www.ros.org/), Indigo or newer
