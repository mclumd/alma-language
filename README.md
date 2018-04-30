# alma-language
ALMA language module using ROS

Speech recognition using CMUSphinx, which is then processed by ALMA, with ROS topic mechanisms for communication. As such, this has the following dependencies:
1) [PocketSphinx](https://github.com/cmusphinx/pocketsphinx), 5prealpha or newer. Note that the microphone_decoder script makes reference to the directory of the Sphinx install, specifically the models such as for the English language.
2) [ALMA](https://github.com/mclumd/Alma). Note that the alma_node script makes reference to the directory of the compiled ALMA binary.
3) [SWI Prolog](http://www.swi-prolog.org/), required to compile and run ALMA.
4) [ROS](http://www.ros.org/), Indigo or newer.

Running the demo:
