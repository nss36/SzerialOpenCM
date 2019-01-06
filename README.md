# SzerialOpenCM
Nick Szczecinski, Case Western Reserve University, 2018.
Library for interfacing a microcontroller with AnimatLab via a serial connection, i.e. Szerial.
To use, clone this repo into your C:\Program Files (x86)\Arduino\libraries folder. This should enable the user to include it in Arduino sketches with the #include "Szerial.h" command.
Then, one must instantiate the Szerial object: Szerial serialName(serialPort, numInputs, numOutputs);.