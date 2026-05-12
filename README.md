# 528-IMU-VR-project
Using an IMU in order to capture positional data, to update a rendered box on a computer

To use this open in visual studio with the associated solution file, anything else will not work because the include libraries need to have a specified filepath. First run the 
udp_bridge file with the format python udp_bridge.py --port COM5 --verbose (change COM5 with whatever port your esp is hooked up to). This will not work unless your ESP output is 
in the format of AX:<float> AY:<float> AZ:<float> | GX:<float> GY:<float> GZ:<float>, this is what pyserial looks for to parse. Firmware is provided in firmware.c.

After that setup, if the python file is active, click the green run button in visual studio and you will be met with a cube that rotates accordingly. 
