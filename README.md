# UnderwaterAcoustics
This is the code required for the Underwater Acoustics STM32 Acquisition SubSystem

main.c contains the necessary code to acquire an analog signal sampling at about 700kHz and storing the result to a .DAT file that contains the analog values.  Although the file has a WAV file header, the values are 16 bit unsigned values so the DAT file needs to be post-processed to convert it to a WAV file.  This post-processing is done when saving to the raspberry pi.

*.ioc contains all the necessary CubeIDE configuration data to run main.c on the Seeed board

uw_acoust_client.py is the app that runs on the raspberry pi and controls the Seeed STM32 acquisition system
