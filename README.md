# stm32_music_player
Music Player from SD card and EQ with GUI on STM32 NUCLEO-H743ZI board

## Description
Plays audio WAV files, the project is set up for 16/44.1k (CD quality) but any other bitrate should work.

They are read from SD card in SDIO mode, 3 band equalizer is applied but this MCU can handle more DSP effects.

Graphical User Interface with touchscreen built with LVGL and EEZ Studio allows for viewing the list of songs and changing through them, and sliders for volume and EQ gain.

## Additional Components:
- PCM5102A DAC module
- SD card module (SDIO)
- Waveshare 4inch TFT Touch Shield

## Acknowledgments:
[STM32World](https://www.youtube.com/@stm32world)

[Phil's Lab](https://www.youtube.com/@PhilsLab/)
