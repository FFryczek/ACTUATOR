ACTUATOR
PID-controlled actuator using:

-Raspberry Pi Pico
-TMAG5273 3D Hall sensor (I2C)
-ESC (Electronic Speed Controller)
-BLDC motor

The actuator rotates to a user-defined target angle using PID control. Currently, it supports unidirectional rotation via PWM.

Demo
See it in action:
https://youtube.com/shorts/4DSmeha2O8s

Getting Started
Clone the repo:
git clone https://github.com/FFryczek/ACTUATOR.git

Open in PlatformIO (VS Code).

Connect the Raspberry Pi Pico.

Build and upload:
platformio run
platformio upload
platformio device monitor

Notes
Currently unidirectional only.

Consider bidirectional ESC (e.g. BLHeli) for full control.

License
MIT License