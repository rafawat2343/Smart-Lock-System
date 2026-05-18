# Smart-Lock-System
This is an embedded system for Smart Lock System where users can unlock/lock door by typing password using their keypad or by scanning their fingerprint using fingerprint sensor. Also, a new user can enroll for the authorization of the lock system.  There is an ultrasonic sensor for sensing the presence of a object within 5 cm of the sensor. If it finds no object within 5 cm during 10 seconds, then it will turn off the OLED display. If it again detects any objects within 5 cm while the OLED display being turned off, then it will turn on the system along with the display.

The following sensors/hardware are used for making the system:
1. STM32F103C8T6 blue pill MCU
2.  0.96" OLED display using I2C connection protocol
3. HC-SR307 Ultrasonic Sensor
4. AS608 fingerprint sensor using UART connection protocol
5. 4*4 membrane keypad
6. 5V active buzzer
7. Built-in LED and an LED
