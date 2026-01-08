#  ST3215-STM Controller
This project implements communication and control routines for ST3215 servos allowing you to command motion, read feedback (position, speed, load, etc.) using STM32 Nucleo-F103RB development board.

## What Is ST3215?

The **ST3215** is a programmable **serial bus servo** featuring :

* 360° absolute angle control with a high-precision magnetic encoder
* Programmable two-way feedback (position, speed, load, voltage)
* UART bus communication (up to 253 servos on a single bus)
* Servo/motor mode switchable by software
* Wide voltage range and high torque (up to 30 kg·cm)
* Useful for robotic arms, walkers, hexapods, and other closed-loop robotic systems ([Waveshare](https://www.waveshare.com/wiki/ST3215_Servo))
