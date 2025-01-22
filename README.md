# serial_bus_servo_controller_cpp
This is a C++ library to control serial servos (such as LX-224, or HT-45) connected to a serial bus servo controller from a windows machine or a linux machine (such as Raspberry Pi).
Is is a fork from @aakmsk's work from (this repository)[git@github.com:Laroto/serial_bus_servo_controller_cpp.git]

LX-224 (https://www.hiwonder.hk/products/hiwonder-lx-224-intelligent-serial-bus-servo)</br>
serial bus servo controller (https://www.hiwonder.hk/collections/servo/products/serial-bus-servo-controller)

# Introduction
In order to make a robot, I used hiwonder's servo (LX-224) and control board (serial bus servo controller), and I tried to control it from python by connecting raspberry pi and control board via USB serial connection. However, there was not much information available, it took a lot of time until I was able to use it. 

In consideration of future use, I created a python module to control the servo from a PC through the control board.

I hope it will be usefull for those who are looking for the same kind of usage as me.

# Connection method
The main connection procedure is in the following three steps.

1. Connect the serial pins of the control board to the USB serial converter.
1. Connect the USB serial converter to the PC (such as raspberry pi).
1. Connect servos and power supply as necessary.

![](https://raw.githubusercontent.com/aakmsk/serial_bus_servo_controller/main/images/img.jpg)

# Download the library, compile and execute the example
## Terminal:
```
git clone https://github.com/Laroto/serial_bus_servo_controller_cpp.git
cd serial_bus_servo_controller_cpp
g++ -Iinclude -o full_example src/examples/full_example.cpp
./full_example

```

# Update history
|date|Details|
|----|----|
|2025/01/22|cpp port|
|2021/08/20|first commit|
