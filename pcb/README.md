![PCB Version 1](/pcb/pcb_v1.png)
This is PCB Version 1.
![PCB Version 2](/pcb/pcb_v2.png)
This is PCB version 2

# Common notice for using this PCB
* ESP32CAM require 5V power to work. The place for power module can be for example a boost module from 3.7V to 5V or a buck module from 7.4V to 5V. Marking on the board here is a tiny boost module, Use anything that fits is fine.
* Power switch has three contact points, two of them close to the battery connector are bridged together. if you want to extend it to your outer case, only the two pins to the left  are required.
* Evan the smallest SMD button are bigger than 2.54mm wide. So there are four contact points reserved on the board. Two of them on the left are grouped and the other two are grouped as well. To extend it just connect the center two is enough.
* By default power module is designed to place on the botton side of this PCB. Then the contact pins will poke out and might interference with ESP32CAM if you also want to minimize overall height. Remember to trim them down and try to cover them with non-conductive tape.
* Or if height is not a problem, try flipping the power module upside down and put it on the top side and then place the ESP32CAM on top of it with female dupont connectors.

# Difference in ver. 1 and ver. 2

## Version 1:
* By default motors powered by battery directly, that is 3.7v for single cell or 7.4v for 2S battery pack. But both ESP32CAM and servo are using 5V from the power module. To change motor voltage to 5V, the power trace is the long one on the back side of the PCB.
* Connecters are placed along center line and might need to connect them from bottom before soldering ESP32CAM and mortor driver module.

## Version 2:
* Battery power directly send to power module first then convert to 5V for all modules. If you want to supply battery power directly to motor driver module, the power trace is on the back side as well.
* Motor connectors are placing near the edge, and LED connectors moved closer to motor driver module. This is easier to connect everything from the top side.