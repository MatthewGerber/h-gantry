Home
* Content
{:toc}

# Two-Axis Fixed-Motor Gantry

<iframe src="https://gmail3021534.autodesk360.com/shares/public/SH286ddQT78850c0d8a4f16d8ed322db9a1e?mode=embed" width="800" height="600" allowfullscreen="true" webkitallowfullscreen="true" mozallowfullscreen="true"  frameborder="0"></iframe>

How do you control a two-axis linear gantry system? The simple solution is to [use two stepper motors](https://www.youtube.com/watch?v=uOSCsBbsX4w), 
one attached to each axis. The harder solution:  Use [two fixed-position stepper motors with a crazy, winding belt](https://www.youtube.com/watch?v=IkM2K7CsiHo).
There are advantages each way. The former is simpler to design and build; however, the motor mass is attached to each
axis, which is not ideal when the axes are changing direction quickly. In the latter design, the motors have fixed 
positions, so the motor mass is not attached to a moving axis. The control is more complicated, but this makes it 
interesting, which is also an advantage!

# Application:  Sand Art Table

<iframe src="https://gmail3021534.autodesk360.com/shares/public/SH90d2dQT28d5b60281107cf401bdab15bcf?mode=embed" width="800" height="600" allowfullscreen="true" webkitallowfullscreen="true" mozallowfullscreen="true"  frameborder="0"></iframe>

The gantry pulls a steel ball over sand using a magnet underneath, creating patterns. Use the "Explode Model" button and
slider to inspect the internal components of the model.

# Materials and Equipment

## Power
* [12V DC power supply](https://www.amazon.com/dp/B087LY94T6)
* [DC power jack adapter](https://www.amazon.com/JacobsParts-Adapter-Female-Pigtails-Security/dp/B00QJAW9F4)
* [12V to 5V converters](https://www.amazon.com/dp/B0BNQ5JNWZ)

## Motors
* [12V stepper motors](https://www.amazon.com/dp/B09YPZ3GCJ)

## Processors
* [Arduino UNO R4 Minima](https://www.amazon.com/dp/B0C78K4CD4)
* Raspberry [Pi 4 Model B](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/) or
  [Pi 5](https://www.raspberrypi.com/products/raspberry-pi-5/)

## Rails and Plates
* [2020 v-slot rails (3)](https://www.amazon.com/dp/B087PWGBM7)
* [V-wheel plates (2)](https://www.amazon.com/QWORK-V-Wheel-Aluminum-Profile-Printer/dp/B0CHYS3G96)

## Other Hardware

* [M4 bolts](https://www.amazon.com/dp/B073SX315M)
* [M2 bolts](https://www.amazon.com/dp/B07W5HBRMP)
* [Hand tap](https://www.amazon.com/dp/B0736T5NM6)
* [Cutting oil](https://www.amazon.com/dp/B003X3ZKXI)
* [Timing belt (1) and timing pulleys (2)](https://www.amazon.com/dp/B07XG9JN5B)
* [Idler pulleys (6)](https://www.amazon.com/dp/B0BSPC7D9S)
* [Limit switches (4)](https://www.amazon.com/dp/B073TYWX86)
* [Joystick (1)](https://www.amazon.com/Joystick-Module-Arduino-ESP8266-Raspberry/dp/B0DQ37P5RQ/)
* [Resistors](https://www.amazon.com/Resistor-Resistors-Assortment-Breadboard-Electronics/dp/B0F4P352BB/)

# Print the Parts

See [here](https://github.com/MatthewGerber/3d-printing) for the 3d-printing tech that I use. The part models for the 
gantry are available [here](https://www.printables.com/model/1390344-h-style-two-axis-gantry). The assembly also uses
the cable drag chain [here](https://www.printables.com/model/1401355-cable-drag-chain) to keep wires from tangling under 
the gantry when it moves. Print two cable drag chains.

# Assemble the Gantry

1. Tap M4 holes in each end of the three rails using the hand tap and tapping oil.
2. Roll the v-wheel plates onto the side rails.
3. Bolt the end caps onto the side rails using M4 bolts.
4. Screw the magnet stand onto the center v-wheel plate using an M2 bolt and the magnet stand washer.
5. Roll the magnet stand v-wheel plate onto the cross rail.
6. Bolt the end caps onto the cross rail using M4 bolts.
7. Bolt the cross-rail end caps onto the side rail v-wheel plates.
8. Bolt timing pulleys to the stepper motors.
9. Bolt the stepper motors to their mounts.
10. Place idler pulleys and bolt them into place using M2 bolts and idler pulley washers.

# Limit Switches

# Cable Drag Chains

# Circuit