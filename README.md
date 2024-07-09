# Vacuum Chamber Development
This is a desktop prototype for a vacuum chamber being developed for the UTS Rocketry Team. 

The aim for this desktop prototype is to develop and test a closed loop controller that is 
able to produce the pressure curve generated by Open Rocket simulations.

Eventually the software electronics developed from this prototype will be integrated onto the
larger scale vacuum chamber based inside the UTS Rocketry Team's workshop, which will be used 
for testing avionics sensors and hardware used onboard high powered rockets.

## Block Diagram
Below is a diagram of the envisioned system setup for the larger, workshop based system:

![image](https://github.com/elenajusto/vacuum_chamber_development/assets/56148816/09236080-10e2-471c-92ee-ac212812bd64)

## Prototype 2.1 - Arduino
Prototype 2.1 is running on the Arduino platform. It has currently demonstrated very basic
altitude holding, being able to maintain a simulated altitude of ~200 meters inside the 
desktop chamber.

<b>Libraries:</b>
- Wire.h
- LiquidCrystal_I2C
- AdafruitBMP280

### Prototype 2.1 - Wiring
![image](https://github.com/elenajusto/vacuum_chamber_development/assets/56148816/a3d6ea6c-3ae8-4845-89b3-ec54038836ca)

### Prototype 2.1 - Hardware Setup
![image](https://github.com/elenajusto/vacuum_chamber_development/assets/56148816/57d55936-a541-4886-86e8-34250e34dbc2)
