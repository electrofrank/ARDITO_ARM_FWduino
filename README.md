# ARDITO_ARM_FWduino
**Firmware running on Atmega2560 Arm controller**

**WHAT THIS FIRMWARE DO**

- connect arm controller at the network via ethernet shield using MQTT protocol. The actual IP of the server (jetson nano) is 10.1.0.12

- subscribe to different topic where the speed target for the arm joints are received 

- the Accellstepper library allow then to control the stepper with this velocity setpoint.

- the topic are published by a python script running on the pc of the ARM pilot (ARM_joystick_to_mqtt_nodisplay.py)

- another script has to run on the jetson (dynamixel_test_multidyn). this is for the control of the dynamixel of the wrist. still not tested


**TO DO**

- implement IK function for Autonomus Operation

- solve the dinamixel issue and connect them to the arm controller board. 

