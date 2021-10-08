# ARDITO_ARM_FWduino
**Firmware running on Atmega2560 Arm controller**

**WHAT THIS FIRMWARE DO**

- connect arm controller at the network via ethernet shield using MQTT protocol. The actual IP of the server (jetson nano) is 10.1.0.12

- subscribe to different topic where the speed target for the arm joints and wrist dynamixel are received 

- the Accellstepper library allow then to control the stepper with this velocity setpoint.
- the DynamixelArduino library manage serial protocol for the dynamixel. 

- the topic are published by a python script running on the pc of the ARM pilot (ARM_joystick_to_mqtt_nodisplay.py)


**TO DO**

- implement IK function for cartesian space operation.



**HOW TO USE**

- upload ARM_FWduino on the ARM controller. Check if it connect correclty to the broker.

- Install python 3.6 or higher
- Install pygame library (pip install pygame)
- install paho mqtt library (pip install paho-mqtt)
- Connect the joystick
- Make sure the mqtt broker is up and running
- Execute ARM_joystick_to_mqtt_nodisplay.py. Now the arm topic should appear (check with mqtt explorer or other)
- if the ARM controller is connected to the MQTT Broker it should start to receive data.

- move the joystick for the ARM joint (base, shoulder , elbow)
- press the trigger and move the joystick for WRIST dinamixel.
- use the slider to control the end effector gripper.
