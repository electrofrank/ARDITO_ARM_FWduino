import pygame
import paho.mqtt.client as mqtt #import the client1

en_flag = 0
flag = 0

broker_address="10.1.0.101" #use external broker
#rok#broker_address="10.1.0.101" #use external broker
#broker_address="127.0.0.1" #use external broker

client = mqtt.Client("ARM JOYSTICK") #create new instance

#client.username_pw_set(username="vaswmufn",password="cyai9JDN7VYU")

client.connect(broker_address, port=1883 ) #connect to broker
 

pygame.init()

clock = pygame.time.Clock()

# Initialize the joysticks
pygame.joystick.init()
#initialize the keyboard
 
joystick = pygame.joystick.Joystick(0)
joystick.init()

# -------- Main Program Loop -----------
while True:
    # EVENT PROCESSING STEP
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
 
        # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN
        # JOYBUTTONUP JOYHATMOTION
        if event.type == pygame.JOYBUTTONDOWN:
            if joystick.get_button(6):
                if en_flag == 0:
                    print("ARM ENABLED")
                    en_flag = 1
                else:
                    en_flag = 0
                    print("ARM DISABLED")

            if joystick.get_button(5):
                if flag == 0:
                    print("WRIST COMMAND")
                    flag = 1
                else:
                    flag = 0
                    print("ARM COMMAND")






        

    

    if (flag == 0):
    #client.publish("ARM/DYN/1",joystick.get_axis(0))
    #client.publish("ARM/DYN/2",joystick.get_axis(1))
    #client.publish("ARM/DYN/3",joystick.get_axis(2))
        #print("arm command")
        string_arm = "%f,%f,%f" %( joystick.get_axis(0),joystick.get_axis(1),joystick.get_axis(2))
        client.publish("ARM/JOINTS",string_arm)
        string_wrist = "0,0,0"
        client.publish("WRIST/JOINTS",string_wrist);


    if (flag == 1):
        #print("wrist command")
        string_arm = "0,0,0"
        client.publish("ARM/JOINTS",string_arm)
        string_wrist = "%f,%f,%f" %( joystick.get_axis(0),joystick.get_axis(1),joystick.get_axis(2))
        client.publish("WRIST/JOINTS",string_wrist);


    if (en_flag == 1):
        client.publish("ARM/EN",1)
   
    if (en_flag == 0):
        client.publish("ARM/EN",0)




 
    # Limit to 60 frames per second
    clock.tick(120)
 
# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit()
