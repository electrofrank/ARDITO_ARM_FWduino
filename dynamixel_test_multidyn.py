import argparse
import numpy
import time
import sys

import pypot.dynamixel

import paho.mqtt.client as mqtt #import the client1

from queue import Queue

q1=Queue()
q2=Queue()
q3=Queue()

messages=[]

speed1 = 0
speed2 = 0
speed3 = 0

def on_connect(client1, userdata, flags, rc):
    client1.connected_flag=True
    #messages.append(m)
    #print(m)

def on_message(client1, userdata, message):
    print("received ") #put messages on queue


def DYN1_callback(client, userdata,message):
    global speed1
    speed1 = str(message.payload.decode("utf-8"))
    #q1.put(speed1) #put messages on queue

def DYN2_callback(client, userdata,message):
    global speed2
    print("message for dyn 2")
    speed2 = str(message.payload.decode("utf-8"))
    q2.put(speed2) #put messages on queue

def DYN3_callback(client, userdata,message):
    global speed3
    speed3 = str(message.payload.decode("utf-8"))
    q3.put(speed3) #put messages on queue


def on_publish (client, userdata, mid):
    global messages
    m="on publish callback mid "+str(mid)
    #messages.append(m)

def on_subscribe(client, userdata, mid, granted_qos):
    m="on_subscribe callback mid "+str(mid)


def read_register(dxl, register, ids):
    print('\tReading {} times the {}...'.format(N, register))

    t = []
    getter = getattr(dxl, 'get_{}'.format(register))

    for _ in range(N):
        start = time.time()
        getter(ids)
        end = time.time()
        t.append(1000 * (end - start))

    print('\tTook {}ms (STD={}) per read'.format(numpy.mean(t), numpy.std(t)))


def full_test(dxl, ids):
    print('Testing the communication speed with motor{} {}'.format('s' if len(ids) else '',
                                                                   ids))
    read_register(dxl, 'present_position', ids)
    read_register(dxl, 'control_table', ids)


broker_address="m16.cloudmqtt.com"
#broker_address="iot.eclipse.org"
client1 = mqtt.Client("P1")    #create new instance
client1.username_pw_set(username="vaswmufn",password="cyai9JDN7VYU")

client1.on_connect= on_connect        #attach function to callback
client1.on_message=on_message        #attach function to callback
client1.on_publish =on_publish 

       #attach function to callback
client1.message_callback_add("ARM/DYN/1", DYN1_callback)
client1.message_callback_add("ARM/DYN/2", DYN2_callback)
client1.message_callback_add("ARM/DYN/3", DYN3_callback)



#client1.on_subscribe =on_subscribe        #attach function to callback
time.sleep(1)
print("connecting to broker")
client1.connected_flag=False
client1.connect(broker_address, port=18790 ) #connect to broker

print("connecting dynamixel")
ports = pypot.dynamixel.get_available_ports()
if not ports:
    raise IOError('no port found!')

print('ports found', ports)

dxl =  pypot.dynamixel.DxlIO("COM10",baudrate=57600)
print('Ok!')



print("starting the loop")

client1.loop_start()    #start the loop

r=client1.subscribe("ARM/DYN/1")
r=client1.subscribe("ARM/DYN/2")
r=client1.subscribe("ARM/DYN/3")

counter = 0

speed = 0

while not client1.connected_flag:
    print("waiting for connect")
    time.sleep(0.5)

while 1:
        
        if float(speed1) < -0.5 or float(speed1) > 0.5:
            val1 = float(speed1)*50.0
        else:
            val1 = 0
        
        if float(speed2) < -0.2 or float(speed2) > 0.2:
            val2 = float(speed2)*100.0
        else:
            val2 = 0

        if float(speed3) < -0.2 or float(speed3) > 0.2:
            val3 = float(speed3)*100.0
        else:
            val3 = 0


        dxl.set_moving_speed({1:-val3})   #giunto1
        dxl.set_moving_speed({2:-val1})  #giunto2
        dxl.set_moving_speed({3:-val2})  #giunto3


        time.sleep(0.01)
        

client.disconnect()
clinet.loop_stop()
