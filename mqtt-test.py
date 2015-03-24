#!/usr/bin/python


import sys
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import time

#Setup the bridge to the Arduino processor
sys.path.insert(0, '/usr/lib/python2.7/bridge')

from time import sleep
from bridgeclient import BridgeClient as bridgeclient

#Variable definitions
tempf = "tempF"
targetf = "targetF"
setf = "setF"

bridge = bridgeclient()

#When connected to the mqtt server
def on_connect(mqttc, obj, flags, rc):
    print("rc: "+str(rc))
    
#When a message is received from the mqtt server
def on_message(mqttc, obj, msg):
    msg = str(msg.payload)
    
    message = msg.split("=")
    if message[0] == tempf:
        print "Temperature in F = " + message[1]
    elif message[0] == targetf:
        print "Target temperature is = " + message[1]
    elif message[0] == setf:
        bridge.put("setTemp", message[1])
    
    #print(msg.topic+" "+str(msg.qos)+" "+str(msg.payload))
        
#When you publish to the MQTT server
def on_publish(mqttc, obj, mid):
    print("mid: "+str(mid))
    #print("message: " + str(obj))

#When you subscribe to a topic on the MQTT server            
def on_subscribe(mqttc, obj, mid, granted_qos):
    print("Subscribed: "+str(mid)+" "+str(granted_qos))
                
#When you need to log something from the MQTT server
def on_log(mqttc, obj, level, string):
    print(string)
                    
# If you want to use a specific client id, use
# mqttc = mqtt.Client("client-id")
# but note that the client id must be unique on the broker. Leaving the client
# id parameter empty will generate a random id for you.
mqttc = mqtt.Client()
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_subscribe = on_subscribe
# Uncomment to enable debug messages
#mqttc.on_log = on_log
mqttc.connect("smoker-relay.us", 1883, 60)
mqttc.subscribe("smoker", 0)

#Loop on a background thread
mqttc.loop_start()                    

while True:
    try:
        mqttc.publish("smoker", tempf + "=" + str(bridge.get('degF')))
        targetTemp = str(bridge.get('targetF'))
        mqttc.publish("smoker", targetf + "=" + targetTemp)
    except:
        print "Error Will Robinson"
        
    time.sleep(5) #sleep for 10 seconds  
