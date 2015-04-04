#!/usr/bin/python


import sys
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import time

#Setup the bridge to the Arduino processor
sys.path.insert(0, '/usr/lib/python2.7/bridge')

from time import sleep
from bridgeclient import BridgeClient as bridgeclient

#Variable definitions to put values in the bridge

#Variable definitions to get values from the bridge

bridge = bridgeclient()

#When connected to the mqtt server
def on_connect(mqttc, obj, flags, rc):
    print("rc: "+str(rc))

    
#When a message is received from the mqtt server, this happens in the background
def on_message(mqttc, obj, msg):
    try:
        msg = str(msg.payload)
    
        message = msg.split("=")
        if message[0] == "runningF":
            print "Temperature in F = " + message[1]
        elif message[0] == "runningTargetF":
            print "Target temperature is = " + message[1]
        elif message[0] == "setTempF":
            bridge.put("setTempF", message[1])
        elif message[0] == "setRunState":
            bridge.put("setRunState", message[1])
        elif message[0] == "startTune":
            bridge.put("tune", message[1])
    except:
    	print("Something wasn't right")
    
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
        #Get the current running variable from the smoker
	runningF = str(bridge.get('runningF'))
        mqttc.publish("smoker", "runningF=" + runningF)

        targetTemp = str(bridge.get('runningTargetF'))
        mqttc.publish("smoker", "runningTargetF=" + targetTemp)

        currentMode = str(bridge.get('runningState'))
        mqttc.publish("smoker", "runningState=" + currentMode)
         
    except:
        print "Error Will Robinson"
        
    time.sleep(5) #sleep for 10 seconds  
