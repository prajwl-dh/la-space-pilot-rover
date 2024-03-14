import RPi.GPIO as GPIO
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import message_filters
from adafruit_servokit import ServoKit
from gps import *
import time, inspect
import subprocess
import socket

#make the motor move
# Set up servo motor kit
kit = ServoKit(channels=16)

left_motors = 0
right_motors = 1

# Define motor speeds
stop_speed = 90
move_speed = 97
reverse_speed = 70

class masTer(Node):
    def __init__(self):
        super().__init__('tes_Ter')
        
        # Declare distance_msg and camera_msg as global variables
        global distance_msg, camera_msg
        distance_msg = None
        camera_msg = None
        
        # Create message filters for distance and camera topics
        distance_sub = self.create_subscription(String, 'distance', self.distance_callback, 2)
        camera_sub = self.create_subscription(String, 'camera', self.camera_callback, 1)
        distance_sub
        camera_sub
        
    def distance_callback(self, msg):
        global distance_msg
        distance_msg = msg
        
    def camera_callback(self, msg):
        global camera_msg
        camera_msg = msg

def moveForward():
    kit.servo[left_motors].angle = move_speed
    kit.servo[right_motors].angle = move_speed

def moveBack():
    kit.servo[left_motors].angle = reverse_speed
    kit.servo[right_motors].angle = reverse_speed
    
def moveLeft():
    kit.servo[left_motors].angle = 65
    kit.servo[right_motors].angle = 105
    
def moveRight():
    kit.servo[left_motors].angle = 105
    kit.servo[right_motors].angle = 65
    
def complete():
    kit.servo[left_motors].angle = 140
    kit.servo[right_motors].angle = 40
 
def stop():
    kit.servo[left_motors].angle = stop_speed
    kit.servo[right_motors].angle = stop_speed
    
def moveLeftCamera():
    kit.servo[left_motors].angle = 70
    kit.servo[right_motors].angle = 100
    
def moveRightCamera():
    kit.servo[left_motors].angle = 100
    kit.servo[right_motors].angle = 70

def moveForwardCamera():
    kit.servo[left_motors].angle = 95
    kit.servo[right_motors].angle = 95
    
def captureGPS():
    gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)
    global finalCoordinate
    finalCoordinate = ""
    while True:
        report = gpsd.next() 
        if report['class'] == 'TPV':
            GPStime =  str(getattr(report,'time',''))
            lat = str(getattr(report,'lat',0.0))
            lon = str(getattr(report,'lon',0.0))
            speed =  str(getattr(report,'speed','nan'))
            sats = str(len(gpsd.satellites))
            print(GPStime + '\t' + lat + '\t' + lon + '\t' + speed + '\t' + sats + '\t')
            finalCoordinate = lat + " " + lon
            break
    return finalCoordinate
    
def sendToAppendage(latestCoordinate):
    # Set the IP address and port number of the receiver
    receiver_ip = "192.168.0.101"  # Replace with the IP address of the receiver
    receiver_port = 5000

    # Create a socket object
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Connect to the receiver
    client_socket.connect((receiver_ip, receiver_port))

    # Send a string message to the receiver
    message = latestCoordinate
    client_socket.sendall(message.encode())

    # Close the socket connection
    client_socket.close()

            
            
def main(args=None):
    rclpy.init(args=args)
    mas_Ter = masTer()
    turn_list = ["left", "right"]
    global detected
    detected = 0
    
    # Run infinite loop to receive messages
    while True:
        rclpy.spin_once(mas_Ter, timeout_sec=0)
        
        # Check for new messages and print them
        global distance_msg, camera_msg
        
        if camera_msg is not None:
            stop()
            time.sleep(0.5)
            cameraString = camera_msg.data
            if cameraString == "found":
                detected = 1
                #capture the GPS
                latestCoordinate = captureGPS()
                print(latestCoordinate)
                #send it using socket to appendage rover
                sendToAppendage(latestCoordinate)
                print("Send to appendage rover")
                moveBack()
                time.sleep(0.5)
                complete()
                time.sleep(0.5)
                detected = 0
                camera_msg = None
        
        
        elif (distance_msg is not None and detected == 0):
            distanceString = distance_msg.data
            distanceValue = int(distanceString)
            
            if (distanceValue < 70):
                random.shuffle(turn_list)
                turn = random.choice(turn_list)
                
                if turn == "left":
                    moveLeft()
                    time.sleep(1)
                    distance_msg = None
                   
                elif turn == "right":
                    moveRight()
                    time.sleep(1)
                    distance_msg = None
                
                else:
                    distance_msg = None
                    
            else:
                moveForward()
                distance_msg = None
    
    stop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
