import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

class distanceCalculator(Node):
    def __init__(self):
        super().__init__('distance_calculator')
        self.publisher_ = self.create_publisher(String, 'distance', 10)

    def run(self):
        GPIO.setmode(GPIO.BCM)

        PIN_TRIGGER = 22
        PIN_ECHO = 27

        GPIO.setup(PIN_TRIGGER, GPIO.OUT)
        GPIO.setup(PIN_ECHO, GPIO.IN)

        while True:
            GPIO.output(PIN_TRIGGER, GPIO.LOW)
            GPIO.output(PIN_TRIGGER, GPIO.HIGH)
            time.sleep(0.1)
            GPIO.output(PIN_TRIGGER, GPIO.LOW)
            time.sleep(0.00001) 
            
            while GPIO.input(PIN_ECHO) == 0:
                pulse_start_time = time.time()
            while GPIO.input(PIN_ECHO) == 1:
                pulse_end_time = time.time()
                
            pulse_duration = pulse_end_time - pulse_start_time
            distance = round(pulse_duration * 17150,2)
            msg = String()
            msg.data = '%d' % distance
            self.publisher_.publish(msg)
            self.get_logger().info(msg.data)
        
        GPIO.cleanup()
        
def main(args=None):
    rclpy.init(args=args)
    distance_calculator = distanceCalculator()
    
    distance_calculator.run()
    
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
