#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import ManualControlSetpoint
import time
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)


# Thresholds — adjust these to tune sensitivity
HIGH_THRESH = 0.8    # stick pushed near full deflection
LOW_THRESH = -0.8    # stick pulled near full opposite
DEAD_ZONE = 0.15     # ignore jitter around center

class RCStickListener(Node):
    def __init__(self):
        super().__init__('rc_stick_listener')

        self.count = 0
        self.next_state = None
        self.prev_swa = None

        # QoS profile compatible with PX4 uXRCE-DDS publications
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(
            ManualControlSetpoint,
            '/fmu/out/manual_control_setpoint',
            self.listener_callback,
            qos,
        )

        # Track previous state so we only print on transitions
        self.prev_state = {
            'throttle': 'center',                    
            'roll': 'center'
        }
        # States: GROUND, MORPHING, AIR
        self.state = 'GROUND'

        # Ground States: IDLE, MOVING_FORWARD, MOVING_BACKWARD, TURNING_RIGHT, TURNING_LEFT        
        self.ground_state = 'IDLE'
        self.get_logger().info('RC Stick Listener started — waiting for stick inputs...')

    def classify(self, value: float) -> str:
        """Classify a stick axis into high / low / center."""
        if value > HIGH_THRESH:
            return 'high'
        elif value < LOW_THRESH:
            return 'low'
        elif abs(value) < DEAD_ZONE:
            return 'center'
        else:
            return 'mid'  # in-between, don't trigger messages

    def listener_callback(self, msg: ManualControlSetpoint):
        # Handle morph countdown — runs every callback (~50Hz)
        # After 150 ticks (~3 seconds), stops motors and exits morphing
        if self.state == 'MORPHING':
            self.count += 1
            if self.count >= 150:
                stop_movement()
                self.count = 0
                self.state = self.next_state
                self.get_logger().info(f'{self.next_state} MODE!')
            return  # Skip stick input while morphing

        # SWA switch — morph control
        swa = 'ON' if msg.aux1 > 0.0 else 'OFF'
        if self.prev_swa is not None and swa != self.prev_swa:
            if swa == 'ON':
                self.get_logger().info('AIR->GROUND Transition!')
                self.state = 'MORPHING'
                self.next_state = 'GROUND'
                self.count = 0
                move_forward()  # testing with move_forward for now

            elif swa == 'OFF':
                self.get_logger().info('GROUND->AIR Transition!')
                self.state = 'MORPHING'
                self.next_state = 'AIR'
                self.count = 0
                move_backward()  # testing with move_backward for now

        self.prev_swa = swa

        #=================================================================================
        #                            LEFT STICK -> MOVEMENT CONTROL
        #=================================================================================
        # Move: front/back on left stick
        if self.state == 'GROUND':  # Only respond to movement commands when on the ground
            throttle_state = self.classify(msg.throttle)
            if throttle_state != self.prev_state['throttle']:
                if throttle_state == 'high':
                    self.get_logger().info(f'Move Forward')
                    if self.ground_state != 'MOVING_FORWARD':
                        self.ground_state = 'MOVING_FORWARD'                 
                        move_forward()
                elif throttle_state == 'low':
                    self.get_logger().info(f'Move Backward')
                    if self.ground_state != 'MOVING_BACKWARD':
                        self.ground_state = 'MOVING_BACKWARD'                
                        move_backward()
                elif throttle_state == 'center':
                    self.get_logger().info(f'STOP')
                    self.ground_state = 'IDLE'              
                    stop_movement()
                self.prev_state['throttle'] = throttle_state
          
            #=================================================================================
            #                            RIGHT STICK -> ROTATION CONTROL
            #=================================================================================
            
    
            # Roll: left/right turn right stick
            roll_state = self.classify(msg.roll)
            if roll_state != self.prev_state['roll']:
                if roll_state == 'high':
                    self.get_logger().info(
                        f'Right turn')
                    if self.ground_state != 'TURNING_RIGHT':
                        self.ground_state = 'TURNING_RIGHT'                   
                        turn_right()
                elif roll_state == 'low':
                    self.get_logger().info(
                        f'Left turn')
                    if self.ground_state != 'TURNING_LEFT':
                        self.ground_state = 'TURNING_LEFT'                  
                        turn_left()
                elif roll_state == 'center':
                    self.get_logger().info(
                        f'STOP turn')
                    self.ground_state = 'IDLE'             
                    stop_movement()
                self.prev_state['roll'] = roll_state
    
def move_forward():
    kit.continuous_servo[6].throttle = 1
    kit.continuous_servo[7].throttle = -1  

def move_backward():
    kit.continuous_servo[6].throttle = -1
    kit.continuous_servo[7].throttle = 1

def stop_movement():     
    kit._pca.channels[6].duty_cycle = 0
    kit._pca.channels[7].duty_cycle = 0


def turn_right():
    kit.continuous_servo[6].throttle = 1  
    kit.continuous_servo[7].throttle = 1   

def turn_left():
    kit.continuous_servo[6].throttle = -1
    kit.continuous_servo[7].throttle = -1

def open_morph():
    kit.continuous_servo[0].throttle = -1
    kit.continuous_servo[1].throttle = -1
    kit.continuous_servo[2].throttle = -1
    kit.continuous_servo[3].throttle = -1

def close_morph():
    kit.continuous_servo[0].throttle = 1
    kit.continuous_servo[1].throttle = 1
    kit.continuous_servo[2].throttle = 1
    kit.continuous_servo[3].throttle = 1

def stop_morph():
    kit._pca.channels[0].duty_cycle = 0
    kit._pca.channels[1].duty_cycle = 5599
    kit._pca.channels[2].duty_cycle = 0
    kit._pca.channels[3].duty_cycle = 0
    kit._pca.channels[1].duty_cycle = 5592

def main(args=None):
    rclpy.init(args=args)
    node = RCStickListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down RC Stick Listener...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()