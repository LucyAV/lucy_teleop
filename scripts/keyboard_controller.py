#!/usr/bin/env python
# Line above declares this document as being a python script

# Imports for ROS, message types and the keyboard module
import rospy
from std_msgs.msg import UInt16
import keyboard

# Control values for the servo
servo_straight = 50
servo_left = 0
servo_right = 100

# Control values for the motor
motor_brake = 0
motor_reverse = 49
motor_idle = 50
motor_normal = 51
motor_shift = 80
motor_space = 100

# Initialize servo and motor current values
servo_current_value = servo_straight
motor_current_value = motor_idle

# Current states of the keys (all initialized as unpressed)
is_forward = False
is_reverse = False
is_left = False
is_right = False
is_brake = False
is_shift = False
is_space = False

def key_change_handler(event):
	global servo_current_value
	global motor_current_value	
	global is_forward
	global is_reverse
	global is_left
	global is_right
	global is_brake
	global is_shift
	global is_space

	# Detect shift for level one boost
	if event.scan_code is 42: # 'shift'
		if event.event_type is keyboard.KEY_DOWN:
			is_shift = True
		else:
			is_shift = False

	# Detect space for level two boost (maximum speed forward)
	if event.scan_code is 57: # 'space'
		if event.event_type is keyboard.KEY_DOWN:
			is_space = True
		else:
			is_space = False

	# Detect motor control keys
	if event.scan_code is 17: # 'w' for minimum speed forward
		if event.event_type is keyboard.KEY_DOWN:
			is_forward = True
		else:
			is_forward = False
	elif event.scan_code is 31: # 's' for braking (or maximum speed backward)
		if event.event_type is keyboard.KEY_DOWN:
			is_brake = True
		else:
			is_brake = False
	elif event.scan_code is 45: # 'x' for minimum speed backward
		if event.event_type is keyboard.KEY_DOWN:
			is_reverse = True
		else:
			is_reverse = False

	# Apply correct motor value based on key states
	if is_forward:
		if is_shift:
			motor_current_value = motor_shift
		elif is_space:
			motor_current_value = motor_space
		else:
			motor_current_value = motor_normal
	elif is_reverse:
		motor_current_value = motor_reverse
	elif is_brake:
		motor_current_value = motor_brake
	else:
		motor_current_value = motor_idle

	# Detect servo control keys
	if event.scan_code is 30: # 'a' for steering left with maximum steering angle
		if event.event_type is keyboard.KEY_DOWN:
			is_left = True
		else:
			is_left = False
	elif event.scan_code is 32: # 'd' for steering right with maximum steering angle
		if event.event_type is keyboard.KEY_DOWN:
			is_right = True
		else:
			is_right = False

	# Apply correct servo value based on key states
	if is_left:
		servo_current_value = servo_left
	elif is_right:
		servo_current_value = servo_right
	else:
		servo_current_value = servo_straight

def publish_data():
	# Initialize the publisher of the motor control data
	publisher = rospy.Publisher('lucy/motor_control', UInt16, queue_size=10)

	# Initialize this node
	rospy.init_node('keyboard_controller', anonymous=True)

	# Create a fixed rate of 10 times per second
	rate = rospy.Rate(10)

	# Publish the current motor and servo control values 10 times per second for
	# as long as the node is running. Both values are included in one UInt16 value.
	while not rospy.is_shutdown():
		publisher.publish( (motor_current_value << 8) | servo_current_value )
		rate.sleep()

if __name__ == "__main__":
	# Assign keyboard listener
	keyboard.hook(key_change_handler)
	try:
		# Call setup method
		publish_data()
	except rospy.ROSInterruptException:
		pass
