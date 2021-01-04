#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Copyright (c) 2021 Tom Wilson https://tomwilson.com/contact/
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
#---------------------------------------------------------------------------------------------------------
#
# This software is at the prototype stage. It consists of two classes, actuator and robot, which implement 
# n-DOF (degrees of freedom) articulated manipulators. These classes can be used to implement a 
# range of LEGO Mindstorms robots.
#
# The two classes are general, the short main program at the end are specifically for this six-axis 
# plus end effector articulator.
# https://photos.app.goo.gl/Z64PNGZADpMZKLYn9
#
# The main program, at the end, demonstrates how to instantiate a robot, calibrate the robot, 
# then move the actuators to specified angles (thetas) using the robot's go_to method.
#
# Before using this code, it's necessary to perform a kinematics analysis using Denavin-Hartenberg notation. 
# Here's the diagram the author created for this initial code example: 
# https://i1.wp.com/tomwilson.com/wp-content/uploads/2020/12/6-axis-angles-1.jpg?w=763&ssl=1
#
# The following inexpensive course explains some of the basic ideas:
# https://www.udemy.com/course/robotics-1/
#
# Future: The robot go_to method will interface to an inverse kinematics
# library. The code will determine the x, y, z of an object to be picked up, eventually using machine
# vision. Inputs to the kinematics library will include that coordinate, plus other parameters, including 
# desired end effector orientation and (yet-to-be-supplied) limb distances (a's).
#
import rpyc
from ev3dev2.motor import LargeMotor, MediumMotor, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds
from threading import Thread
import sys
from sys import stderr
from pathlib import Path
from smbus import SMBus # pixy2
import pickle
from time import sleep
from ev3dev2.port import LegoPort
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D

class actuator:
    def __init__ (self, motor_object, actuator_name, calibrate_to_origin, \
        gear_ratio, calibration_speed_sp, calibration_stall_duty_cycle_sp, \
        calibrate_hold, normal_speed_p, normal_duty_cycle_sp, ramp_sp,\
        theta_positive_motor_direction, theta_hold,\
        theta_positive_direction, theta_range_of_motion, theta_end_margin, \
        initial_theta, theta_adjustments):
        # theta_positive_motor_direction: direction of motor (normal = motor moves clockwise) 
        #    to move theta in a positive direction
        # theta_positive_direction: positive direction for axis (clockwise or anticlockwise)
        self.motor_object = motor_object
        self.calibrate_hold = calibrate_hold
        self.name = actuator_name
        self.calibrate_to_origin = calibrate_to_origin
        self.gear_ratio = gear_ratio
        self.calibration_speed_sp = calibration_speed_sp
        self.calibration_stall_duty_cycle_sp = calibration_stall_duty_cycle_sp # +ve value
        self.normal_duty_cycle_sp = normal_duty_cycle_sp
        self.normal_speed_p = normal_speed_p
        self.ramp_sp = ramp_sp
        self.theta_positive_motor_direction = theta_positive_motor_direction # normal or inversed
        # direction of motor for theta to increase (bigger positive angle)
        self.theta_positive_direction =  theta_positive_direction # currently clockwise for every joint
        self.theta_hold = theta_hold
        self.theta_range_of_motion = theta_range_of_motion
        # full range of motion degrees, including theta_end_margin at each end
        self.theta_end_margin = theta_end_margin # allow a few degrees at each end of the range of motion
        #   where joint will not go 
        self.initial_theta = initial_theta
        self.theta = 0
        self.theta_adjustments = theta_adjustments
        # amount thetas need to be adjusted when this joint is turned by its motor by 1 degree 
    
    def on_to_theta(self, new_theta, normal_speed_p, normal_duty_cycle_sp):
        if self.theta_in_range(new_theta) and self.theta != new_theta:
            start_theta = self.theta
            new_motor_angle = new_theta/self.gear_ratio
            self.motor_object.duty_cycle_sp = normal_duty_cycle_sp
            self.motor_object.stop_action = self.theta_hold
            self.motor_object.on_to_position(normal_speed_p, new_motor_angle)
            self.theta = new_theta

    def calibrate(self):
        self.motor_object.reset()
        self.motor_object.polarity = self.theta_positive_motor_direction
        if self.calibrate_to_origin: 
            the_calibration_duty_cycle = 0 - self.calibration_stall_duty_cycle_sp
        else:
            the_calibration_duty_cycle = self.calibration_stall_duty_cycle_sp
        
        self.motor_object.ramp_up_sp = self.ramp_sp
        self.motor_object.ramp_down_sp = self.ramp_sp
        self.motor_object.run_direct(duty_cycle_sp = the_calibration_duty_cycle, \
            speed_sp =self.calibration_speed_sp, stop_action = 'brake')
        self.motor_object.wait_until('stalled')  
        self.motor_object.stop() 
        self.motor_object.stop_action = 'brake'
        self.motor_object.speed_p = self.normal_speed_p # percent
        self.motor_object.duty_cycle_sp = self.normal_duty_cycle_sp # degrees

        if self.calibrate_to_origin:
            self.theta = 0 # degrees
            self.motor_object.position = 0 # degrees          
        else: # calibrate to full range of motion
            self.theta = self.theta_range_of_motion # degrees
            self.motor_object.position = self.theta_range_of_motion / self.gear_ratio # degrees
        
        self.on_to_theta(self.initial_theta, self.normal_speed_p, self.normal_duty_cycle_sp)
        self.motor_object.wait_while('running', timeout = 2000)

    def theta_in_range(self, new_theta):
        # verify new_theta will not cause limb to hit end points of range of motion
        return (new_theta > self.theta_end_margin and new_theta < (self.theta_range_of_motion - self.theta_end_margin))

class robot:
    def __init__(self, actuators_parameters):
        # actuators' parameters
            # 0 motor_object
            # 1 actuator_name
            # 2 calibrate_to_origin
            # 3 gear_ratio
            # 4 calibration_speed_sp
            # 5 calibration_stall_duty_cycle_sp
            # 6 calibrate_hold
            # 7 normal_speed_p
            # 8 normal_duty_cycle_sp
            # 9 ramp_sp
            # 10 theta_positive_motor_direction
            # 11 theta_hold
            # 12 theta_positive_direction
            # 13 theta_range_of_motion
            # 14 theta_end_margin
            # 15 initial_theta
            # 16 theta_adjustments
        i = -1
        self.actuator_objects=[]
        for motor_object in actuators_parameters[0]:
            i = i + 1
            self.actuator_objects.append( \
                actuator(motor_object, actuators_parameters[1][i], actuators_parameters[2][i], \
                actuators_parameters[3][i], actuators_parameters[4][i], actuators_parameters[5][i], \
                actuators_parameters[6][i], actuators_parameters[7][i], actuators_parameters[8][i], \
                actuators_parameters[9][i], actuators_parameters[10][i], actuators_parameters[11][i], \
                actuators_parameters[12][i], actuators_parameters[13][i], actuators_parameters[14][i],\
                actuators_parameters[15][i], actuators_parameters[16][i]))   

        #self.read_angles_file()
    
    def go_to(self, new_thetas, speeds, duty_cycles, is_brakes):   
        actuator_threads = []
        theta_corrections= []
        theta_deltas= []
        i=-1
        for actuator_object in self.actuator_objects: # must start at actuator 0 then work towards actuator n
            i=i+1
            actuator_threads.append(None)
            theta_corrections.append(0)
            theta_deltas.append(0)
            if actuator_object != None:
                if new_thetas[i] == None:  
                    new_thetas[i] = actuator_object.theta # new_theta = current theta
                    theta_deltas[i] = 0 
                else:   
                    theta_deltas[i] = new_thetas[i] - actuator_object.theta
            # Check previous actuators for corrections to this actuator. 
            # This piece of code needs additional work to more accurately deal with theta
            # anomalies and motor positions when one joint turns another joint or the motor
            # for another joint
                if i > 0: # not for actuator zero
                    j = i
                    while j > 0:
                        j = j - 1
                        if self.actuator_objects[j] != None:
                            theta_deltas[i] = theta_deltas[i] + \
                                self.actuator_objects[j].theta_adjustments[i] * theta_deltas[j] 
        i = -1
        for actuator_object in self.actuator_objects:
            i = i + 1
            if actuator_object != None:
                new_theta_plus_correction = actuator_object.theta + theta_deltas[i] 
                actuator_threads[i] = Thread(target=self.actuator_objects[i].on_to_theta, \
                        args=(new_theta_plus_correction, speeds[i], duty_cycles[i]))  

        for actuator_thread in actuator_threads:
            if actuator_thread != None:
                actuator_thread.start()

        self.wait_until_not_moving()
        
        i = -1
        for actuator_object in self.actuator_objects:
            i = i + 1
            if actuator_object != None:
                actuator_object.theta = new_thetas[i]
                if actuator_object.motor_object.is_stalled: 
                    print ('***motor stalled ', actuator_object.name, file=stderr)

        # self.save_angles_file()

    def wait_until_not_moving(self):
        i = -1
        for actuator_object in self.actuator_objects:
            i = i + 1
            if actuator_object != None:
                while actuator_object.motor_object.is_running: 
                    sleep(.1)

    def save_settings(self):
        self.save_angles_file()       

    def calibrate(self, actuator_ordinals):
        for actuator_ordinal in actuator_ordinals:
            self.actuator_objects[actuator_ordinal].calibrate()
        #self.save_angles_file()

    def save_angles_file(self):  
        # Currently the program does not use this to save motor angles and thetas to a file
        # Calls to this and other angles files methods are temporarily commented out     
        motor_positions=[]
        thetas=[]
        i = -1
        for actuator_object in self.actuator_objects: 
            i = i + 1
            if actuator_object == None:
                motor_positions.append(None)
                thetas.append(None)
            else:
                motor_positions.append(actuator_object.motor_object.position)
                thetas.append(actuator_object.theta)
            
        #the_file = open("angles.txt", 'w')
        the_dump=[motor_positions, thetas]  

        with open("angles.txt", 'wb') as fp:
            pickle.dump(the_dump, fp)
        fp.close()

    def print_angles(self, comment):
        print('----------', comment, '----------', file=stderr)
        for actuator_object in self.actuator_objects: 
            if actuator_object != None:
                print(actuator_object.name, "Motor Position: ", str(actuator_object.motor_object.position), \
                " theta: ", str(actuator_object.theta), file=stderr)

    def read_angles_file(self):
        the_file = open( "angles.txt", "rb" ) 
        the_dump = pickle.load(the_file)
        motor_positions= the_dump[0]
        actuator_thetas = the_dump[1]
        ii = len(motor_positions) - 1
        i = -1
        while i < ii:
            i = i + 1 
            if self.actuator_objects[i] != None:
                self.actuator_objects[i].motor_object.position=motor_positions[i]
                self.actuator_objects[i].theta=actuator_thetas[i]
        the_file.close()

    def close(self):
        for actuator_object in self.actuator_objects: 
            if actuator_object != None:
                actuator_object.motor_object.stop()
                actuator_object.motor_object.reset()
        sleep(3)

def initialize_angles_file():
    motor_positions=[0, 0, 0, -643, 1900, 535]
    thetas=[0, 0, 0, 170, 128, 176]
    the_dump=[motor_positions, thetas]
    with open("angles.txt", 'wb') as fp:
        pickle.dump(the_dump, fp)
    fp.close()

# Start of main program

#initialize_angles_file()

conn1 = rpyc.classic.connect('192.168.0.1') # ev30
remote_motor_class = conn1.modules['ev3dev2.motor'].LargeMotor

no_theta_adjustments = [0, 0, 0, 0, 0, 0, 0] # adjustments that need to be made to other actuators
# when a given actuator moves. no_theta_adjustments specifies that no adjustments are needed to other actuators.

motor_objects =  [remote_motor_class(OUTPUT_A),remote_motor_class(OUTPUT_B), remote_motor_class(OUTPUT_C), \
        MediumMotor(OUTPUT_A), MediumMotor(OUTPUT_B), MediumMotor(OUTPUT_C), MediumMotor(OUTPUT_D)]
actuator_names = ['waist', 'shoulder', 'upper_arm', 'wrist_rotation', 'wrist_bend', 'flange', 'end_effector']
calibrate_to_origin = [True, False, False, False, False, False, False]
gear_ratios = [1/9, 8/60, 8/60, 12/60, 20/60 * 12/24, 1/3, 1]
calibration_speed_sps = [400, 300, 70, 80, 600, 90, 100]
calibration_stall_duty_cycle_sps = [80, 70, 90, 100, 90, 100, 50]
calibrate_holds = [True, False, False, False, False, False, False]
normal_speed_ps = [40, 40, 50, 100, 80, 90, 50]
normal_duty_cycle_sps = [40, 80, 90, 100, 90, 100, 70]
ramp_sps = [1000, 1000, 1000, 0, 0, 0, 0] # up and down
theta_positive_motor_directions = ['inversed', 'inversed', 'inversed', 'inversed', 'inversed', 'inversed', 'inversed']
theta_holds = ['coast','hold', 'hold', 'coast', 'coast', 'coast', 'coast']
theta_positive_directions = ['counterclockwise', 'counterclockwise', 'counterclockwise', 'counterclockwise', \
        'counterclockwise', 'counterclockwise', 'counterclockwise']
theta_range_of_motions = [180, 120, 90, 300, 240, 200, 80]
theta_end_margins = [5, 5, 5, 5, 5, 5, 0]
initial_thetas = [105, 75, 45, 100, 90, 60, 40]
theta_adjustments = [no_theta_adjustments, no_theta_adjustments, no_theta_adjustments, [0, 0, 0, 0, .1, -1/6, 0], \
        no_theta_adjustments, no_theta_adjustments, no_theta_adjustments] 
        # Actuators 3,4,5 have concentric drives. Actuator 3 has impact on actuators 4 and 5. 
        # The code to compensate for this needs additional work to adjust thetas more correctly

actuators_parameters = [motor_objects, actuator_names, calibrate_to_origin, gear_ratios, \
    calibration_speed_sps, calibration_stall_duty_cycle_sps, calibrate_holds, \
    normal_speed_ps, normal_duty_cycle_sps, ramp_sps, \
    theta_positive_motor_directions, theta_holds, theta_positive_directions, theta_range_of_motions, \
    theta_end_margins, initial_thetas, theta_adjustments]
# Alternatively, the program could read the above data from an external file, such as a JSON file. The file could
# be fronted by a GUI, making it easy to create and modify robots.

my_robot = robot(actuators_parameters) # more than one robot can be created in this program

my_robot.calibrate([1, 2, 0, 4, 3, 4, 5, 6]) # actuators are calibrated by jamming in the specified
# order. Joint 4 is calibrated twice to avoid a collision. 6 operates the end effector, a motorized valce
# to operate a pneumatic grabber.

sleep(5)

my_robot.go_to( \
    [35, 50, 160, 100, 110, 20, 80], \
    [40, 50, 40, 40, 40, 40, 60], \
    [100, 80, 40, 60, 40, 20, 60], \
    [True, True, True, True, True, True, True])
# apart from theta angles, the other parameters should be optional

sleep(5)

my_robot.close()
sys.exit()
