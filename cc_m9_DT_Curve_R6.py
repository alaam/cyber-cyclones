#Copyright 2022 TIA Cyber Cyclones
#Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
#and associated documentation files (the "Software"), to deal in the Software without restriction, 
#including without limitation the rights to use, copy, modify, merge, publish, distribute, 
#sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
#furnished to do so, subject to the following conditions:
#The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
#BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
#IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
#WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE 
#OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from spike.operator import equal_to,less_than
from math import *
hub = PrimeHub()

#hub.light_matrix.show_image('HAPPY')
class Robot:
    def __init__(self):
        global hub
        self.motor_pair = MotorPair("C", "B")
        self.motor = Motor("C")
        self.color = ColorSensor("A")
        #self.motion_sensor =
        self.motor.set_degrees_counted(0)
        self.motor_pair.set_default_speed(20)
        self.hub=hub

class Missions:
    def __init__(self,robot):
        self.robot=robot
        self.hub=self.robot.hub
        self.motor=self.robot.motor
        self.motor_pair=self.robot.motor_pair
        self.color=self.robot.color


    def is_yaw_angle_le(self):
        print("yaw angle:",self.hub.motion_sensor.get_yaw_angle()," requested angle:",self.requested_angle)
        return False # (abs(self.hub.motion_sensor.get_yaw_angle()) != abs(self.requested_angle))

    def turn_using_gyro_wu(self,angle=0,turn_speed=10):
        self.hub.motion_sensor.reset_yaw_angle()
        print("start angle:",self.hub.motion_sensor.get_yaw_angle()," requested angle:",angle)
        if angle<0:
            self.motor_pair.start_tank(0, turn_speed)
        else:
            self.motor_pair.start_tank(turn_speed, 0)
        self.requested_angle=angle
        #wait_until(lamda an=angle, hb=hub: (abs(hb.motion_sensor.get_yaw_angle()) < abs(an)))
        wait_until(self.is_yaw_angle_le)
        self.motor_pair.stop()
        print("current angle:",hub.motion_sensor.get_yaw_angle())

    def turn_using_gyro(self,angle=0,turn_speed=10):
        MAX_NAC_ERR_COUNT=300
        MAX_ANGLE_LIMIT=170
        
        if abs(angle)>MAX_ANGLE_LIMIT:
            angle=MAX_ANGLE_LIMIT* -1 if angle < 0 else 1
        self.hub.motion_sensor.reset_yaw_angle()
        print("start angle:",self.hub.motion_sensor.get_yaw_angle()," requested angle:",angle)

        prev_ya=self.hub.motion_sensor.get_yaw_angle()
        no_angle_change_err_count=0

        if angle<0:
            print("Moving left")
            self.motor_pair.start_tank(0, turn_speed)
        else:
            print("Moving Right")
            self.motor_pair.start_tank(turn_speed, 0)
        cont=True
        while cont:
            cya=self.hub.motion_sensor.get_yaw_angle()
            #print("current yaw angle:",self.hub.motion_sensor.get_yaw_angle()," requested angle:",angle)
            if abs(cya) < abs(angle):
                pass
            else:
                cont=False
            if (prev_ya == cya):
                no_angle_change_err_count+=1
                if no_angle_change_err_count > MAX_NAC_ERR_COUNT:
                    cont=True #False
            else:
                #print("change detected in angle:",self.hub.motion_sensor.get_yaw_angle(),"nacec:",no_angle_change_err_count)
                prev_ya=cya

        self.motor_pair.stop()
        print("end angle:",self.hub.motion_sensor.get_yaw_angle(),"nacec:",no_angle_change_err_count)

    def init_align(self,correction=True):
        #reversing to correct the alignment by hitting the back wall
        if correction:
            self.motor_pair.move(-3,"cm",0)

    def goto_mission_zone(self):
        self.init_align()

        #move fwd with 0.9 degree right turn so it corrects left side alignment
        self.motor_pair.move(10,"cm",1,60)
        #move fwd with 58.8 degree right turn so it head towards the blackline to follow
        self.motor_pair.move(5,"cm",66)
        #move fwd stright to arrive on blackline
        self.motor_pair.move(25,"cm",0,60)

    def line_follow(self,distance):
        while self.motor.get_degrees_counted() >= distance: #-1550: #-1260:
        # steer value is the difference between
        # the reflected light and the reflected light value
        # at the edge
            print(self.motor.get_degrees_counted())
            reflected_light_white = 99
            reflected_light_black = 36
            multiplier = 1.1 #1.2
            #avg color value of the edge of the black-white line
            value_at_edge = (reflected_light_white+reflected_light_black)/2
            #compute the correction angle based on the current color sensor value and reference edge color value
            steer_value = round((value_at_edge - color.get_reflected_light()) * multiplier)
            self.motor_pair.start(steer_value)
        self.motor_pair.stop()

    def turn_using_distance(self,angle,dist):        
        dirction=-1 if angle <0 else 1
        aangle=abs(angle)
        while True:
            self.motor_pair.move(dist,"cm",aangle*dirction)
            #self.motor_pair.start(aangle*dirction)
            aangle-=100
            if aangle < 0: break

    def turn(self,angle,dist,use_gyro=False,turn_speed=10):
        if use_gyro:
            self.turn_using_gyro(angle,turn_speed)
        else:
            self.turn_using_distance(angle, dist)

    def M9_DT(self,straight=False):        
        print("M9_DT:Starting position:R8 Points: 30 = DT 30")
        self.init_align(correction=False)
        #moving base2 to base1
        if straight: #starting point right base block #8
            self.motor_pair.move(175,"cm", 0, 100)
        else: #starting point right base block #6
            self.motor_pair.move(70,"cm", 5, 100)
            self.motor_pair.move(105,"cm", -7, 100)
        #end of function
        self.motor_pair.stop()

    def M1_IPM_M9_DT(self):
        print("M1_IPM_M9_DT: Starting position:R6 Points: 40 = IP 10 + DT 30")
        self.init_align(correction=False)

        #moving from base2 to Hydrogen Plant to drop Innovation project model
        self.motor_pair.move(90,"cm", 8, 100)
        #reversing little back so we can turn towards left base
        self.motor_pair.move(-20,"cm", 0, 100)
        #turning so we can point left base
        self.turn(-33, 7, use_gyro=True)
        #moving from hydrogen plant to left base
        self.motor_pair.move(95,"cm", -3, 100)

        #end of function
        self.motor_pair.stop()

#no new code below this line
def main():
    print("Welcome to Cyber Cyclone World!")
    robot=Robot()
    missions=Missions(robot)

    #print("Moving left")
    #missions.turn(-150,7,use_gyro=True,turn_speed=20)
    #print("Moving right")
    #missions.turn(145,7,use_gyro=True,turn_speed=20)

    #missions.M1_IPM_M9_DT()
    missions.M9_DT(straight=False)

    #missions.turn(-150,10)
    #missions.turn(150,10)


main()
#end of main
