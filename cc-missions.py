from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
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

    def turn_using_gyro(self,angle=0,turn_speed=20):
        hub=self.robot.hub
        motor=self.robot.motor
        motor_pair=self.robot.motor_pair
        
        ca = abs(hub.motion_sensor.get_yaw_angle())
        hub.motion_sensor.reset_yaw_angle()
        # motor_pair.move_tank(1,'cm',0,50)
        
        while (abs(hub.motion_sensor.get_yaw_angle())-ca) < abs(angle):
            # motor_pair.start(50)
            #print(hub.motion_sensor.get_yaw_angle())
            print((abs(hub.motion_sensor.get_yaw_angle())-ca))
            if angle<0:
                motor_pair.start_tank(0, turn_speed)
            else:
                motor_pair.start_tank(turn_speed, 0)
        motor_pair.stop()
        
        pass

    def init_align(self):
        hub=self.robot.hub
        motor=self.robot.motor
        motor_pair=self.robot.motor_pair

        #reversing to correct the alignment by hitting the back wall
        motor_pair.move(-3,"cm",0)

    def goto_mission_zone(self):
        hub=self.robot.hub
        motor=self.robot.motor
        motor_pair=self.robot.motor_pair
        self.init_align()

        #move fwd with 0.9 degree right turn so it corrects left side alignment
        motor_pair.move(10,"cm",1,60)
        #move fwd with 58.8 degree right turn so it head towards the blackline to follow
        motor_pair.move(5,"cm",66)
        #move fwd stright to arrive on blackline
        motor_pair.move(25,"cm",0,60)

    def line_follow(self,distance):
        hub=self.robot.hub
        motor=self.robot.motor
        motor_pair=self.robot.motor_pair
        color=self.robot.color
        while motor.get_degrees_counted() >= distance: #-1550: #-1260:
        # steer value is the difference between
        # the reflected light and the reflected light value
        # at the edge
            print(motor.get_degrees_counted())
            reflected_light_white = 99
            reflected_light_black = 36
            multiplier = 1.1 #1.2
            #avg color value of the edge of the black-white line
            value_at_edge = (reflected_light_white+reflected_light_black)/2
            #compute the correction angle based on the current color sensor value and reference edge color value
            steer_value = round((value_at_edge - color.get_reflected_light()) * multiplier)
            motor_pair.start(steer_value)
        motor_pair.stop()

    def turn_using_distance(self,angle,dist):
        hub=self.robot.hub
        motor=self.robot.motor
        motor_pair=self.robot.motor_pair
        color=self.robot.color
        dirction=-1 if angle <0 else 1
        aangle=abs(angle)
        while True:
            motor_pair.move(dist,"cm",aangle*dirction)
            #motor_pair.start(aangle*dirction)
            aangle-=100
            if aangle < 0: break

    def turn(self,angle,dist,use_gyro=False,turn_speed=20):
        if use_gyro:
            self.turn_using_gyro(angle, turn_speed)
        else:
            self.turn_using_distance(angle, dist)

    def M2_OilPlatform(self):
        hub=self.robot.hub
        motor=self.robot.motor
        motor_pair=self.robot.motor_pair

        self.goto_mission_zone()
        #move fwd stright to arrive just before the target so we can turn right 90 degree to push
        motor_pair.move(47,"cm",0,65)

        #align stright to 0 deg
        motor_pair.move(5,"cm",-85)
        #turn 90 deg left
        motor_pair.move(5,"cm",-100)

        motor_pair.move(5,"cm",0)
        motor_pair.move(5,"cm",0)

    # moving forward and backward so the energy unit goes to the truck
        for i in range (0, 5):
            mfb = 12
            motor_pair.move(mfb,"cm",0)
            motor_pair.move(-mfb,"cm",0)

        motor_pair.move(5,"cm",-100,100)
        motor_pair.move(5,"cm",-100,100)
        motor_pair.move(55,"cm",0,100)
        motor_pair.stop()

    def M3_EnergyStorage(self):
        hub=self.robot.hub
        motor=self.robot.motor
        motor_pair=self.robot.motor_pair
        color=self.robot.color
        self.goto_mission_zone()
        #Proposational line following algorithem
        while motor.get_degrees_counted() >= -1930: #-1260:
        # steer value is the difference between
        # the reflected light and the reflected light value
        # at the edge
            print(motor.get_degrees_counted())
            reflected_light_white = 99
            reflected_light_black = 36
            multiplier = 1 #1.2
            #avg color value of the edge of the black-white line
            value_at_edge = (reflected_light_white+reflected_light_black)/2
            #compute the correction angle based on the current color sensor value and reference edge color value
            steer_value = round((value_at_edge - color.get_reflected_light()) * multiplier)
            motor_pair.start(steer_value)

        #align with the target mission
        motor_pair.move(-1,"cm",-35)
        motor_pair.move(1,"cm",35)
        #bring back the energy units to base
        motor_pair.move(-84,"cm",-9,100)
        motor_pair.stop()


    def M4_SolarFarm(self):
        hub=self.robot.hub
        motor=self.robot.motor
        motor_pair=self.robot.motor_pair
        color=self.robot.color
        self.goto_mission_zone()
        #Proposational line following algorithem
        moving_ditance=1620
        self.line_follow(-moving_ditance)
        #turn right to follow the ling again
        #motor_pair.move(5,"cm",100)
        #motor_pair.move(5,"cm",40)
        self.turn(50,10) #each 100 angle 5cm will move
        moving_ditance+=950
        self.line_follow(-moving_ditance)

        #Turn left towards engergy collection target area
        #motor_pair.move(5,"cm",-100)
        #motor_pair.move(5,"cm",-20)
        self.turn(-80,10) #each 100 angle 5cm will move

        #move forward so it can cover the energy units
        motor_pair.move(10,"cm",0)

        #turn left more so it can take more energy units
        #motor_pair.move(5,"cm",-85)
        self.turn(-55,10) #each 100 angle 5cm will move

        #move forward to collect more energy units
        motor_pair.move(84,"cm")

        #turn left more so it can point towards the base
        #motor_pair.move(5,"cm",-85)
        self.turn(-85,5) #each 100 angle 5cm will move

        #move forward to with the collected units
        motor_pair.move(120,"cm",100)

        motor_pair.stop()

    def M2_OilPlatform_TruckPickup(self):
        hub=self.robot.hub
        motor=self.robot.motor
        motor_pair=self.robot.motor_pair
        color=self.robot.color
        self.init_align()
        motor_pair.move(53,"cm",0,50)
        motor_pair.move(-50,"cm",0,50)

    def M9(self):
        hub=self.robot.hub
        motor=self.robot.motor
        motor_pair=self.robot.motor_pair
        self.init_align()
        #move forward 5 cm so we can turn right
        motor_pair.move(5,"cm",0)

        #turn 65 degree right
        motor_pair.move(15,"cm",50)

        #move forward 75 cm
        motor_pair.move(75,"cm",0,85)
        #turn 20 degree right so it will go in the middle of the target area
        #motor_pair.move(15,"cm",-25)
        #move forward 5 cm to get into target area and drop the innovation block
        #motor_pair.move(2,"cm",0,85)

        #Come back to base 75 cm
        motor_pair.move(-80,"cm",0,85)
        #turn 20 degree right so it will go in the middle of the target area
        #motor_pair.move(15,"cm",40)
        #motor_pair.move(-65,"cm",0,100)

        motor_pair.stop()


    def M08_TVWatch(self):
        hub=self.robot.hub
        motor=self.robot.motor
        motor_pair=self.robot.motor_pair
        color=self.robot.color
        self.init_align()
        #start the mission logic here
        motor_pair.move(56,"cm",0,35) #initial movement towards tv
        motor_pair.move(-15,"cm",52,100) #reverse back to go to windmill
        motor_pair.move(50,"cm",0,50) #go to windmill
        if False:
            motor_pair.move(7,"cm",100,50) # first turn towards windmill
            motor_pair.move(7,"cm",45,50) #second turn **
        else:
            self.turn(-45,7,use_gyro=True,turn_speed=20)
        for i in range (0,4):
            motor_pair.move(38,"cm",0,40)
            motor_pair.move(-20,"cm",0,40)

        motor_pair.move(-7,"cm",100,50)
        motor_pair.move(-7,"cm",43,50)
        #250,251 turning towards hand 253 driving towards hand
        motor_pair.move(52,"cm",-30,50)

        #end of function
        motor_pair.stop()

    def M08_TVWatch2(self):
        hub=self.robot.hub
        motor=self.robot.motor
        motor_pair=self.robot.motor_pair
        color=self.robot.color
        self.init_align()
        #start the mission logic here
        motor_pair.move(56,"cm",0,35) #initial movement towards tv

        #motor_pair.move(-15,"cm",52,100) #reverse back with a turn to go to windmill
        self.turn(-45,7,use_gyro=True,turn_speed=20)

        motor_pair.move(50,"cm",0,50) #go to windmill by moving stright

        self.turn(-65,7,use_gyro=True,turn_speed=20) #turn right towards windmill

        if False:
            for i in range (0,4):
                motor_pair.move(38,"cm",0,40)
                motor_pair.move(-20,"cm",0,40)

            motor_pair.move(-7,"cm",100,50)
            motor_pair.move(-7,"cm",43,50)
            #250,251 turning towards hand 253 driving towards hand
            motor_pair.move(52,"cm",-30,50)

            #end of function
        motor_pair.stop()



    def M07_Windturben(self):
        hub=self.robot.hub
        motor=self.robot.motor
        motor_pair=self.robot.motor_pair
        color=self.robot.color
        self.init_align()
        #start the mission logic here

        #end of function
        motor_pair.stop()


#no new code below this line
def main():
    print("Hello World!")
    robot=Robot()
    missions=Missions(robot)

    #print("Moving left")
    #missions.turn(-45,7,use_gyro=True,turn_speed=20)
    #print("Moving right")
    #missions.turn(45,7,use_gyro=True,turn_speed=20)

    missions.M08_TVWatch2()
    #missions.turn(-150,10)
    #missions.turn(150,10)


main()