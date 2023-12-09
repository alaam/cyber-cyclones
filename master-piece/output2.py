#from _typeshed import Self
from hub import light_matrix, motion_sensor
import runloop, sys
import motor_pair
import motor
import color_sensor
import math
import time
from hub import port


class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def calculate(self, error):
        self.integral += error
        derivative = error - self.prev_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

class FLLBaseLib:
    def __init__(self):
        self.lmotor_p=port.C
        self.rmotor_p=port.B
        self.lhmotor_p=port.E
        self.rhmotor_p=port.F
        self.lcolor_p=port.A
        self.rcolor_p=port.D
        self.cur_turn_deg=0
        motor_pair.pair(motor_pair.PAIR_1,self.lmotor_p,self.rmotor_p)
        motion_sensor.set_yaw_face(motion_sensor.FRONT)
        motion_sensor.reset_yaw(0)



    # cm, this is a constant for your robot
    WHEEL_CIRCUMFERENCE = 17.6
    TRACK = 8 #11.2 # cm - please measure your own robot.


    def follow_line(self,time_to_follow_in_ms):
        # Initialize PID controller with appropriate constants
        pid_controller = PIDController(Kp=0.1, Ki=0.01, Kd=0.05)

        # Set a base motor speed (adjust based on your requirements)
        base_speed = 30
        start_time=time.ticks_ms()
        # Example line-following loop
        while time.ticks_ms()-start_time < time_to_follow_in_ms:
            sensor_value = color_sensor.reflection(self.lcolor_p)
            steering=math.floor(3/5)*sensor_value+base_speed
            # Setpoint is the center sensor value when the robot is perfectly on the line
            setpoint = 31

            # Calculate the error
            error = setpoint - sensor_value
            # Use PID controller to get the correction value
            pid_output = pid_controller.calculate(error)

            #print("steering:",steering)
            #print("time_elapsed:",time.ticks_ms()-start_time," given:",time_to_follow_in_ms)
            motor_pair.move(motor_pair.PAIR_1,math.floor(pid_output),velocity=100)
            #motor.run(self.rmotor_p,math.floor(base_speed-pid_output),acceleration=100)
            #motor.run(self.lmotor_p,math.floor(base_speed+pid_output),acceleration=100)
        motor_pair.stop(motor_pair.PAIR_1)
        motor.stop(self.lmotor_p)
        motor.stop(self.rmotor_p)
        print("follow line done,", time.ticks_ms()-start_time)


    # input must be in the same unit as WHEEL_CIRCUMFERENCE
    def deg_for_dist(self,distance_cm):
        # Add multiplier for gear ratio if needed
        return int((distance_cm/FLLBaseLib.WHEEL_CIRCUMFERENCE) * 360)




    async def move(self,distance_in_cm,velo=360,steer=0):
        distance_in_deg=self.deg_for_dist(distance_in_cm)
        await motor_pair.move_for_degrees(motor_pair.PAIR_1,distance_in_deg,steer,velocity=velo,stop=motor.BRAKE, acceleration=500)

    async def lefthand_move(self,distance_in_deg=0,velo=360,direction=motor.COUNTERCLOCKWISE):
        #distance_in_deg=self.deg_for_dist(distance_in_cm)
        #await motor.run_for_degrees(self.lhmotor_p, distance_in_deg,velo)
        await motor.run_to_absolute_position(self.lhmotor_p,distance_in_deg,velo,direction=direction)

    async def lefthand_move_relative(self,distance_in_deg=0,velo=360,direction=motor.COUNTERCLOCKWISE):
        #distance_in_deg=self.deg_for_dist(distance_in_cm)
        #await motor.run_for_degrees(self.lhmotor_p, distance_in_deg,velo)
        motor.reset_relative_position(self.lhmotor_p, position=0)
        await motor.run_to_relative_position(self.lhmotor_p,distance_in_deg,velo)

    async def lefthand_moveup(self,distance_in_deg=130,velo=360):
        await self.lefthand_move(distance_in_deg,velo,direction=motor.CLOCKWISE)

    async def lefthand_movedown(self,distance_in_deg=100,velo=360):
        await self.lefthand_move(-distance_in_deg,velo)

    async def lefthand_moveup_relative(self,distance_in_deg=130,velo=360):
        await self.lefthand_move_relative(-1 * distance_in_deg,velo)

    async def lefthand_movedown_relative(self,distance_in_deg=130,velo=360):
        await self.lefthand_move_relative(distance_in_deg,velo)

    async def righthand_move(self,distance_in_deg=0,velo=360,direction=motor.COUNTERCLOCKWISE):
        await motor.run_to_absolute_position(self.rhmotor_p,distance_in_deg,velo,direction=direction)

    async def righthand_moveup(self,distance_in_deg=130,velo=360):
        await self.righthand_move(distance_in_deg,velo,direction=motor.CLOCKWISE)

    async def righthand_movedown(self,distance_in_deg=100,velo=360):
        await self.righthand_move(-distance_in_deg,velo)



    async def reset_yaw(self):
        motion_sensor.reset_yaw(0)
        await runloop.until(motion_sensor.stable)

    # Function that returns true when the absolute yaw angle is 90 degrees
    def turn_done(self)->bool:
        #convert tuple decidegree into same format as in app and blocks
        return abs(motion_sensor.tilt_angles()[0] * 0.1) > abs(self.cur_turn_deg)


    async def turn_using_gyro(self,deg):
        self.cur_turn_deg=deg
        #await runloop.sleep_ms(1500)
        motion_sensor.set_yaw_face(motion_sensor.FRONT)
        motion_sensor.reset_yaw(0)
        #runloop.sleep_ms(200)

        #await runloop.until(motion_sensor.stable)
        if deg<0:
            turn=-100
        else:
            turn=100
        print("yaw:",motion_sensor.tilt_angles()[0])
        motor_pair.move(motor_pair.PAIR_1,turn,velocity=100,acceleration=100)
        #await runloop.until(self.turn_done)

        while abs(motion_sensor.tilt_angles()[0])<abs(deg*10): #yaw is decidegree hence multiply incoming angle by 10
            runloop.sleep_ms(100)
        print("yaw:",motion_sensor.tilt_angles()[0])
        #runloop.sleep_ms(3000)
        #print("after 3sec yaw:",motion_sensor.tilt_angles()[0])
        motor_pair.stop(motor_pair.PAIR_1,stop=motor.COAST)
        #motor_pair.move_tank_for_degrees(motor_pair.PAIR_1,deg,0,360)


    async def turn(self,deg):
        motion_sensor.set_yaw_face(motion_sensor.FRONT)
        motion_sensor.reset_yaw(0)
        print("start yaw:",motion_sensor.tilt_angles()[0])
        if deg<0:
            await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1,deg,0,100,acceleration=100)
        else:
            await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1,deg,100,0,acceleration=100)
        await runloop.sleep_ms(3000)
        print("end yaw:",motion_sensor.tilt_angles()[0])


    async def spin_turn(self,degrees, mspeed=200):
        motion_sensor.set_yaw_face(motion_sensor.FRONT)
        motion_sensor.reset_yaw(0)
        print("start yaw:",motion_sensor.tilt_angles()[0])


        SPIN_CIRCUMFERENCE = FLLBaseLib.TRACK * math.pi
        # Add a multiplier for gear ratios if you’re using gears
        mot_degrees = int((SPIN_CIRCUMFERENCE/FLLBaseLib.WHEEL_CIRCUMFERENCE) * abs(degrees))
        if degrees > 0:
            # spin clockwise
            await motor_pair.move_for_degrees(motor_pair.PAIR_1, mot_degrees, 100, velocity=mspeed)
        else:
            #spin counter clockwise
            await motor_pair.move_for_degrees(motor_pair.PAIR_1, mot_degrees, -100, velocity=mspeed)
        #await runloop.sleep_ms(3000)
        print("end yaw:",motion_sensor.tilt_angles()[0])


    async def pivot_turn(self, robot_degrees, motor_speed):
        PIVOT_CIRCUMFERENCE = 2 *FLLBaseLib.TRACK * math.pi
        # Add a multiplier for gear ratios if you’re using gears
        motor_degrees = int((PIVOT_CIRCUMFERENCE/ FLLBaseLib.WHEEL_CIRCUMFERENCE) * abs(robot_degrees))
        if robot_degrees > 0:
            # pivot clockwise
            await motor_pair.move_for_degrees(motor_pair.PAIR_1, motor_degrees, 50, velocity=motor_speed)
        else:
            #pivot counter clockwise
            await motor_pair.move_for_degrees(motor_pair.PAIR_1, motor_degrees, -50, velocity=motor_speed)


    async def move_backward(self,distance_in_cm,velo=360,steer=0):
        await self.move(-distance_in_cm,velo=velo,steer=steer)

    async def move_forward(self,distance_in_cm,velo=360,steer=0):
        await self.move(distance_in_cm,velo=velo,steer=steer)


    async def turn_right(self,deg,speed=200):
        await self.spin_turn(deg,speed)
        #await self.pivot_turn(deg,speed)


    async def turn_left(self,deg,speed=200):
        await self.spin_turn(-deg,speed)
        #await self.pivot_turn(-deg,speed)


class Cyber_Cylones(FLLBaseLib):
    def test(self):
        pass
        
    ##### Mission 2 Theater scence change  ##############
    ##### Alignement - From Right second block ######
    ##### Run time for mission is 15 seconds ##########
    async def mission2(self):
        
        #Initialisation to start the mission
        motion_sensor.reset_yaw(0)
        await self.move_forward(68,360)
        await self.turn_left(25,150)
        await self.move_forward(10,360)
        await runloop.sleep_ms(50)
        await self.move_backward(15,360)
        await self.move_forward(25,360)
        await runloop.sleep_ms(50)
        await self.move_backward(10,360)
        await self.turn_right(60,190)
        await self.move_backward(80,360)
        


cc_robot = Cyber_Cylones()
async def main():
    # # write your code here
    # light_matrix.write("Hi!")
    # cc_robot=CyberCylone()
    # #await ls_robot.follow_line(10000)
    # print("calling move reverse")
    # await cc_robot.move_backward(20)
    # if True:
    #    print("calling move fwd")
    #    await cc_robot.move_forward(20)
    #    #await runloop.sleep_ms(100)
    #    print("calling move turn left")
    #    await cc_robot.turn_left(90)
    #    #await runloop.sleep_ms(2000)
    #    print("calling move turn right")
    #    await cc_robot.turn_right(90)
    #    print("calling move end of main")
    
    await cc_robot.mission2()
    


#main()
runloop.run(main())


