from hub import light_matrix, motion_sensor, sound, temperature
import runloop, sys
import motor_pair
import motor
import color_sensor
import math
import time
from hub import port
#from typing import Awaitable

# Record the start time when the program begins
start_time = time.ticks_ms()

def myprint(*args, **kwargs):
    # Calculate the elapsed time since the program started
    elapsed_ms = time.ticks_diff(time.ticks_ms(), start_time)

    # Convert milliseconds to minutes, seconds, and milliseconds
    total_seconds = elapsed_ms // 1000
    millis = elapsed_ms % 1000
    minutes = total_seconds // 60
    seconds = total_seconds % 60

    # Format the timestamp as MM:SS.mmm
    timestamp = '{:02d}:{:02d}.{:03d}'.format(minutes, seconds, millis)

    # Combine all positional arguments into a single string
    message = ' '.join(str(arg) for arg in args)

    # Print the timestamp followed by the original message
    print('{}: {}'.format(timestamp, message))


def myprint_(*args, **kwargs):
    #pass
    print(args,kwargs)

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


def is_stable () -> bool:
    #myprint("accy", motion_sensor.acceleration(True)[0],"accy:", motion_sensor.acceleration(True)[1],"accz:", motion_sensor.acceleration(True)[2])
    return (motion_sensor.acceleration(True)[0] <100)
    #+ motion_sensor.acceleration(True)[1])==0

class FLLBaseLib:
    def __init__(self,cfg=None):
        if cfg==None:
            cfg={
                "left_wheel_port":port.B,
                "right_wheel_port":port.A,
                "first_color_sensor_port":port.F,
                "second_color_sensor_port":port.E,
                "first_arm_port":port.C,
                "second_arm_port":port.D,
                "first_arm_motor_position":"",
                "second_arm_motor_position":""
                }

        self.cfg=cfg
        self.left_wheel_port = cfg["left_wheel_port"] #port.B
        self.right_wheel_port = cfg["right_wheel_port"] #port.A
        self.first_color_sensor_port = cfg["first_color_sensor_port"] #port.F
        self.second_color_sensor_port = cfg["second_color_sensor_port"] #port.E
        self.first_arm_port = cfg["first_arm_port"] #port.D
        self.second_arm_port = cfg["second_arm_port"] #port.C

        self.first_arm_init_deg = motor.absolute_position(self.first_arm_port)
        motor_pair.pair(motor_pair.PAIR_1,self.left_wheel_port,self.right_wheel_port)
        motion_sensor.set_yaw_face(motion_sensor.FRONT)
        ret=self.reset_yaw(0)


    # cm, this is a constant for your robot
    WHEEL_CIRCUMFERENCE = 28 # Big Wheel circumference in cm
    TRACK = 9 #11.2 # Big Wheel track in cm - please measure your own robot.


    def follow_line(self,time_to_follow_in_ms):
        # Initialize PID controller with appropriate constants
        pid_controller = PIDController(Kp=0.1, Ki=0.01, Kd=0.05)


        # Set a base motor speed (adjust based on your requirements)
        base_speed = 30
        start_time=time.ticks_ms()
        # Example line-following loop
        while time.ticks_ms()-start_time < time_to_follow_in_ms:
            sensor_value = color_sensor.reflection(self.first_color_sensor_port)
            steering=math.floor(3/5)*sensor_value+base_speed
            # Setpoint is the center sensor value when the robot is perfectly on the line
            setpoint = 31


            # Calculate the error
            error = setpoint - sensor_value
            # Use PID controller to get the correction value
            pid_output = pid_controller.calculate(error)


            #myprint("steering:",steering)
            #myprint("time_elapsed:",time.ticks_ms()-start_time," given:",time_to_follow_in_ms)
            motor_pair.move(motor_pair.PAIR_1,math.floor(pid_output),velocity=100)
            #motor.run(self.right_wheel_port,math.floor(base_speed-pid_output),acceleration=100)
            #motor.run(self.left_wheel_port,math.floor(base_speed+pid_output),acceleration=100)
        motor_pair.stop(motor_pair.PAIR_1)
        motor.stop(self.left_wheel_port)
        motor.stop(self.right_wheel_port)
        myprint("follow line done,", time.ticks_ms()-start_time)




    # input must be in the same unit as WHEEL_CIRCUMFERENCE
    def deg_for_dist(self,distance_cm):
        # Add multiplier for gear ratio if needed
        return int((distance_cm/FLLBaseLib.WHEEL_CIRCUMFERENCE) * 360)


    async def move(self,distance_in_cm,velo=360,steer=0,correct_error=False):
        distance_in_deg=self.deg_for_dist(distance_in_cm)
        sa=0
        if correct_error:
            await self.reset_yaw(0)
            sa=max(.1,self.get_360_mapped_yaw())

        await motor_pair.move_for_degrees(motor_pair.PAIR_1,distance_in_deg,steer,velocity=velo,stop=motor.SMART_BRAKE, acceleration=1000)

        if correct_error:
            ea=self.get_360_mapped_yaw(stablize_first=True)
            ed=self.compute_error(sa,ea,sa,requested_deg_per_achived=0,hw_err_threshold=150)
            ed = ed * (-1 if distance_in_cm<0 else 1)
            await self.turn_using_gyro(ed,speed=50,correct_error=False)


    async def reset_yaw(self,offset=0):
        #check_stable = lambda : (( motion_sensor.acceleration(True)[0] + motion_sensor.acceleration(True)[1] ) ==0 )
        #await runloop.until(is_stable)
        await runloop.sleep_ms(65)
        motion_sensor.reset_yaw(offset)
        await runloop.sleep_ms(65)
        #await runloop.until(motion_sensor.acceleration)

    def get_360_mapped_yaw(self,stablize_first=False):
        if stablize_first:
            time.sleep(0.065)
        cy=motion_sensor.tilt_angles()[0]
        #if (cy<0): return cy+3600
        return cy/10

    async def turn_using_gyro(self,deg,speed=200,correct_error=False):
        self.cur_turn_deg=deg
        await self.reset_yaw(0)
        turn = -100 if deg<0 else 100

        start_yaw_angle=self.get_360_mapped_yaw(stablize_first=True) #motion_sensor.tilt_angles()[0]
        curr=self.get_360_mapped_yaw() #motion_sensor.tilt_angles()[0]
        diff = abs(curr)-abs(start_yaw_angle)
        myprint("from yaw:",curr, " start_yaw_angle:",start_yaw_angle," diff:",diff, "asked:",deg*10)
        if True:
            motor_pair.move(motor_pair.PAIR_1,turn,velocity=speed)
        else:
            if deg < 0: #left turn
                motor.run(self.left_wheel_port,speed)
            else:
                motor.run(self.right_wheel_port,-speed)

        count=1
        requested_deg_per_achived= 1 if correct_error else 1
        while abs(diff) < abs(deg)*requested_deg_per_achived: #yaw is decidegree hence multiply incoming angle by 10
            #await runloop.sleep_ms(1)
            #myprint("from yaw:",abs(curr), " start_yaw_angle:",abs(start_yaw_angle)," diff:",abs(diff), "asked:",abs(deg*10))
            curr=self.get_360_mapped_yaw() #motion_sensor.tilt_angles()[0]
            diff = abs(curr)-abs(start_yaw_angle)
            count+=1
        motor_pair.stop(motor_pair.PAIR_1,stop=motor.BRAKE)
        #myprint("after loop yaw:",motion_sensor.tilt_angles()[0])
        #await runloop.sleep_ms(200)
        ea=self.get_360_mapped_yaw(stablize_first=True) #motion_sensor.tilt_angles()[0]
        myprint("after 200ms yaw ea:",ea," loop count:",count)
        sa=start_yaw_angle
        if correct_error:
            ed=self.compute_error(sa,ea,deg,requested_deg_per_achived)
            myprint("gyro: degrees:",deg,"ea:",ea," sa:",sa, "error correction:",ed)
            await self.turn_using_gyro(ed,speed=100,correct_error=False)
        myprint("end yaw:",ea)


    async def turn(self,deg):
        motion_sensor.set_yaw_face(motion_sensor.FRONT)
        await self.reset_yaw(0)
        myprint("start yaw:",self.get_360_mapped_yaw())
        if deg<0:
            await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1,deg,0,100,stop=motor.SMART_BRAKE,acceleration=100)
        else:
            await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1,deg,100,0,stop=motor.SMART_BRAKE,acceleration=100)
        myprint("end yaw:",self.get_360_mapped_yaw(stablize_first=True))


    def compute_error(self,gyro_starta,gyro_enda,requested_deg,requested_deg_per_achived,hw_err_threshold=0.3):
        ea=gyro_enda
        sa=gyro_starta
        degrees=requested_deg
        ed=abs(abs(ea)-abs(sa))-abs(degrees)
        if (abs(ed)>abs(degrees)*hw_err_threshold): #only correct 30% of error ; assuming beyond 30% is malfunction of hw but visibly its not
            myprint("compute error: HW mal function: degrees:",degrees,"ea:",ea," sa:",sa, "error correction:",ed)
            if (ed<0):
                ed=-abs(degrees)*(1-requested_deg_per_achived)
            else:
                ed=abs(degrees)*(1-requested_deg_per_achived)
            #ed=0 #not doing error correction if error is more than 30%

        if ed<0: #under run
            ed = ed * (-1 if degrees < 0 else 1)
        else: #over run
            ed = ed * (1 if degrees < 0 else -1) #reverse the direct
        myprint("compute error: degrees:",degrees,"ea:",ea," sa:",sa, "error correction:",ed)
        return ed

    async def spin_turn(self,degrees, mspeed=200,correct_error=False):
        #motion_sensor.set_yaw_face(motion_sensor.FRONT)
        await self.reset_yaw(0)

        sa=self.get_360_mapped_yaw() #(motion_sensor.tilt_angles()[0])/10

        myprint("start yaw:",sa," deg requsted:",degrees)


        SPIN_CIRCUMFERENCE = FLLBaseLib.TRACK * math.pi
        ed=0
#        while ed or degrees:
        # Add a multiplier for gear ratios if you’re using gears
        requested_deg_per_achived= 1 if correct_error else 1
        mot_degrees = int((SPIN_CIRCUMFERENCE/FLLBaseLib.WHEEL_CIRCUMFERENCE) * abs(degrees)*requested_deg_per_achived)
        if degrees > 0:
            # spin clockwise
            await motor_pair.move_for_degrees(motor_pair.PAIR_1, mot_degrees, 100, velocity=mspeed)
        else:
            #spin counter clockwise
            await motor_pair.move_for_degrees(motor_pair.PAIR_1, mot_degrees, -100, velocity=mspeed)
        #await runloop.sleep_ms(150)
        ea=self.get_360_mapped_yaw(stablize_first=True) #(motion_sensor.tilt_angles()[0])/10

        if correct_error:
            ed=self.compute_error(sa,ea,degrees,requested_deg_per_achived)
            myprint("spin: degrees:",degrees,"ea:",ea," sa:",sa, "error correction:",ed)
            #await self.spin_turn(ed,correct_error=False,mspeed=100)
            await self.turn_using_gyro(ed,speed=50,correct_error=False)


        if False and correct_error:
            ed=abs(degrees)-abs(abs(ea)-abs(sa))
            #ed = ed * -1 if degrees < 0 else 1
            myprint("degrees:",degrees,"ea:",ea," sa:",sa, "error correction:",ed)
            await self.spin_turn(ed,correct_error=False,mspeed=100)
        myprint("end yaw:",ea)


    async def pivot_turn(self, robot_degrees, motor_speed=200):
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


    async def move_forward_with_correction(self,distance_in_cm,velo=360,steer=0):
        await self.move(distance_in_cm,velo=velo,steer=steer, correct_error=True)


    async def move_forward(self,distance_in_cm,velo=360,steer=0):
        await self.move(distance_in_cm,velo=velo,steer=steer)


    async def turn_right(self,deg,speed=200,use_gyro=False,correct_error=False): #200
        if use_gyro==False:
            await self.spin_turn(deg,speed,correct_error=correct_error)
            #await self.pivot_turn(deg,speed)
        else:
            await self.turn_using_gyro(deg,speed,correct_error=correct_error)


    async def turn_left(self,deg,speed=200,use_gyro=False,correct_error=False): #200
        if use_gyro==False:
            await self.spin_turn(-deg,speed,correct_error=correct_error)
            #await self.pivot_turn(-deg,speed)
        else:
            await self.turn_using_gyro(-deg,speed,correct_error=correct_error)

    async def _arm_reset(self,port,angle_overdrive,initial_postition_deg,speed):
        #In motor.run_for_degrees function, positive deg spins counter-clock and negative deg spins counter-clock
        cp=motor.absolute_position(port)
        rp=motor.relative_position(port)
        myprint("_arm_reset port:",port," before ap:",cp, " rp:",rp)
        ret=await motor.run_for_degrees(port, angle_overdrive, speed)
        #await self.sleep(500)
        cp=motor.absolute_position(port)
        rp=motor.relative_position(port)
        myprint("After angle overdrive ap:",cp, " rp:",rp)
        motor.reset_relative_position(port,0)
        odsign= -1 if angle_overdrive<0 else 1
        ipd=abs(initial_postition_deg)*-odsign
        ret=await motor.run_for_degrees(port, ipd, speed)
        #await self.sleep(500)
        cp=motor.absolute_position(port)
        rp=motor.relative_position(port)
        myprint("After reset rp ap:",cp, " rp:",rp)
        #await sound.beep()
        pass

    async def _arm_reset_top_left(self,port,angle_overdrive=359,initial_postition_deg=150,speed=700):
        await self._arm_reset(port,angle_overdrive,initial_postition_deg,speed)

    async def _arm_reset_top_right(self,port,angle_overdrive=359,initial_postition_deg=150,speed=700):
        await self._arm_reset(port,-angle_overdrive,initial_postition_deg,speed)

    async def _arm_reset_back_left(self,port,angle_overdrive=359,initial_postition_deg=150,speed=700):
        await self._arm_reset(port,-angle_overdrive,initial_postition_deg,speed)

    async def _arm_reset_back_right(self,port,angle_overdrive=359,initial_postition_deg=150,speed=700):
        await self._arm_reset(port,angle_overdrive,initial_postition_deg,speed)

    async def _arm_reset_left_left(self,port,angle_overdrive=359,initial_postition_deg=150,speed=700):
        await self._arm_reset(port,-angle_overdrive,initial_postition_deg,speed)

    async def move_arm_relative(self,port,deg,speed):
        #In positive deg spins counter-clock and negative deg spins counter-clock
        ret=await motor.run_for_degrees(port, deg, speed)

    async def second_arm_reset(self,speed=700):
        if self.cfg["second_arm_motor_position"]=="top_left":
            await self._arm_reset_top_left(self.second_arm_port,speed)
        elif self.cfg["second_arm_motor_position"]=="left_left":
            await self._arm_reset_left_left(self.second_arm_port,speed)

    async def first_arm_reset(self,speed=700):
        await self._arm_reset_back_right(self.first_arm_port,speed)

    async def second_arm_up(self,degree=200,speed=700):
        if self.cfg["second_arm_motor_position"]=="top_left":
            await self.move_arm_relative(self.second_arm_port,abs(degree),speed)
        elif self.cfg["second_arm_motor_position"]=="left_left":
            await self.move_arm_relative(self.second_arm_port,-abs(degree),speed)
        myprint("second_arm_up done")


    async def second_arm_down(self,degree=200,speed=700):
        if self.cfg["second_arm_motor_position"]=="top_left":
            await self.move_arm_relative(self.second_arm_port,-abs(degree),speed)
        elif self.cfg["second_arm_motor_position"]=="left_left":
            await self.move_arm_relative(self.second_arm_port,abs(degree),speed)
        myprint("second_arm_down done")

    async def first_arm_up(self,degree=200,speed=700):
        await self.move_arm_relative(self.first_arm_port,abs(degree),speed)
        myprint("first_arm_up done")

    async def first_arm_down(self,degree=200,speed=700):
        await self.move_arm_relative(self.first_arm_port,-abs(degree),speed)
        myprint("first_arm_down done")

    '''
        async def second_arm_up(self,degree=130):
            await motor.run_to_absolute_position(self.second_arm_port,degree,200,direction=motor.CLOCKWISE)
            myprint("second_arm_up done")


        async def second_arm_down(self,degree=200):
            await motor.run_to_absolute_position(self.second_arm_port,degree,200,direction=motor.COUNTERCLOCKWISE)
            myprint("second_arm_down done")

        async def first_arm_up(self,degree=10,speed=200,force=False):
            #degree = (degree+self.first_arm_init_deg)%360
            cur_pos=motor.absolute_position(self.first_arm_port)
            print("first_arm_up: cur_pos:",cur_pos)
            #hack needed sometime motor does overdrive
            if not force and (cur_pos> 350 or cur_pos < 25): return
            await motor.run_to_absolute_position(self.first_arm_port,degree,speed,direction=motor.CLOCKWISE)
            myprint("second_arm_up done")


        async def first_arm_down(self,degree=180,speed=200,acce=1000):
            #degree = (degree+self.first_arm_init_deg)%360
            await motor.run_to_absolute_position(self.first_arm_port,degree,speed,direction=motor.COUNTERCLOCKWISE,acceleration=acce)
            myprint("second_arm_down done")
    '''

    async def sleep(self,sleep_ms):
        await runloop.sleep_ms(sleep_ms)



class FLL2025UnearthedMissions(FLLBaseLib):
    def __init__(self, cfg):
        super().__init__(cfg)


    async def robot_arm_reset(self):
        await self.first_arm_reset()
        await self.second_arm_reset()

    #########################################################################
    #########All Mission Code Starts from Here ##########################
    ########################################################################
    ######### Left side launch area missions#######################
    ############ race6 #################
    ###### Aligenment from right side 2nd block tip###
    async def mission_12_SO_30(self):
        await self.first_arm_reset()
        await self.second_arm_up(degree=180,speed=200)
        await self.move_forward(41,velo=700)
        await self.second_arm_down(degree=180)
        await self.move_backward(10,velo=650)
        await self.second_arm_up(degree=180)
        await self.move_backward(43,velo=700)
        return
        await self.turn_left(60)
        await self.move_forward(20,velo=180)
        await self.turn_right(40)
        await self.move_forward(50,velo=700,steer=2)
        pass

    async def mission_4_CR_30(self):
    ### Race 7####
    ### Alignment right side 4 blocks to tip####
        await self.first_arm_reset()
        await self.first_arm_up(degree=90)
        await self.move_forward(82,velo=700)
        await self.first_arm_down(degree=104)
        await self.turn_right(118)
        await self.move_forward(10,velo=50)
        #await self.first_arm_down(degree=180)
        await self.turn_right(90)
        await self.move_forward(41,velo=700)
        await self.move_backward(43,velo=700)
        await self.first_arm_up(degree=120,speed=100)
        await self.move_backward(20,velo=50)
        pass


    ###race 8####
    #### Alignment direct facing to the shovel to grab it.. run this if shovel standing straight up ###
    async def mission_1_SB_10(self):
        await self.move_forward(40,velo=700)
        await self.first_arm_up(degree=120,speed=100)
        await self.move_backward(40,velo=700)

    ##### race1    #######
    ### Alignment right side 4 blocks to tip####
    async def mission_1_SB_30_2_MR_30(self):
        await self.first_arm_reset()
        await self.second_arm_reset()
        #await self.second_arm_down(degree=100,speed=100)
        await self.move_forward(53,velo=500)
        await self.turn_left(40)
        await self.first_arm_down(degree=180,speed=100)
        await self.move_forward(23,velo=400)
        await self.second_arm_up(degree=220,speed=100)
        await self.first_arm_up(degree=-180,speed=100)
        await self.turn_right(40)
        await self.move_backward(80,velo=700)
        pass

##### race2    #######
##### Alignment from right side 4 blocks to tip####
##### 17 secs
    async def mission_03_13_ME_SR_30_30(self):
     await self.first_arm_reset()
     await self.move_forward(74,velo=700)
     await self.turn_right(105)
     await self.first_arm_down(degree=60,speed=250)
     await self.move_forward(16,velo=700 )
     await self.first_arm_up(degree=85,speed=100)
     await self.sleep(10)
     await self.first_arm_down(degree=85,speed=100)
     await self.turn_right(80)
     await self.move_forward(7.2,velo=700)
     await self.first_arm_up(degree=230,speed=700)
     await self.move_backward(35,velo=700)
     await self.turn_right(70)
     await self.move_forward(80,velo=800)
    pass
    # await self.first_arm_down(degree=90,speed=250)
    #await self.turn_right(19)
    #await self.move_forward(55,velo=700 )
    #await self.first_arm_up(degree=85,speed=100)
    #await self.turn_right(193)
    #await self.first_arm_down(degree=260,speed=900)

    ########## Right Side launch area missions code starts from here###############
    ###########################################################################


    async def mission_5_WL_6_Forge_SB_60_new(self):
        await self.first_arm_reset(speed=200)
        await self.second_arm_reset(speed=200)
        await self.second_arm_up(degree=15,speed=200)

        #getting ready for forge
        await self.move_forward(40,velo=700)
        await self.turn_left(55)
        await self.second_arm_down(degree=10,speed=200)
        await self.move_forward(20)
        await self.turn_right(87)

        #preping to move the lever
        await self.move_forward(8)
        # await self.second_arm_down(degree=30,speed=200)
        #this was 42
        await self.turn_left(40,speed=700)

            #getting ready for who lives here
        await self.move_backward(5)
        await self.second_arm_down(degree=10,speed=200)
        # await self.second_arm_up(degree=20,speed=200)
        await self.turn_left(27)
        
        #getting ready to move the lever for who live there
        await self.move_forward(6,steer=0)
        await self.turn_left(80,speed=100)
        
        #getting ready for tip your scale
        # await self.second_arm_down(degree=30,speed=200)
        # await self.turn_left(8)
        # await self.move_forward(3)
        # await self.turn_left(39)
        await self.move_backward(10)
        await self.turn_left(46)

        await self.second_arm_up(degree=80,speed=200)
        
        await self.move_forward(30)
        #await self.move_backward(70,velo=700)
        await self.turn_left(96)
        await self.first_arm_up(45)
        await self.move_forward(4,velo=700)
        await self.first_arm_down(80)
        await self.first_arm_up(80)
        await self.move_backward(6,velo=700)
        await self.turn_right(120)
        await self.move_backward(90,velo=800,steer=-5)

       
    async def mission_5_WL_6_Forge_SB_60(self):
       await self.first_arm_reset()
       await self.second_arm_reset()
       await self.second_arm_up(degree=15,speed=200)
       #getting ready for forge
       await self.move_forward(40,velo=600)
       await self.turn_left(55)
       await self.second_arm_down(degree=10,speed=200)
       await self.move_forward(21)
       await self.turn_right(83,speed=200)
       #preping to move the lever
       await self.move_forward(9.2)
       await self.turn_right(25,speed=700)
       await self.turn_left(70,speed=700)
       
       #getting ready for who lives here
       await self.move_backward(10)
       await self.second_arm_down(degree=10,speed=200)
       await self.turn_left(10,speed=200)
       #getting ready to move the lever for who live there
       await self.move_forward(10.5,steer=0)
       await self.turn_right(25,speed=700)
       await self.turn_left(95)
       #getting ready for tip your scale
       await self.move_backward(10)
       await self.turn_left(46)
       await self.second_arm_up(degree=80,speed=200)
       await self.move_forward(29)
       await self.turn_left(124,speed=200)
       await self.first_arm_up(45)
       await self.move_forward(4,velo=700)
       await self.first_arm_down(80)
       await self.first_arm_up(80)
       await self.move_backward(7,velo=700)
       await self.turn_right(105)
       await self.move_backward(90,velo=800)



    async def mission_8_Silo_30(self):
        await self.second_arm_reset()
        await self.second_arm_up(degree=150)
        await self.move_forward(37,velo=800)
        #await self.second_arm_up(degree=150)
        await self.second_arm_down(degree=180)
        await self.second_arm_up(degree=180)
        await self.second_arm_down(degree=180)
        await self.second_arm_up(degree=180)
        await self.second_arm_down(degree=180, speed=1100)
        await self.second_arm_up(degree=180)
        # await self.second_arm_down(degree=180, speed=1100)
       # await self.second_arm_up(degree=180)
        await self.move_backward(38,velo=700)

    async def mission_7_HL_HeavyLift_30(self):
     await self.first_arm_reset()
     await self.first_arm_up(degree=150,speed=200)
     #await self.second_arm_up(degree=150,speed=200)
     await self.move_forward(3,velo=700)
     await self.turn_right(100)
     await self.move_forward(33,velo=700)
     await self.turn_left(90)
     await self.move_forward(50,velo=700)
     await self.first_arm_down(degree=150,speed=200)
     await self.turn_left(30)
     await self.move_forward(10,velo=700)
     await self.first_arm_up(degree=150,speed=200)
     await self.move_backward(30,velo=700)
     await self.turn_right(30)
     await self.move_backward(100,velo=700)

    async def mission_10_Send_0(self):
     await self.move_forward(500,velo=1000)
    
    async def mission_09_WoS_30(self):
     await self.second_arm_down(140)
     await self.move_forward(43,velo=800)
     await self.second_arm_down()
     await self.move_backward(15,velo=400)
     await self.sleep(1)
     await self.second_arm_up(degree=20)
     await self.move_backward(4,velo=100)
     await self.second_arm_down(55)
     await self.turn_left(60,speed=1100)
     await self.second_arm_up()
     await self.move_backward(10,velo=100)
     await self.turn_right(150)
     await self.move_backward(50,velo=100)

     return
     await self.second_arm_down(degree=150)
     await self.turn_left(60)
     await self.move_forward(31,velo=800)
     await self.second_arm_up(degree=150)
     await self.turn_right(45)
     await self.move_forward(31,velo=800)
     await self.second_arm_down(50)
     await self.turn_left(60)

    

        #await self.turn_left(30)
        #await self.move_forward(20)

    async def race1(self):
        await self.mission_7_HL_HeavyLift_30()
        pass

    async def race2(self):
        await self.mission_5_WL_6_Forge_SB_60()
        pass

    async def race3(self):
        await self.mission_8_Silo_30()
        pass

    async def race4(self):
        await self.mission_09_WoS_30()
        pass

    async def race5(self):
        await self.mission_10_Send_0()
        pass

    async def race7(self):
        await self.mission_09_WoS_30()
        pass

    async def race8(self):
        await self.mission_rasing_mast_30_20()
        pass

    async def race9(self):
        await self.mission_moving_ship_30_20()
        pass
    
    async def race6(self):
        await self.mission_10_Send_0()
        pass

    
    async def test(self):
        pass

        await self.first_arm_reset()
        await self.first_arm_down()
        await sound.beep()
        await self.first_arm_up(degree=200)
        await sound.beep()
        return
        #total 200 degrees freedom of movement
        await self.second_arm_up(degree=100)

        return
        await self.second_arm_reset()
        return
        await self.second_arm_down()
        await sound.beep()
        return
        #total 200 degrees freedom of movement
        await self.second_arm_up(degree=100)
        await sound.beep()
        await self.second_arm_up(degree=100)
        await sound.beep()
        await self.second_arm_down(degree=100)
        await sound.beep()
        await self.second_arm_up(degree=100)

        pass





async def main():
    # write your code here
    #light_matrix.write("Hi!")
    cfg_ls={
        "left_wheel_port":port.B,
        "right_wheel_port":port.A,
        "first_color_sensor_port":port.F,
        "second_color_sensor_port":port.E,
        "first_arm_port":port.C,
        "second_arm_port":port.D, #port.D
        "first_arm_motor_position":"back_right",
        "second_arm_motor_position":"top_left"
        }
    cfg_cc={
        "left_wheel_port":port.B,
        "right_wheel_port":port.D,
        "first_color_sensor_port":port.E,
        "second_color_sensor_port":port.F,
        "first_arm_port":port.C,  # Center arm/motor
        "second_arm_port":port.A, #port.D# Side arm
        "first_arm_motor_position":"top_right",
        "second_arm_motor_position":"left_left"
        }
    cfg=cfg_cc
    fll_match_missions=FLL2025UnearthedMissions(cfg)
    #await ls_robot.follow_line(10000)

    test=False
    race1=True  #HeavyLift
    race2=False #Fodge/Whats on Sale/ Tip the scale
    race3=False #Silo
    race4=False #Whats on Sale
    race5=False # send the robot
    race6=False
    race7=False
    race9=False
    race10=False
    arm_reset=False
    if arm_reset:
        await fll_match_missions.robot_arm_reset()
    #test
    if test:
        await fll_match_missions.test()
        return
    #test done
    #race 1
    if race1:
        await fll_match_missions.race1()
    #race 1 ends

    #race 2
    if race2:
        await fll_match_missions.race2()
    #race 2 ends

    #race 3
    if race3:
        await fll_match_missions.race3()
    #race 3 ends

    #race4
    if race4:
        await fll_match_missions.race4()

    #race 5
    if race5:
        await fll_match_missions.race5()

    #race6
    if race6:
        await fll_match_missions.race6()

    if race7:
        await fll_match_missions.race7()

    if race8:
        await fll_match_missions.race8()

    if race9:
        await fll_match_missions.race9()

    if race10:
        await fll_match_missions.race10()
    #races done



    if False:
        myprint("calling move reverse")
        await fll_match_missions.move_backward(60,velo=1110)
        myprint("calling move fwd")
        await fll_match_missions.move_forward(60,velo=1110)
        #await runloop.sleep_ms(100)
        myprint("calling move turn left")
        #await fll_match_missions.turn_left(90,use_gyro=False,correct_error=True,speed=600)
        await runloop.sleep_ms(500)
        myprint("calling move turn right")
        #await fll_match_missions.turn_right(90,use_gyro=False,correct_error=True,speed=600)
        #await fll_match_missions.turn_right(90,use_gyro=True)
        myprint("calling move end of main")
    sys.exit(0)

#main()
runloop.run(main())
