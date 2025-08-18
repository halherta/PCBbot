import time
from machine import Pin, PWM, ADC, time_pulse_us

class RobotDirection:
    STP = 0
    FWD = 1
    REV = 2
    BRK = 3
    LEFT = 4
    CKW = 4
    RIGHT = 5
    CCKW = 5
    
class MotorDirection:
    STP = 0
    FWD = 1
    REV = 2
    BRK = 3
        
class PCBBotPins:
    AN0 = 2
    AN1 = 1
    AN2 = 4
    AN3 = 5
    TRG1 = 6
    ECH1 = 7
    TRG2 = 47
    ECH2 = 48
    MOT1A = 11
    MOT1B = 12
    MOT1EN = 10
    MOT1ENCA = 16
    MOT1ENCB = 15
    MOT2A = 8
    MOT2B = 18
    MOT2EN = 9
    MOT2ENCA = 21
    MOT2ENCB = 14
    
class PCBBot:
    def __init__(self):
        self.mot1a = Pin(PCBBotPins.MOT1A, Pin.OUT)
        self.mot1b = Pin(PCBBotPins.MOT1B, Pin.OUT)
        self.mot2a = Pin(PCBBotPins.MOT2A, Pin.OUT)
        self.mot2b = Pin(PCBBotPins.MOT2B, Pin.OUT)
        self.mot1en = PWM(Pin(PCBBotPins.MOT1EN), freq=500, duty_u16=65535)
        self.mot2en = PWM(Pin(PCBBotPins.MOT2EN), freq=500, duty_u16=65535)
        
        self.ech1 = Pin(PCBBotPins.ECH1,mode=Pin.IN, pull=None)
        self.trg1 = Pin(PCBBotPins.TRG1, mode=Pin.OUT, pull=None)
        
        self.ech2 = Pin(PCBBotPins.ECH2,mode=Pin.IN, pull=None)
        self.trg2 = Pin(PCBBotPins.TRG2, mode=Pin.OUT, pull=None)
        
        self.an0 = ADC(PCBBotPins.AN0)
        self.an1 = ADC(PCBBotPins.AN1)
        self.an2 = ADC(PCBBotPins.AN2)
        self.an3 = ADC(PCBBotPins.AN3)
        
        self.obstacle_avoidance_mode=False
        self.line_following_mode=False
        
    def mot1_move(self,direction=MotorDirection.STP, speed=0.5):
        if direction == MotorDirection.BRK :
            self.mot1a.value(1)
            self.mot1b.value(1)
        elif direction == MotorDirection.FWD :
            self.mot1a.value(0)
            self.mot1b.value(1)
        elif direction == MotorDirection.REV :
            self.mot1a.value(1)
            self.mot1b.value(0)
        else:
            self.mot1a.value(0)
            self.mot1b.value(0)
    
        self.mot1en.duty_u16(int(speed*65535))
        
    def mot2_move(self,direction=MotorDirection.STP, speed=0.5):
        if direction == MotorDirection.BRK :
            self.mot2a.value(1)
            self.mot2b.value(1)
        elif direction == MotorDirection.FWD :
            self.mot2a.value(1)
            self.mot2b.value(0)
        elif direction == MotorDirection.REV :
            self.mot2a.value(0)
            self.mot2b.value(1)
        else:
            self.mot2a.value(0)
            self.mot2b.value(0)
        
        self.mot2en.duty_u16(int(speed*65535))   
    
    def robot_move(self, direction=RobotDirection.STP,speedmot1=0.5, speedmot2=0.5):
        if direction == RobotDirection.BRK :
            self.mot1_move(MotorDirection.BRK, 1.0)
            self.mot2_move(MotorDirection.BRK, 1.0)
        elif direction == RobotDirection.FWD :
            self.mot1_move(MotorDirection.FWD, speedmot1)
            self.mot2_move(MotorDirection.FWD, speedmot2)
        elif direction == RobotDirection.REV :
            self.mot1_move(MotorDirection.REV, speedmot1)
            self.mot2_move(MotorDirection.REV, speedmot2)
        elif direction == RobotDirection.RIGHT :
            self.mot1_move(MotorDirection.FWD, speedmot1)
            self.mot2_move(MotorDirection.REV, speedmot2)
        elif direction == RobotDirection.LEFT :
            self.mot1_move(MotorDirection.REV, speedmot1)
            self.mot2_move(MotorDirection.FWD, speedmot2)
        else:
            self.mot1_move(MotorDirection.STP, 1.0)
            self.mot2_move(MotorDirection.STP, 1.0)
    
    def robot_move_with_delay(self, direction=RobotDirection.STP, delay_ms=100, speedmot1=0.5, speedmot2=0.5):
        self.robot_move(direction,speedmot1, speedmot2)
        time.sleep_ms(delay_ms)
        self.robot_move(RobotDirection.BRK,1.0, 1.0)
        time.sleep_ms(100)
        
        
    def read_line_sensors(self):
        return self.an0.read_uv()/1.0e6, self.an1.read_uv()/1.0e6, self.an2.read_uv()/1.0e6, self.an3.read_uv()/1.0e6

    def ultrasonic_left(self,timeout_us=30000):
        self.trg1.value(0) # Stabilize the sensor
        time.sleep_ms(1)
        # Send a 10us pulse.
        self.trg1.value(1)
        time.sleep_us(10)
        self.trg1.value(0)
        
        pulse_time = time_pulse_us(self.ech1, 1,timeout_us)
        if pulse_time == -2 or pulse_time == -1 :
            print("timeout")
            return pulse_time
        else:
            mm = pulse_time * 100 // 582
            return mm

    def ultrasonic_right(self,timeout_us=30000):
        self.trg2.value(0) # Stabilize the sensor
        time.sleep_ms(1)
        
        # Send a 10us pulse.
        self.trg2.value(1)
        time.sleep_us(10)
        self.trg2.value(0)
        
        pulse_time = time_pulse_us(self.ech2, 1, timeout_us)
        if pulse_time == -2 or pulse_time == -1 :
            print("timeout")
            return pulse_time
        else:
            mm = pulse_time * 100 // 582
            return mm
        
    def enable_object_avoidance_timed(self, distance_th, back_delay_ms, right_delay_ms, left_delay_ms, fwd_speed, manoeuvre_speed, duration_ms):
        """
        Run obstacle avoidance for a specific duration
    
        Args:
            distance_th: Distance threshold in mm
            back_delay_ms: Time to reverse when obstacle detected
            right_delay_ms: Time to turn right
            left_delay_ms: Time to turn left  
            fwd_speed: Forward movement speed (0.0-1.0)
            manoeuvre_speed: Speed for maneuvers (0.0-1.0)
            duration_ms: How long to run obstacle avoidance in milliseconds
        """
        print(f"Starting obstacle avoidance for {duration_ms}ms")
        self.obstacle_avoidance_mode=True
        self.robot_move(RobotDirection.FWD, fwd_speed, fwd_speed)
        time.sleep_ms(50)
        start_time = time.ticks_ms()
    
        while self.obstacle_avoidance_mode==True and time.ticks_diff(time.ticks_ms(), start_time) < duration_ms:
            
            
            distance_right = self.ultrasonic_right()
            time.sleep_ms(10)
            distance_left = self.ultrasonic_left()
            time.sleep_ms(10)
        
            print(f"distance right: {distance_right}, distance_left: {distance_left}")
            if(distance_right < 0 or distance_left < 0):
                print('one of the sensors timed out...re-reading sensors...')
                continue
            if(distance_right < distance_left and distance_right < distance_th):
                self.robot_move_with_delay(RobotDirection.STP, 100)
                self.robot_move_with_delay(RobotDirection.REV, back_delay_ms, manoeuvre_speed, manoeuvre_speed)
                self.robot_move_with_delay(RobotDirection.STP, 100)
                self.robot_move_with_delay(RobotDirection.RIGHT, right_delay_ms, manoeuvre_speed, manoeuvre_speed)
                self.robot_move_with_delay(RobotDirection.STP, 100)
                self.robot_move(RobotDirection.FWD, fwd_speed, fwd_speed)
            elif(distance_left < distance_right and distance_left < distance_th):
                self.robot_move_with_delay(RobotDirection.STP, 100)
                self.robot_move_with_delay(RobotDirection.REV, back_delay_ms, manoeuvre_speed, manoeuvre_speed)
                self.robot_move_with_delay(RobotDirection.STP, 100)
                self.robot_move_with_delay(RobotDirection.LEFT, left_delay_ms, manoeuvre_speed, manoeuvre_speed)
                self.robot_move_with_delay(RobotDirection.STP, 100)
                self.robot_move(RobotDirection.FWD, fwd_speed, fwd_speed) 
            else:
                self.robot_move(RobotDirection.FWD, fwd_speed, fwd_speed)
        
            #time.sleep_ms(100)
    
        # Stop the robot when time is up
        print("Obstacle avoidance time expired - stopping robot")
        self.robot_move(RobotDirection.STP)
        self.obstacle_avoidance_mode = False

    def enable_line_following_timed(self, base_speed, turn_speed, duration_ms):
        """
        Run line following for a specific duration using 4 TCRT5000 sensors
        
        Args:
            base_speed: Base forward movement speed (0.0-1.0)
            turn_speed: Speed for turning maneuvers (0.0-1.0)  
            duration_ms: How long to run line following in milliseconds
        """
        print(f"Starting line following for {duration_ms}ms")
        self.line_following_mode = True
        start_time = time.ticks_ms()
        
        # Voltage thresholds
        BLACK_THRESHOLD = 0.8  # Above this voltage = black line detected
        WHITE_THRESHOLD = 0.3  # Below this voltage = white background detected
        
        while self.line_following_mode == True and time.ticks_diff(time.ticks_ms(), start_time) < duration_ms:
            # Read all four sensors (an0=rightmost, an3=leftmost when moving forward)
            sensor_right, sensor_mid_right, sensor_mid_left, sensor_left = self.read_line_sensors()
            
            # Convert to binary: True = black line detected, False = white background
            right_on_line = sensor_right > BLACK_THRESHOLD
            mid_right_on_line = sensor_mid_right > BLACK_THRESHOLD
            mid_left_on_line = sensor_mid_left > BLACK_THRESHOLD
            left_on_line = sensor_left > BLACK_THRESHOLD
            
            right_off_line = sensor_right < WHITE_THRESHOLD
            mid_right_off_line = sensor_mid_right < WHITE_THRESHOLD
            mid_left_off_line = sensor_mid_left < WHITE_THRESHOLD
            left_off_line = sensor_left < WHITE_THRESHOLD
            
            print(f"Sensors: R={sensor_right:.2f} MR={sensor_mid_right:.2f} ML={sensor_mid_left:.2f} L={sensor_left:.2f}")
            print(f"On line: R={right_on_line} MR={mid_right_on_line} ML={mid_left_on_line} L={left_on_line}")
            
            # Line following logic based on sensor combinations
            if (mid_left_on_line and left_off_line) or (mid_right_on_line and right_off_line):
                # Both middle sensors on line - go straight
                self.robot_move(RobotDirection.FWD, base_speed, base_speed)
                print("Going straight")
                
            elif left_on_line:
                # Line detected on far left - sharp right turn
                self.robot_move(RobotDirection.RIGHT, turn_speed * 0.8, turn_speed)
                print("Sharp left turn")
                
            elif right_on_line:
                # Line detected on far right - sharp right turn
                self.robot_move(RobotDirection.LEFT, turn_speed, turn_speed * 0.8)
                print("Sharp right turn")
                
            elif not any([right_on_line, mid_right_on_line, mid_left_on_line, left_on_line]):
                # No line detected - stop or search
                self.robot_move(RobotDirection.STP)
                print("Line lost - stopping")
                
            else:
                # Default case - move forward slowly
                self.robot_move(RobotDirection.FWD, base_speed * 0.5, base_speed * 0.5)
                print("Default forward")
            
            time.sleep_ms(10)  # Small delay for stability
        
        # Stop the robot when time is up
        print("Line following time expired - stopping robot")
        self.robot_move(RobotDirection.STP)
        self.line_following_mode = False    



#     def enable_object_avoidance_mode(self, distance_th, back_delay_ms, right_delay_ms, left_delay_ms, fwd_speed, manoeuvre_speed):
#         self.obstacle_avoidance_mode = True
#         #self.robot_move(RobotDirection.FWD,fwd_speed,fwd_speed)
#         while self.obstacle_avoidance_mode == True:
#             time.sleep_us(100)
#             distance_right = self.ultrasonic_right()
#             time.sleep_us(100)
#             distance_left = self.ultrasonic_left()
#             time.sleep_us(100)
#             print(f"distance right: {distance_right}, distance_left: {distance_left}")
#             if(distance_right < distance_left and distance_right < distance_th):
#                 self.robot_move_with_delay(RobotDirection.STP,100)
#                 self.robot_move_with_delay(RobotDirection.REV,back_delay_ms,manoeuvre_speed,manoeuvre_speed)
#                 self.robot_move_with_delay(RobotDirection.LEFT,left_delay_ms,manoeuvre_speed,manoeuvre_speed)
#                 self.robot_move(RobotDirection.FWD,fwd_speed,fwd_speed)
#             elif(distance_left < distance_right and distance_left < distance_th):
#                 self.robot_move_with_delay(RobotDirection.STP,100)
#                 self.robot_move_with_delay(RobotDirection.REV,back_delay_ms,manoeuvre_speed,manoeuvre_speed)
#                 self.robot_move_with_delay(RobotDirection.RIGHT,left_delay_ms,manoeuvre_speed,manoeuvre_speed)
#                 self.robot_move(RobotDirection.FWD,fwd_speed,fwd_speed) 
#             else:
#                 self.robot_move(RobotDirection.FWD,fwd_speed,fwd_speed)
#             time.sleep_us(100000)
#                 


#bot = PCBBot()
#while True:
#    (r, mr, ml, l) = bot.read_line_sensors()
#    print(f'Left: {l}, Middle Left: {ml}, Middle Right: {mr}, Right: {r}')
#    time.sleep_ms(1000)
    
#bot.enable_line_following_timed(0.5, 0.3, 10):
