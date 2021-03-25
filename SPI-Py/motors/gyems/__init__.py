# TODO:
# add rotation counter 
# add low pass filtering of the cuirrent and velocity 
# put state in dictionary
# add conversion from raw data to state
# add sensor scales

from time import perf_counter
from math import pi

class GyemsDRC:
    """ This class provide interface to the Gyems BLDC motor driver over CAN socket"""
    def __init__(self, can_bus = None, device_id = 0x141, units = 'rad'):

        # TODO: pass dict with reciver/transmitter functions from the specific bus
        if not can_bus:
            print('Provide can_bus as argument')
            self.__del__()
        
        self.transmiter = can_bus.send_bytes
        self.reciver = can_bus.recive_frame

        self.device_id = device_id
        # TODO: 
        # Check iether parsing from dict affect speed or not
        self.protocol = dict()
        self.protocol = {'write_pid_ram': b'\x31', # write PI gains for position, velocity and current loops in to RAM
                         'write_pid_rom': b'\x32', # write PI gains for position, velocity and current loops in to ROM
                         'read_accel_lim' : b'\x33', # read the value of the acceleration limit
                         'write_accel_lim_ram' : b'\x34', # write accceleration limit to ram
                         'read_encoder_data' : b'\x90', # read the encoder data 
                         'set_encoder_offset' : b'\x91', # set encoder offset to the specofoc value
                         'set_encoder_zero_rom' : b'\x19', # set the current position as zero for encoder and save it tp ROM
                         'read_multiturn_angle' : b'\x92', # read the encoder data as cumalitive angle
                         'read_single_angle' : b'\x94', 
                         'read_motor_status_1' : b'\x9A',
                         'read_motor_status_2' : b'\x9C',
                         'read_motor_status_3' : b'\x9D',
                         'clear_error_flags' : b'\x9B',
                         'motor_off' : b'\x80',
                         'motor_stop' : b'\x81',
                         'motor_running' : b'\x88',
                         'set_torque' : b'\xA1',
                         'set_speed' : b'\xA2',
                         'set_pos_1' : b'\xA3',
                         'set_pos_2' : b'\xA4',
                         'set_pos_3' : b'\xA5',
                         'set_pos_4' : b'\xA6'}


        self.command = self.protocol['motor_off'] + 7*b'\x00'
        
        self.gains = {'pos':{'p':0, 'i':0},
                      'vel':{'p':0, 'i':0}, 
                      'cur':{'p':0, 'i':0}}

        self.speed_limit = None
        self.accel_limit = 0
        self.current_limit = 500
        self.torque_limit = 500
        self.encoder_offset = 0
        self.error_state = 'normal'

        self.torque_constant = 1
        self.encoder_scale = 16384
        
        # self.angle_scale = 2*pi/self.encoder_scale
        # self.encoder_grad_scale = 360/self.encoder_scale
        self.current_scale = 1
        # self.speed_scale = 1
        # self.speed_rad_scale = 2*pi/360
        self.temp_scale = 1
        self.units = units
        self.set_units(self.units)
    
        self.motor_status = ['on', 'off', 'error']
    
    
        state_labels = ['temp', 'angle', 'speed', 'torque', 'current']
        self.state = dict(zip(state_labels, [0,0,0,0,0]))


    
        self.voltage = 0
        self.temp = 0
        self.angle = 0
        self.pos = 0
        self.speed = 0 
        self.current = 0
        self.torque = 0
        self.phases_current = {'A':0,'B':0,'C':0}
        
        self.raw_state_data = {'temp':0, 
                               'encoder': 0,
                               'speed': 0, 
                               'current': 0} 
        
        self.encoder_prev = 0


        self.desired_speed = 0
        self.desired_pos = 0 
        self.desired_angle = 0 
        self.desired_torque = 0
        self.estimated_speed = 0 

        self.reply = 0
        self.time = 0
        self.dt = 0
        self.motor_turns = 0
        


    def to_bytes(self, n, integer, signed = True):
        return int(integer).to_bytes(n, byteorder='little', signed = signed)


    def from_bytes(self, byte_string, signed = True):
        return int.from_bytes(byte_string, byteorder='little', signed = signed)


    def send_command(self, command):
        self.transmiter(self.device_id, command)

    def recive_reply(self):        
        _, _, self.reply = self.reciver()
        return self.reply



    def clear_errors(self):
        command = self.protocol['clear_error_flags'] + 7*b'\x00'
        self.send_command(command)
        self.recive_reply()


    def check_errors(self):
        pass


    # Turn motor modes 
    def pause(self, clear_errors = False):
        if clear_errors:
            self.clear_errors()

        command = self.protocol['motor_stop'] + 7*b'\x00' # message = {0x141:  b'\x81\x00\x00\x00\x00\x00\x00\x00'}
        self.send_command(command)
        self.recive_reply()
            
        # pass
    
    def disable(self, clear_errors = True):
        if clear_errors:
            self.clear_errors()

        command = self.protocol['motor_off'] + 7*b'\x00' # message = {0x141:  b'\x81\x00\x00\x00\x00\x00\x00\x00'}
        self.send_command(command)
        self.recive_reply()
        

    def enable(self, clear_errors = False):
        if clear_errors:
            self.clear_errors()

        command = self.protocol['motor_running'] + 7*b'\x00' # message = {0x141:  b'\x81\x00\x00\x00\x00\x00\x00\x00'}
        self.send_command(command)
        self.recive_reply()
        


    def reset(self, go_to_zero = False):
        self.disable(clear_errors = True)
        self.enable()

    def go_to_zero(self):
        """Go to the specific point and set new zero at this point"""
        pass

    def set_as_zero(self):
        """Go to the specific point and set new zero at this point"""
        pass

    def set_degrees(self):
        """Set angle and speed scales for degrees"""
        self.angle_scale = 360/self.encoder_scale 
        self.speed_scale = 1/10


    def set_radians(self):
        """Set radians for angle and speed scales"""
        self.angle_scale = 2*pi/self.encoder_scale
        self.speed_scale = 2*pi/360


    def set_units(self, units = 'rad'):
        if units == 'deg':
            self.units = units
            self.set_degrees()
        else:
            self.units == 'rad'
            self.set_radians()



    def parse_sensor_data(self, reply):
        """parse the raw sensor data from the CAN frame"""
        
        self.raw_state_data['temp'] = reply[1]  
        self.raw_state_data['current'] = self.from_bytes(reply[2:4])
        self.raw_state_data['speed'] = self.from_bytes(reply[4:6])
        self.raw_state_data['encoder'] = self.from_bytes(reply[6:])
        return self.raw_state_data
    



    def multiturn_encoder(self, encoder_data, threshold = 8000,  velocity_data = 0):
        # self.velocity_estimate = self.encoder_prev
        if self.encoder_prev - encoder_data >= threshold:
            self.motor_turns +=1
        elif self.encoder_prev - encoder_data <= -threshold:
            self.motor_turns +=-1
        self.encoder_prev = encoder_data
        return encoder_data + (self.encoder_scale)*self.motor_turns



    # Parsing from the CAN frames      
    def parse_state(self, reply):
        """parse the motor state from CAN frame"""
        self.parse_sensor_data(reply) # parse the raw data to self.raw_state_data
        # state_labels = ['temp', 'angle', 'speed', 'torque', 'current']
        
        # some function to handle encoder 
        # self.time = time.time()
        self.state['angle'] = self.angle_scale*self.multiturn_encoder(self.raw_state_data['encoder'])
        self.state['temp'] = self.temp_scale*self.raw_state_data['temp']
        self.state['speed'] = self.speed_scale*self.raw_state_data['speed']
        self.state['current'] = self.current_scale*self.raw_state_data['current']
        self.state['torque'] = self.torque_constant*self.state['current']
        return self.state

    


    def check_angle(self, reply):
        t = perf_counter()
        dt = self.time - t
        self.estimated_speed = -self.angle_scale*(self.from_bytes(reply[6:]) - self.angle)/dt
        self.time = t

        



    def parse_status(self, reply):
        self.temp = reply[1]
        self.voltage = reply[3:5]
        self.error = reply[7]
        pass

    
    def parse_phases(self, reply):
        pass


    def parse_pos(self, reply):
        self.pos = self.from_bytes(reply[1:])



    
    def parse_pid(self, reply):
        self.gains = {'pos':{'p':reply[2], 'i':reply[3]},
                      'vel':{'p':reply[4], 'i':reply[5]}, 
                      'cur':{'p':reply[6], 'i':reply[7]}}
    

    def set_pid(self, gains, persistant = False):
        
        command = self.protocol['write_pid_ram']+b'\x00'
        memory_type = 'RAM'
        
        if persistant:
            print('New PID gains: will be setted to the ROM, type Y to continue')
            user_input = input()
            memory_type = 'ROM'
            if user_input == 'Y' or user_input == 'y':
                command = self.protocol['write_pid_rom']+b'\x00'
            else:
                print('Canceling, gains will be written to RAM')
            
        # TODO:
        # convert gains dict to array
        
        gains = [40, 40, 35, 15, 40, 40]
        for gain in gains:
            command += self.to_bytes(1, gain, signed = False)
        
        self.send_command(command)
        self.recive_reply()
        print(f'New gains are written to {memory_type}')

    
    def set_zero(self,persistant = False):
        """ Set a current position as a zero of encoder"""
        command = self.protocol['set_encoder_offset']+7*b'\x00'
        memory_type = 'RAM'

        if persistant:
            print('Current encoder value will be written as zero, type Y to continue')
            user_input = input()
            memory_type = 'ROM'
            if user_input == 'Y' or user_input == 'y':
                command = self.protocol['set_encoder_zero_rom']+7*b'\x00'
            else:
                print('Canceling, zero will be written to RAM')
        
        self.send_command(command)
        self.recive_reply()
        # print(f'New gains are written to {memory_type}')

        
        # print('')
    

    # ///////////////////////////
    # ///// Control Modes ///////
    # ///////////////////////////
    # 
    # Protocol:
    #   0xA1 - current control 
    #   0xA2 - speed control
    #   0xA3 - position control
    #   0xA4 - position control with speed limit 
    #  
    # ///////////////////////////
    

    def limiter(self, value, limit):
        if value>limit:
            value = limit
        if value < -limit:
            value = - limit
        return value


    def set_current(self, current):
        self.desired_current = self.limiter(current, self.current_limit)
        self.command = self.protocol['set_torque'] + 3*b'\x00' + self.to_bytes(2, self.desired_current) + 2*b'\x00'
        self.send_command(self.command)
        self.recive_reply()
        self.parse_state(self.reply)   
        # print(self.reply)

    def set_torque(self, torque, torque_limit = None):
        pass
    

    def set_speed(self, speed, accel_limit = None):
        self.desired_speed = 100*speed/self.speed_scale
        self.command = self.protocol['set_speed'] + 3*b'\x00' + self.to_bytes(4, self.desired_speed)
        self.send_command(self.command)
        self.recive_reply()
        self.parse_state(self.reply)        
        # sending message goes here


    def set_angle(self, angle, speed_limit = None):
        # TODO: Check scales
        self.desired_angle = angle
        if speed_limit:
            self.speed_limit = speed_limit

        if self.speed_limit:
            self.command = self.protocol['set_pos_2'] + b'\x00' + self.to_bytes(2, self.speed_limit) + self.to_bytes(4, self.desired_angle)
        else:
            self.command = self.protocol['set_pos_1'] + 3*b'\x00' + self.to_bytes(4, self.desired_angle)
        
        self.send_command(self.command)
        self.recive_reply()
        self.parse_state(self.reply)    
        # sending message goes here


    # Measurements 
    def get_state(self):
        pass

    def get_vel(self):
        pass

    def get_angle(self):
        pass

    def get_pos(self):
        pass

    def get_phases_current(self):
        pass

    # def 
    # def calibrate():