import spi
class TMotorQDD:
    # TODO:
    # add tx_message dict
    # add rx_message dict
    def __init__(self, motor_id, CAN_ID):
        '''
        motor_id is ID of the motor (CAN MOTOR ID)
        ID is the CAN_ID [0,1,2,3]
        '''
        # TODO: Implement this
        self.name = None

        self.motor_id = motor_id
        self.CAN_ID = CAN_ID

        self.state_send = ['pos', 'vel', 'kp', 'kd', 'tor']

        self.state_bounds = [[-95.5, 95.5], 
                             [-30.0, 30.0], 
                             [0.0, 500.0], 
                             [0.0, 5.0], 
                             [-18.0, 18.0]]
        
        self.bounds = dict(zip(self.state_send, self.state_bounds))
        self.state_bits = dict(zip(self.state_send, [16,12,12,12,12]))
    
        self.des_state_int = dict(zip(self.state_send, [0,0,0,0,0]))
    
        self.zero_state = dict(zip(self.state_send, [0.,0.,0.,0.,0.]))
        self.zero_state_bytes = self.state_to_bytes(self.zero_state)

        self.states_recv =  ['pos', 'vel', 'tor']

        self.state_recv_bytes = dict(zip(self.states_recv, [b'\x00',b'\x00',b'\x00']))
        self.state_recv_ints = dict(zip(self.states_recv, [0,0,0]))
        self.state = dict(zip(self.states_recv, [0.,0.,0.]))


        self.commands = {'mot_mode_on': [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],
                         'mot_mode_off': [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD],
                         'set_zero': [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE], 
                         'empty':[0x00]*8, 
                         'zero_state': self.zero_state_bytes 
                        }
        self.t_data = self.commands['mot_mode_off']
        self.r_data = self.commands['empty']
        self.torque_limit = 1
        self.gear_ratio = 6
        self.torque_constant = 1
        
        # self.message = {self.device_id:self.commands['zero_state']}


    def __del__(self):
        print('Motor object was destructed')



    def float_to_uint(self, data_float, data_min, data_max, bits):
        span = data_max - data_min
        offset = data_min
        return int((data_float-offset)*(float((1<<bits)-1))/span)


    def uint_to_float(self, data_int,  data_min, data_max, bits):
        span = data_max - data_min
        offset = data_min
        return (float(data_int))*span/(float((1<<bits)-1)) + offset


    def state_to_bytes(self, state_dict):
        """Get the bytes from desired state"""
        for state_label in self.state_send:
            self.des_state_int[state_label] = self.float_to_uint(state_dict[state_label], self.bounds[state_label][0], self.bounds[state_label][1], self.state_bits[state_label]) 
        
        state_bytes = [self.des_state_int['pos']>>8,
                       self.des_state_int['pos']&0xFF, 
                       self.des_state_int['vel']>>4, 
                       ((self.des_state_int['vel']&0xF)<<4)|(self.des_state_int['kp']>>8), 
                       self.des_state_int['kp']&0xFF, 
                       self.des_state_int['kd']>>4, 
                       ((self.des_state_int['kd']&0xF)<<4)|(self.des_state_int['tor']>>8), 
                       self.des_state_int['tor']&0xff]
        data_out = [self.motor_id] + state_bytes
        return data_out



    def bytes_to_state(self, recived_bytes):
        """Prase the state from the motor reply"""
        i = 1 # shift
        if recived_bytes:
            self.state_recv_bytes['pos'] = (recived_bytes[i + 1]<<8)|recived_bytes[i + 2]
            self.state_recv_bytes['vel'] = (recived_bytes[i + 3]<<4)|(recived_bytes[i + 4]>>4)
            self.state_recv_bytes['tor'] = ((recived_bytes[i + 4]&0xF)<<8)|recived_bytes[i + 5]
            
            for state_label in self.states_recv:
                self.state[state_label] = self.uint_to_float(self.state_recv_bytes[state_label],
                                                             self.bounds[state_label][0], 
                                                             self.bounds[state_label][1], 
                                                             self.state_bits[state_label]) 
        else:
            pass
    

        

    def recive_reply(self):        
        _, _, self.reply = self.reciver()
        return self.reply


    def enable(self):
        """This will enable motor mode"""
        self.data_out = [self.motor_id] + self.commands['mot_mode_on']
        # data_out[56] = 0xAD
        # data_out[57] = 0xAD
        # data_out[58] = 0xAD
        # data_out[59] = 0xAD
        # data_to_send = tuple(data_out)
        # self.reply = spi.transfer(self.device, data_to_send)
        # self.bytes_to_state(self.reply)
        print('Motor mode enabled')

    def disable(self): 
        """This will disable motor mode"""
        self.data_out = [self.motor_id] + self.commands['mot_mode_off']
        print('Motor mode disabled')


    def set_zero(self):
        print(f'You are going to assign a new zero for motor with ID {self.motor_id}, press "Y" to continue...\n')
        # user_input = input()
        # if user_input == 'Y' or user_input == 'y':
        self.data_out = [self.motor_id] + self.commands['set_zero']
        print('New encoder zero is setted') 
        # else:
        #     print('Canceling...')
        

    def set_torque_limit(self, torque_limit):
        """Set new torque limit (defualt is 1)"""
        self.torque_limit = torque_limit
        print(f'Torque limit is seted to: {torque_limit}')
    

    def set_torque(self, torque):
        """Set the desired torque and execute command"""
        '''
        motor_id is the number of motor of i-th device
        ID is the CAN id of the motor 
        '''
        # if self.torque_limit:

        if torque>self.torque_limit:
            torque = self.torque_limit
        if torque<-self.torque_limit:
            torque = -self.torque_limit
        
        state_data_dict = self.zero_state
        state_data_dict['tor'] = torque
        # TODO: move this to the separate function (execute command)
        self.data_out = self.state_to_bytes(state_data_dict)
        # self.reply = spi.transfer(self.device, data_out)
        # self.bytes_to_state(self.reply)



    def set_state(self, state_data_dict):
        data_out = self.state_to_bytes(state_data_dict)
        self.reply = spi.transfer(self.device, data_out)
        self.bytes_to_state(self.reply)


    def set_pos(self, pos, kp, kd):
        state_data_dict = self.zero_state
        state_data_dict['kp'] = kp
        state_data_dict['kd'] = kd
        state_data_dict['pos'] = pos
        # TODO: move this to the separate function (execute command)
        self.data_out = self.state_to_bytes(state_data_dict)
        # self.reply = spi.transfer(self.device, data_out)
        # self.bytes_to_state(self.reply) 

    def set_kp(self, kp):
        pass 

    def set_kd(self, kd):
        pass