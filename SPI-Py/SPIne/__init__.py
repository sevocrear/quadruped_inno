import spi

class SPIne():
    def __init__(self, motors):
        self.device_0 = spi.openSPI(device="/dev/spidev1.1",
                           mode=0,
                           speed=1957000)
        self.device_1 = spi.openSPI(device="/dev/spidev1.0",
                           mode=0,
                           speed=1957000)

        self.data_out_0 = list([0]*60)
        self.data_in_0 = list([0]*60)

        self.data_out_1 = list([0]*60)
        self.data_in_1 = list([0]*60)

        self.data_out_0[0] = 3
        self.data_out_0[28] = 3
        self.data_out_1[0] = 3
        self.data_out_1[28] = 3

        self.data_out_0[1] = 1
        self.data_out_0[10] = 2
        self.data_out_0[19] = 3
        
        self.data_out_0[29] = 1
        self.data_out_0[38] = 2
        self.data_out_0[47] = 3

        self.data_out_1[1] = 1
        self.data_out_1[10] = 2
        self.data_out_1[19] = 3
        
        self.data_out_1[29] = 1
        self.data_out_1[38] = 2
        self.data_out_1[47] = 3

        # Control sums
        self.data_out_0[56] = 0xAD
        self.data_out_0[57] = 0xAD
        self.data_out_0[58] = 0xAD
        self.data_out_0[59] = 0xAD

        self.data_out_1[56] = 0xAD
        self.data_out_1[57] = 0xAD
        self.data_out_1[58] = 0xAD
        self.data_out_1[59] = 0xAD



        self.motors = motors
        self.motors_IDs = list(map(lambda x: [x.CAN_ID, x.motor_id], self.motors))

        self.spi_box_placing_0 = [[[0,2],[1,10]],
                                [[0,1],[10,19]],
                                [[0,0],[19,28]],
                                [[1,2],[29,38]],
                                [[1,1],[38,47]],
                                [[1,0],[47,56]]]
        
        self.spi_box_placing_1 = [[[2,2],[1,10]],
                                [[2,1],[10,19]],
                                [[2,0],[19,28]],
                                [[3,2],[29,38]],
                                [[3,1],[38,47]],
                                [[3,0],[47,56]]]
    
    def transfer_and_receive(self, ):
        for spi_box in self.spi_box_placing_0:
            if spi_box[0] in self.motors_IDs:
                box_range =  spi_box[1]
                self.data_out_0[box_range[0]:box_range[1]] = self.motors[self.motors_IDs.index(spi_box[0])].data_out
        for spi_box in self.spi_box_placing_1:
            if spi_box[0] in self.motors_IDs:
                box_range =  spi_box[1]
                self.data_out_1[box_range[0]:box_range[1]] = self.motors[self.motors_IDs.index(spi_box[0])].data_out

        self.data_in_0 = spi.transfer(self.device_0, tuple(self.data_out_0))

        self.data_in_1 = spi.transfer(self.device_1, tuple(self.data_out_1))
        # print(self.data_out_0)
        # print(self.data_in_0)

        for spi_box in self.spi_box_placing_0:
            if spi_box[0] in self.motors_IDs:
                box_range =  spi_box[1]
                self.motors[self.motors_IDs.index(spi_box[0])].bytes_to_state(self.data_in_0[box_range[0]:box_range[1]])
        for spi_box in self.spi_box_placing_1:
            if spi_box[0] in self.motors_IDs:
                box_range =  spi_box[1]
                self.motors[self.motors_IDs.index(spi_box[0])].bytes_to_state(self.data_in_1[box_range[0]:box_range[1]])
        
