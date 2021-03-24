
import numpy as np
packet = np.array(range(1,61), dtype = np.uint8)
packet = [0, 0, 1, 128, 128, 127, 248, 113, 0] + [0]*47 + [128, 127, 249, 241]

def check_control_sum(data, length):
    def xor(a, b):
        result = ""
        for i in range(len(a)):
            if a[i] == b[i]:
                result += "0"
            else:
                result += "1"
        return result
    data = np.uint32(data)
    
    checksum = "{0:032b}".format(0)

    for i in range(0,length, 4):
        data_32 = "{0:008b}".format(data[i+3]) + "{0:008b}".format(data[i+2]) + "{0:008b}".format(data[i+1]) + "{0:008b}".format(data[i])
        checksum = xor(checksum, data_32)
    return [int(checksum[0:8],2), int(checksum[8:16],2), int(checksum[16:24],2), int(checksum[24:32],2)][::-1]
    
checksum = check_control_sum(packet, 56)
print(checksum)