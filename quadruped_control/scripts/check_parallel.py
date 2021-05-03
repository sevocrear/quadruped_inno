import multiprocessing as mp
import pandas as pd

U_sum = {'LF_leg':[], 'RF_leg':[], 'LB_leg':[], 'RB_leg': []}

def fill_dict(name, fill_data):
    U_sum[name]=fill_data
    return name, fill_data
def collect_result(result):
    global U_sum
    U_sum[result[0]] = result[1] 
    
names = ['LF_leg', 'RF_leg', 'LB_leg', 'RB_leg']
fill_data_list = [1,2,3,4]

for i in range(3):
    pool = mp.Pool(mp.cpu_count())
    for name, fill_data in zip(names, fill_data_list):
        # fill_dict(name, fill_data)
        pool.apply_async( fill_dict, args=(name, fill_data), callback = collect_result)

    pool.close()
    pool.join()  # postpones the execution of next line of code until all processes in the queue are done.
    print(U_sum)