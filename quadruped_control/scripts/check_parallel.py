from copy import deepcopy
U_sum  = {'LF_leg':[], 'RF_leg':[], 'LB_leg':[], 'RB_leg': []}
q_des = deepcopy(U_sum)
q_des['LF_leg'] = [1]

print(U_sum)
