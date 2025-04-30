import math

def first(file_data:dict):
    time_dict = {}
    min_time = math.inf
    first = None
    
    for file in file_data.keys():
        data = file_data[file]
        for n in range(len(data['dti'])):
            if data['dti'][n] == 0:
                time_dict[file] = data['time'][n]
                
    for file in time_dict.keys():
        if time_dict[file] < min_time:
            min_time = time_dict[file]
            first = file

    return first

def cri_AC(mti_A, mti_C, dti_A, dti_C):
    bot_length = 0.30
    ttc_CA = mti_C - mti_A
    #ttc_BC = mti_A + 4 - mti_C 
    d_a = max(0, (dti_C-dti_A) - bot_length)
    d_b = max(0, 1.5 - d_a - bot_length)
    k_a = d_a/(d_a+d_b)
    k_b = d_b/(d_a+d_b)
    #print(ttc_BC, ttc_CA, mti_C, mti_A)
    #cri_a = max(math.exp(-k_a), 0)
    cri_a = max(math.exp(-ttc_CA * k_a), 0)
    #cri_b = max(math.exp(-ttc_BC * k_b), 0)
    if ttc_CA <= 0: cri_a = None
    return cri_a 


def cri_CB(mti_B, mti_C, dti_B, dti_C):
    bot_length = 0.30
    ttc_BC = mti_B - mti_C
    #ttc_CA = mti_C - mti_B + 4 # Behöver lösas på annat sätt
    d_b = max(0, (dti_B-dti_C) - bot_length)
    d_a = max(0, 1.5 - d_b - bot_length)
    #k_a = d_a/(d_a+d_b)
    k_b = d_b/(d_a+d_b) 
    #print(ttc_BC,mti_C,mti_B)
    #cri_a = max(math.exp(-ttc_CA * k_a), 0)
    cri_b = max(math.exp(-ttc_BC*k_b), 0)
    if k_b <= 0: cri_b = 0
    print('CBCBCBCBCBCBCB')
    return cri_b


def get_value(time, data, time_point, main_time=None):
    index = 0 
    try:
        if main_time[1] > time_point:
            return data[0]
        else:
            for i in range(len(main_time)):
                if main_time[i] > time_point:
                    index = i
                    break
    except: pass
    
    if main_time == None:
        for j in range(len(time)):
            if time[j] >= time_point:
                index = j
                break
    return data[index]
    
    # for n in range(len(time)):
    #     if main_time[1] > time_point:
    #         return data[0]
    #     if time[n] >= time_point:
    #         index = n
    #         break
    # return data[index]
    

def cri(file_data, size, main_file_name, ramp_file_name):
    first_bot = first(file_data)
    main_mti_list = file_data[main_file_name]['mti'][:size]
    ramp_mti_list = file_data[ramp_file_name]['mti'][:size]
    main_dti_list = file_data[main_file_name]['dti'][:size]
    ramp_dti_list = file_data[ramp_file_name]['dti'][:size]
    cri = []
    for time in file_data[ramp_file_name]['time'][:size]:
        cri_value = None
        try:
            main_mti = get_value(file_data[ramp_file_name]['time'], main_mti_list, time, file_data[main_file_name]['time'])
            ramp_mti = get_value(file_data[ramp_file_name]['time'], ramp_mti_list, time)
            main_dti = get_value(file_data[ramp_file_name]['time'], main_dti_list, time, file_data[main_file_name]['time'])/1000
            ramp_dti = get_value(file_data[ramp_file_name]['time'], ramp_dti_list, time)/1000
            
            if main_mti == 0 or ramp_mti == 0: pass
            
            elif main_file_name == first_bot:
                cri_value = cri_AC(main_mti, ramp_mti, main_dti, ramp_dti)
            elif ramp_file_name == first_bot:
                cri_value = cri_CB(main_mti, ramp_mti, main_dti, ramp_dti)
            
            cri_value = max(0, min(1, cri_value))    
            
        except: pass
            
        cri.append(cri_value)
            
    file_data[ramp_file_name]['cri'] = cri
    
    
