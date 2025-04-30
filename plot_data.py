import csv
import os
import numpy as np
import matplotlib.pyplot as plt 
import math
from tkinter import filedialog
from cri import cri
from os import listdir
from os.path import isfile, join

def write_csv(filnamn, data, rubriker):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(script_dir, filnamn)  
    fil_exists = os.path.exists(file_path)

    with open(file_path, mode="a", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        if not fil_exists:
            writer.writerow(rubriker)
        writer.writerow(data)


def read_bot_csv_file(filnamn):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(script_dir, filnamn)  

    data = {}

    with open(file_path, mode="r") as file:
        read = csv.reader(file)
        first_row = True
        names = []            
        for row in read:
            if first_row:
                for name in row:
                    data[name] = []  
                    names.append(name)
                first_row = False
                continue
            for n in range(len(names)):
                data[names[n]].append(row[n])
    return data
        
def to_floats(array):
    new_array = []
    for val in array:
        try:
            if val == "inf":
                new_array.append(math.inf)
                continue
            new_array.append(float(val))
        except:
            new_array.append(None)
    return new_array

def start_time(times):
    start_time = None
    start_time_str = ""
    for time_list in times:
        time = to_floats(time_list[0].split("-"))
        if start_time == None:
            start_time = time
            start_time_str = time_list[0]
        else:
            diff = [start_time[0]-time[0], start_time[1]-time[1], start_time[2]-time[2]]
            if diff[0] > 0 or (diff[0] == 0 and diff[1] > 0) or (diff[0] == 0 and diff[1] == 0 and diff[2] > 0):
                start_time = time
                start_time_str = time_list[0]
                
    return start_time_str   

def time_step(start, times):
    start_time_str_list = start.split("-")
    start_time_str_list[2] = '0.'+start_time_str_list[2]
    start_time_list = to_floats(start_time_str_list)
    start_time = start_time_list[0]*60 + start_time_list[1] + start_time_list[2]
    new_times = [0]
    for time in times[1:]:
        time_str_list = time.split("-")
        time_str_list[2] = '0.'+time_str_list[2]
        time_list = to_floats(time_str_list)
        time_diff = time_list[0]*60 + time_list[1] + time_list[2] - start_time
        new_times.append(time_diff)
    return new_times
        

def acc(time_list, speed_list):
    acc = [0]
    for n in range(1, len(time_list)):
        dt = time_list[n-1] - time_list[n]
        dv = speed_list[n-1] - speed_list[n]
        acc.append(dv/dt)
    return acc

def avg_speeds(data):
    for s in range(len(data)-50):
        if type(data[s]) == str: 
            continue
        new_speeds = []
        for i in range(50):
            new_speeds.append(data[s+i])
        avg_speed = sum(new_speeds)/50
        data[s] = avg_speed
    return data

def mti_avgspeed(speedlist, dtilist, mtilist):
    avg_mti = []
    for i in range(len(mtilist)):
        if type(mtilist[i]) == float: avg_mti.append(dtilist[i]/1000/speedlist[i])
        else: avg_mti.append(mtilist[i])
    return avg_mti


def lim_change(array, lim):
    new_array = []
    prev_element = None
    for element in array:
        
        if element == None:
            new_array.append(element)
            continue
            
        if prev_element == None: 
            prev_element = element
            new_array.append(element)
            continue
        
        new_element = prev_element + max(-lim, min(lim, element-prev_element))
        
        new_array.append(new_element) 
        prev_element = new_element

    return new_array

def plot_data(files, parameters:list[str]):
    file_data = {}
    size = math.inf
    times = []
    
    if type(files) == int:
        for _ in range(files):
            filename = filedialog.askopenfilename()
            data = read_bot_csv_file(filename)
            file_data[filename] = data
            times.append(data['time'])
            
    elif type(files) == list:
        for filename in files:
            data = read_bot_csv_file(filename)
            file_data[filename] = data
            times.append(data['time'])
    
    start_time_str = start_time(times)

    for file in file_data.keys():
        data = file_data[file]
        times = time_step(start_time_str, data['time'])
        size = min(len(times), size)
        file_parameters = {'time':times}
        params = parameters.copy()
        params.extend(['mti','dti'])
        for param_type in params:
            if param_type == 'cri': continue
            float_data = to_floats(data[param_type])
            file_parameters[param_type] = float_data
        
        file_data[file] = file_parameters

    try:
        for file in file_data.keys():
        
            print(file_data[file]['speed'])
            file_data[file]['speed'] = avg_speeds(file_data[file]['speed'])
            file_data[file]['mti'] = mti_avgspeed(file_data[file]['speed'], file_data[file]['dti'], file_data[file]['mti'])
    except: pass
    
    if 'cri' in parameters: 
        main_file = list(file_data.keys())[0]
        ramp_file = list(file_data.keys())[1]
        cri(file_data, size, main_file, ramp_file)

    for file_name in file_data.keys():
        data_set = file_data[file_name]
        for param_type in parameters:

            if not param_type in data_set.keys(): continue
            if param_type == 'time': continue
            data = data_set[param_type][:size]
            time = data_set['time'][:size]
            
            data = lim_change(data, 0.005) # hindrar snabba svängar
            d = data.copy()
            data = np.array(data)
            time = np.array(time)
            
            name = "bot 4" if "bot 4" in file_name else file_name
            name = "bot 5" if "bot 5" in file_name else file_name
            name += f' k{file_name[-5]}' if '_k' in file_name else ''
            plt.plot(time, data, label = name + " - " + param_type)
            
            d.reverse()
            for v in d:
                if not v == None:
                    print(v)
                    break
                
    plt.legend()
    plt.show()
            







def get_plot_data(files, parameters:list[str]):
    file_data = {}
    size = math.inf
    times = []
    
    if type(files) == int:
        for _ in range(files):
            filename = filedialog.askopenfilename()
            data = read_bot_csv_file(filename)
            file_data[filename] = data
            times.append(data['time'])
            
    elif type(files) == list:
        for filename in files:
            data = read_bot_csv_file(filename)
            file_data[filename] = data
            times.append(data['time'])
    
    start_time_str = start_time(times)

    for file in file_data.keys():
        data = file_data[file]
        times = time_step(start_time_str, data['time'])
        size = min(len(times), size)
        file_parameters = {'time':times}
        params = parameters.copy()
        params.extend(['mti','dti','x','y'])
        for param_type in params:
            if param_type == 'cri': continue
            float_data = to_floats(data[param_type])
            file_parameters[param_type] = float_data
        
        file_data[file] = file_parameters

    try:
        for file in file_data.keys():
        
            print(file_data[file]['speed'])
            file_data[file]['speed'] = avg_speeds(file_data[file]['speed'])
            file_data[file]['mti'] = mti_avgspeed(file_data[file]['speed'], file_data[file]['dti'], file_data[file]['mti'])
    except: pass
    
    if 'cri' in parameters: 
        main_file = list(file_data.keys())[0]
        ramp_file = list(file_data.keys())[1]
        cri(file_data, size, main_file, ramp_file)
        
        entry_range = {}
        for file in file_data:
            x = file_data[file]['x']
            y = file_data[file]['y']
            ## TODO
        

    plot_data = {}

    for file_name in file_data.keys():
        data_set = file_data[file_name]
        plot_data[file_name] = {}
        for param_type in parameters:

            if not param_type in data_set.keys(): continue
            if param_type == 'time': continue
            data = data_set[param_type][:size]
            time = data_set['time'][:size]
            
            data = lim_change(data, 0.005) # hindrar snabba svängar
            # data = np.array(data)
            # time = np.array(time)
            
            name = "bot 4" if "bot 4" in file_name else file_name
            name = "bot 5" if "bot 5" in file_name else file_name
            
            try:
                name += f' Körning {int(file_name[-6:-4])}'
            except:
                try:
                    name += f' Körning {int(file_name[-5])}'
                except:pass

            plot_data[file_name][name] = (param_type, time, data)

    return plot_data


def get_files_in_folder():
    directory = filedialog.askdirectory(title="Select Folder:")
    files = [directory+'/'+f for f in listdir(directory) if isfile(join(directory, f))]
    return files


# m = get_files_in_folder()
# r = get_files_in_folder()

# plot_data_dict = {}

# for n in range(len(m)-8):
#     plot_data([m[n], r[n]], ['cri', 'speed'])

# #plt.legend()
# plt.show()

def eval_cri(): # hitta nedsaktnings punkt
    m = get_files_in_folder()
    r = get_files_in_folder()

    plot_data_dict = {}

    for n in range(len(m)):
        plot_data_dict.update(get_plot_data([m[n], r[n]], ['cri']))

    cris = []

    for file_name in plot_data_dict:
        for name in plot_data_dict[file_name]:
            data = plot_data_dict[file_name][name][2]
            time = plot_data_dict[file_name][name][1]
            param_type = plot_data_dict[file_name][name][0]
            
            name = name[6:]
            
            plt.plot(time, data, label = name)
            
            data.reverse()
            for v in data:
                if not v == None:
                    cris.append(v)
                    break
    s = ''
    for a in [f'{round(c, 4)}&' for c in cris]: s+=a
    print(s)
    print(f'\n{np.array([1-v for v in cris]).sum()/len(cris)}')
    
    handles, labels = plt.gca().get_legend_handles_labels() 
    order = [0,2,3,4,5,6,7,8,9,1] 
    
    
    plt.ylabel('cri')
    plt.xlabel('tid [s]')
    plt.title('Cut-in risk indicator (cri)')
    plt.legend([handles[i] for i in order], [labels[i] for i in order])
    plt.show()
    
    
eval_cri()    





