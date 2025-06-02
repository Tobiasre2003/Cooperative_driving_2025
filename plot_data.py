import csv
import os
import numpy as np
import matplotlib.pyplot as plt 
import math
from tkinter import filedialog
from cri import cri
from os import listdir
from os.path import isfile, join
import scipy.signal as ss

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


def median_filer(data, data_range):
    new_data = []
    for n in range(len(data)):
        sample = data[max(0, n-data_range):n] + data[n:min(len(data), n+data_range)]
        while None in sample:sample.remove(None)
        
        if len(sample) == 0: 
            new_data.append(None)
        else:
            new_data.append(np.median(np.array(sample)))
    
    return new_data

def moving_average(data, data_range):
    new_data = []
    for n in range(len(data)):
        sample = data[max(0, n-data_range+1):n+1] #+ data[n:min(len(data), n+data_range)]
        while None in sample:sample.remove(None)
        
        if len(sample) == 0: 
            new_data.append(None)
        else:
            new_data.append(np.mean(np.array(sample)))
    
    return new_data





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
        
            #print(file_data[file]['speed'])
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
    camera_switch = {}
    for file in file_data:
        camera_switch[file] = []
        x = file_data[file]['x']
        y = file_data[file]['y']
        for n in range(len(x)):
            if not file in entry_range:
                p = [(654+2023)/2, (5237+4765)/2]
                d = np.sqrt((x[n]-p[0])**2 + (y[n]-p[1])**2)
                if d < 3000:
                    entry_range[file] = n
            
            if n == 0: continue
            d = np.sqrt((x[n]-x[n-1])**2 + (y[n]-y[n-1])**2)
            if d > 200:
                camera_switch[file].append(file_data[file]['time'][n])

    plot_data = {}

    for file_name in file_data.keys():
        data_set = file_data[file_name]
        plot_data[file_name] = {}
        for param_type in parameters:

            if not param_type in data_set.keys(): continue
            if param_type == 'time': continue
            data = data_set[param_type][:size]
            time = data_set['time'][:size]
            
            # data = lim_change(data, 0.05) # hindrar snabba svängar     0.005
            # data = median_filer(data, 10)
            #data = moving_average(data, 5)
            #print(data)
            
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

    return plot_data, entry_range, camera_switch


def get_files_in_folder():
    directory = filedialog.askdirectory(title="Select Folder:")
    files = [directory+'/'+f for f in listdir(directory) if isfile(join(directory, f))]
    return files





def eval_cri(m, r, cb = False, ia=False): # hitta nedsaktnings punkt
    plot_data_dict = {}
    entry_range = {}
    camera_switch = {}

    #plt.figure(figsize=(9, 9))

    for n in range(len(m)):
        pd, er, cs = get_plot_data([m[n], r[n]], ['cri'])
        entry_range.update(er)
        plot_data_dict.update(pd)
        camera_switch.update(cs)

    cris = []

    entry_sign_x = 0
    entry_sign_c = 0
    last_cri = []
    first_cri = []
    last_time = 0
    
    for file_name in plot_data_dict:
                
        for name in plot_data_dict[file_name]:
            data = plot_data_dict[file_name][name][2]
            time = plot_data_dict[file_name][name][1]
            param_type = plot_data_dict[file_name][name][0]
            
            name = name[6:]
            
            plt.plot(time, data, label = name)
            #plt.plot(time[entry_range[file_name]], 0, marker = 'o')
            if ia:
                if not len(m) == 1:
                    plt.axvline(x=time[entry_range[file_name]], color='gray', linestyle='--', linewidth=1)
                else:
                    plt.axvline(x=time[entry_range[file_name]], color='gray', linestyle='--', linewidth=1, label = 'Inträdesavstånd')
                

            if not time[entry_range[file_name]] == None:
                entry_sign_x += time[entry_range[file_name]] 
                entry_sign_c += 1
            
            entry_range_cri = data[entry_range[file_name]]
            entry_range_cri = entry_range_cri if not entry_range_cri == None else 1
            
            first_cri.append(entry_range_cri)
            
            data.reverse()
            time.reverse()
            for i in range(len(data)):
                v = data[i]
                if not v == None:
                    cris.append(entry_range_cri - v)
                    last_cri.append(v)
                    last_time = time[i] if time[i] > last_time else last_time
                    break
      
    if cb:         
        cs = {} 
            
        for file_name in camera_switch:
            
            csl = camera_switch[file_name]
            
            color = 'red'
            if 'bot 5' in file_name: color = 'blue'
            new_csl = []
            
            for t in csl:
                if t < last_time:
                    new_csl.append(t)
            
            if len(new_csl) == 0: continue
            
            try:
                cs[color][0] = min(cs[color][0], new_csl[0])
                cs[color][1] = max(cs[color][1], new_csl[-1])
            except:
                cs[color] = [new_csl[0], new_csl[-1]]
            
        for color in cs:
            if color == 'red':
                style = '--'
                label = 'Kamerabyte 1'
            else:
                style = '-.'
                label = 'Kamerabyte 2'

            plt.axvspan(cs[color][0], cs[color][-1], color=color, alpha=0.15, label=label)

    
                
        
    s = ''
    for a in [f'{round(c, 4)}&' for c in cris]: s+=a
    print(s)
    print(f'\n{np.array(cris).sum()/len(cris)}')
    
    print(np.array(last_cri).sum()/len(last_cri))
    
    print(np.array(first_cri).sum()/len(first_cri))
    
    if len(m) == 10:
        handles, labels = plt.gca().get_legend_handles_labels() 
        if cb: order = [0,2,3,4,5,6,7,8,9,1,10,11] 
        else: order = [0,2,3,4,5,6,7,8,9,1] 
    
    if not len(m) == 1 and ia: 
        plt.text(entry_sign_x/entry_sign_c, 0.2, 'Inträdesavstånd', ha='center', va='top', fontsize=10,
            bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="black", lw=1))
    plt.ylabel('cri')
    plt.xlabel('tid [s]')
    plt.title('Cut-in risk indicator (cri)')
    if len(m) == 10: plt.legend([handles[i] for i in order], [labels[i] for i in order], loc='upper right')
    else: plt.legend()
    
    plt.show()
    


# m = get_files_in_folder()
# r = get_files_in_folder()

# # m = [filedialog.askopenfilename()]
# # r = [filedialog.askopenfilename()]

# eval_cri(m,r, True, True)    





def plot_mult(data:list[list], dist:int):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for n in range(len(data)):
        [x,y] = data[n]
        ax.plot(x, y, n*dist, color='r')


    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()



def eval_cri_2(folders): 
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    for i in range(len(folders)):
        
        m,r = folders[i]
    
        plot_data_dict = {}
        entry_range = {}
        camera_switch = {}


        for n in range(len(m)):
            pd, er, cs = get_plot_data([m[n], r[n]], ['cri'])
            entry_range.update(er)
            plot_data_dict.update(pd)
            camera_switch.update(cs)

 
        for file_name in plot_data_dict:
                    
            for name in plot_data_dict[file_name]:
                data = plot_data_dict[file_name][name][2]
                time = plot_data_dict[file_name][name][1]
                param_type = plot_data_dict[file_name][name][0]
                name = name[6:]
                
                z = np.full_like(time, i*10)
                data = np.array([val if not val == None else np.inf for val in data])
                time = np.array(time)
                
                
                ax.plot(time, data, z)
                


    ax.set_xlabel('Tid [s]')
    ax.set_ylabel('CRI')

    # ax.legend()
    plt.show()
    
    


    
def eval_cri_3(folders, labels, cb=False): 
    color_cycle = ['r','g','b','y']
    
    for i in range(len(folders)-1,-1,-1):

        first = True
        m,r = folders[i]
    
        plot_data_dict = {}
        entry_range = {}
        camera_switch = {}
        last_time = 0

        for n in range(len(m)):
            pd, er, cs = get_plot_data([m[n], r[n]], ['cri'])
            entry_range.update(er)
            plot_data_dict.update(pd)
            camera_switch.update(cs)

 
        for file_name in plot_data_dict:
                    
            for name in plot_data_dict[file_name]:
                data = plot_data_dict[file_name][name][2]
                time = plot_data_dict[file_name][name][1]
                
                if first:
                    plt.plot(time, data, color_cycle[i], label = labels[i])
                    first = False
                else:
                    plt.plot(time, data, color_cycle[i])
                    
                data.reverse()
                time.reverse()
                for j in range(len(data)):
                    v = data[j]
                    if not v == None:
                        last_time = time[j] if time[j] > last_time else last_time
                        break
      
                    
        if cb:         
            cs = {} 
                
            for file_name in camera_switch:
                
                csl = camera_switch[file_name]
                
                color = 'red' if i == 0 else 'yellow'
                if 'bot 5' in file_name: color = 'blue' if i == 0 else 'green'
                new_csl = []
                
                for t in csl:
                    if t < last_time:
                        new_csl.append(t)
                
                if len(new_csl) == 0: continue
                
                try:
                    cs[color][0] = min(cs[color][0], new_csl[0])
                    cs[color][1] = max(cs[color][1], new_csl[-1])
                except:
                    cs[color] = [new_csl[0], new_csl[-1]]
                
            for color in cs:
                if color == 'red':
                    label = 'Kamerabyte 1'
                elif color == 'blue':
                    label = 'Kamerabyte 2'
                elif color == 'yellow':
                    label = 'Kamerabyte 1 (tidigare arbete)'
                elif color == 'green':
                    label = 'Kamerabyte 2 (tidigare arbete)'

                
                plt.axvspan(cs[color][0], cs[color][-1], color=color, alpha=0.1, label=label, linewidth=3)
                
                
    handles, labels = plt.gca().get_legend_handles_labels() 
    order = [3,4,5,0,1,2] 
    plt.legend([handles[i] for i in order], [labels[i] for i in order], loc='upper right')
    plt.show()
    
def data_average(data, time):
    new_data = []
    new_time = []
    n_max = 0
    for i in range(len(data)):
        array = data[i]
        if len(array)>n_max:
            n_max = len(array)
            new_time = time[i]
            
    for n in range(n_max):
        values = []
        for array in data:
            val = array[n] if n < len(array) else None
            val = val if not val == None else np.inf
            values.append(val)
        
        new_data.append(np.mean(np.array(values)))
    
    return new_data, new_time
        
def data_max(data, time):
    new_data = []
    new_time = []
    n_max = 0
    for i in range(len(data)):
        array = data[i]
        if len(array)>n_max:
            n_max = len(array)
            new_time = time[i]
            
    for n in range(n_max):
        values = []
        for array in data:
            len_diff = n_max - len(array)
            val = array[n-len_diff] if n-len_diff >= 0 else None
            val = val if not val == None else np.inf
            values.append(val)
        new_data.append(max(values))
        
    return new_data, new_time
       
        
def remove_last(data):
    for n in range(len(data)-1,-1,-1):
        try:
            float(data[n])
            return data[:n+1]
        except: pass    
        

from matplotlib.colors import to_hex 


base_colormaps = {
    0: plt.colormaps.get_cmap('Blues'),
    1: plt.colormaps.get_cmap('Greens'),
    2: plt.colormaps.get_cmap('Reds'),
} 

def eval_cri_4(folders, labels): 
    num_curves_per_group = 2
    
    for i in range(len(folders)):
        
        cmap = base_colormaps[(i%3)]

        first = True
        m,r = folders[i]
    
        plot_data_dict = {}
        entry_range = {}
        camera_switch = {}

        for n in range(len(m)):
            pd, er, cs = get_plot_data([m[n], r[n]], ['cri'])
            entry_range.update(er)
            plot_data_dict.update(pd)
            camera_switch.update(cs)

        data_list = []
        time_list = []
        
        for file_name in plot_data_dict:
            for name in plot_data_dict[file_name]:
                data = plot_data_dict[file_name][name][2]
                time = plot_data_dict[file_name][name][1]
            
                data = remove_last(data)
                time = time[:len(data)]
                
                data_list.append(data)
                time_list.append(time)
            
        shade = (int(np.floor(i/3)) + 1) / (num_curves_per_group + 1)
        color = to_hex(cmap(shade))
        data, time = data_average(data_list, time_list)

        plt.plot(time, data, color=color, label=labels[i])

    plt.xlim(right=10)
    plt.ylim((0, 1.35))     
    
    handles, labels = plt.gca().get_legend_handles_labels() 
    order = [5,4,3,2,1,0]
    plt.title('Cut-in Risk Indicator')
    plt.ylabel('CRI')
    plt.xlabel('Tid [s]')
    plt.legend([handles[i] for i in order], [labels[i] for i in order], loc='upper right')
    plt.tight_layout()
    plt.show()
    



