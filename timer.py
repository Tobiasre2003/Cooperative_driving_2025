import math
import csv
import os
import math
from tkinter import filedialog
from os import listdir
from os.path import isfile, join
import numpy as np
import matplotlib.pyplot as plt
class Point:
    def __init__(self,x:float, y:float, z:float = 0):
        self.x = x
        self.y = y
        self.z = z
    
    def __add__(self, point):
        try: return Point(self.x+point.x, self.y+point.y)
        except: return Point(self.x+point, self.y+point)
    
    def __sub__(self, point):
        try: return Point(self.x-point.x, self.y-point.y)
        except: return Point(self.x-point, self.y-point)
    
    def __mul__(self, point):
        try: return Point(self.x*point.x, self.y*point.y)
        except: return Point(self.x*point, self.y*point)
    
    def distance_between_points(self, point) -> float:
        if point == None: return None
        return math.sqrt((self.x-point.x)**2 + (self.y-point.y)**2)
    

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

def get_data(files:list[str], parameters:list[str] = []):
    file_data = {}
    size = math.inf
    times = []
    
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
    
        for param_type in parameters:
            float_data = to_floats(data[param_type])
            file_parameters[param_type] = float_data
        
        file_data[file] = file_parameters

    for file in file_data.keys():
        for param_type in file_data[file].keys():
            file_data[file][param_type] = file_data[file][param_type][:size]
            
    return file_data

def get_files(number_of_files:int):
    files = []
    for _ in range(number_of_files):
        filename = filedialog.askopenfilename()
        files.append(filename)
    return files

def get_path(file:str):
    file_data = get_data([file], ['x', 'y'])
    size = 0
    path = []
    for file_name in file_data.keys():
        x_list = file_data[file_name]['x']
        y_list = file_data[file_name]['y']
        time_list = file_data[file_name]['time']
        size = len(x_list)
        
        for n in range(size):
            path.append([time_list[n], Point(x_list[n], y_list[n])])
            
    return path

def find_pos(point:Point, path):
    dist = None
    index = None
    for i in range(len(path)):
        pos = path[i]
        d = point.distance_between_points(pos[1])
        if dist == None or d < dist: 
            dist = d
            index = i
    
    return path[index]

def get_time_diff(start_point:Point, end_point:Point, file:str):
    path = get_path(file)
    start_pos = find_pos(start_point, path)
    end_pos = find_pos(end_point, path)
    time = end_pos[0] - start_pos[0]
    time = time if time >= 0 else None
    return time

def get_time(case:tuple, file:str = None):
    file = get_files(1)[0] if file == None else file
    time = get_time_diff(*case, file)
    print(file, ':', time)
    return time

def get_times(case:tuple, files:list):
    times = []
    for file in files:
        times.append(get_time(case, file))
    return times

def get_mean(case:tuple, number_of_files:int = 0, files:list = None):
    
    total = 0
    if files == None:
        for _ in range(number_of_files):
            total += get_time(case)
    else:
        number_of_files = len(files)
        for n in range(len(files)):
            total += get_time(case, files[n])
    
    mean = total / number_of_files
    print('Mean : ', mean)
    return mean

def get_diff(case:tuple):
    a = get_time(case)
    b = get_time(case)
    diff = abs(a-b)
    print('Diff : ', diff)
    return diff

def get_files_in_folder():
    directory = filedialog.askdirectory(title="Select Folder:")
    files = [directory+'/'+f for f in listdir(directory) if isfile(join(directory, f))]
    return files
    
def plot_normal_distribution(times):
    times = np.array(times)
    m = np.mean(times)
    s = np.sqrt(np.array([(x - m)**2 for x in times]).sum() / len(times))
    X = np.linspace(m - 4*s, m + 4*s, 4000)
    F = np.array([(1/(s*np.sqrt(2*np.pi))) * np.exp((-(x-m)**2)/(2*s**2)) for x in X])

    plt.xticks(np.array([m +s*i for i in range(-4,5)]), labels=[f'{round(m+i*s, 3)}\n{i}\u03C3' if not i == 0 else f'{round(m, 3)}\n\u03BC' for i in range(-4, 5)])
    percentage = [2.1, 13.6, 34.1, 34.1, 13.6, 2.1]
    color = ['cornflowerblue', 'royalblue', 'blue', 'blue', 'royalblue', 'cornflowerblue']
    
    for i in range(-3, 3):
        j = i+3
        fill_from = m + s*i
        fill_to = m + s*(i+1)
        if fill_from is not None and fill_to is not None:
            mask = (X >= fill_from) & (X <= fill_to)
            plt.fill_between(X[mask], F[mask], alpha=0.3, color=color[j])
            
            x_mid = (fill_from + fill_to) / 2
            y_max = F[mask].max()
            
            plt.text(x_mid, y_max * 0.6, f"{percentage[j]} %", 
                 ha='center', va='center', fontsize=10, color='black',
                 bbox=dict(boxstyle='round,pad=0.3', facecolor='lightsteelblue', edgecolor='black'))

    plt.plot(X, F)
    plt.legend()
    plt.grid(True)
    plt.show()


cases = {
    'intersection_1' : (Point(3846, 8028), Point(1862, 3450)), 
    'intersection_2' : (Point(3400, 500), Point(3487, 6122)), 
    'intersection_3' : (Point(3817, 6928), Point(1862, 3450)),
    'intersection_4' : (Point(722, 4120), Point(3361, 7297)),
    'intersection_5' : (Point(722, 4120), Point(4095, 1221)),
    'merging_main' : (Point(1020, 8920), Point(1150, 3380)),
    'merging_ramp' : (Point(3150, 7840), Point(1150, 3380)),
    'roundabout_1' : (Point(3817, 6928), Point(1862, 3450)),
    'roundabout_2' : (Point(1117, 4052), Point(2281, 3533))
}



# files = get_files_in_folder()
# a = get_mean(cases['merging_ramp'], files=files)
# print(a)


# files = get_files_in_folder()
# a = get_mean(cases['merging_ramp'], files=files)
# print(a)

# files = get_files_in_folder()
# a = get_mean(cases['merging_ramp'], files=files)
# print(a)





# def box_plot(title, times_list, ffss):
#     fig, axs = plt.subplots(1, len(times_list)*2, figsize=(2, 5))  

#     for n in range(len(times_list)):
#         times = times_list[n]
#         times = np.array(times)
#         ffs = np.array(ffss[n])
        
#         print(f'FFT: max: {max(ffs)}, min: {min(ffs)}')
#         print(f'KF: max: {max(times)}, min: {min(times)}')
        
#         axs[2*n].boxplot(ffs, patch_artist = True)
#         axs[2*n].set_title(f'FFT scenario {n}')   
#         axs[2*n+1].boxplot(times, patch_artist = True)
#         axs[2*n+1].set_title(f'KF scenario {n}')  
    
#     plt.tight_layout()
#     plt.show()


def box_plot(title, times_list, ffss):
    num_scenarios = len(times_list)
    fig, axs = plt.subplots(1, num_scenarios * 2, figsize=( 2.5*num_scenarios , 6))
    scenario_positions = []

    if num_scenarios == 1: axs = [axs] 

    for n in range(num_scenarios):

        ffs = np.array(ffss[n])
        min_f = np.min(ffs)
        max_f = np.max(ffs)
        median_f = np.median(ffs)
        q1_f, q3_f = np.percentile(ffs, [25, 75])

        ax_fft = axs[2 * n]
        ax_fft.boxplot(ffs, patch_artist=True, widths=0.5, whis=(0, 100))
        ax_fft.set_title(f'FFT')
        ax_fft.set_xticks([])
        ax_fft.set_yticks([])  

        times = np.array(times_list[n])
        min_t = np.min(times)
        max_t = np.max(times)
        median_t = np.median(times)
        q1_t, q3_t = np.percentile(times, [25, 75])

        ax_kf = axs[2 * n + 1]
        ax_kf.boxplot(times, patch_artist=True, widths=0.5, whis=(0, 100))
        ax_kf.set_title(f'KF')
        ax_kf.set_xticks([])
        ax_kf.set_yticks([])  

        
        for ax, values_colors in zip([ax_fft, ax_kf],
            [[(min_f, 'black'),(q1_f, 'black'),(median_f, 'black'),(q3_f, 'black'),(max_f, 'black')],
             [(min_t, 'black'),(q1_t, 'black'),(median_t, 'black'),(q3_t, 'black'),(max_t, 'black')]]):

            for val, color in values_colors:
                ax.annotate(f'{val:.1f}', xy=(0.995, val), xycoords=('axes fraction', 'data'),
                            fontsize=8, color=color, ha='right', va='center')
                
        scenario_positions.append((ax_fft, ax_kf)) 
                
    fig.suptitle(title)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    for i, (ax_fft, ax_kf) in enumerate(scenario_positions):
        pos_fft = ax_fft.get_position()
        pos_kf = ax_kf.get_position()
        center_x = (pos_fft.x0 + pos_kf.x1) / 2
        fig.text(center_x, 0.9, f'Scenario {i+1}', ha='center', fontsize=12, weight='bold')

    plt.show()


# def box_plot(title, times_list, ffss):
#     import matplotlib.pyplot as plt
#     import numpy as np

#     num_scenarios = len(times_list)
#     fig, axs = plt.subplots(num_scenarios, 2, figsize=(10, 2.5 * num_scenarios))

#     if num_scenarios == 1:
#         axs = [axs]  # axs blir en 1D-array om bara ett scenario

#     scenario_positions = []

#     for n in range(num_scenarios):
#         ffs = np.array(ffss[n])
#         min_f, max_f = np.min(ffs), np.max(ffs)
#         median_f = np.median(ffs)
#         q1_f, q3_f = np.percentile(ffs, [25, 75])

#         times = np.array(times_list[n])
#         min_t, max_t = np.min(times), np.max(times)
#         median_t = np.median(times)
#         q1_t, q3_t = np.percentile(times, [25, 75])

#         ax_fft = axs[n][0] if num_scenarios > 1 else axs[0]
#         ax_kf = axs[n][1] if num_scenarios > 1 else axs[1]

#         # Horisontella boxplots
#         ax_fft.boxplot(ffs, patch_artist=True, widths=0.5, whis=(0, 100), vert=False)
#         ax_kf.boxplot(times, patch_artist=True, widths=0.5, whis=(0, 100), vert=False)

#         ax_fft.set_title("FFT", loc='left')
#         ax_kf.set_title("KF", loc='left')

#         for ax in [ax_fft, ax_kf]:
#             ax.set_yticks([])
#             ax.set_xticks([])

#         for ax, values_colors in zip([ax_fft, ax_kf],
#             [[(min_f, 'black'), (q1_f, 'black'), (median_f, 'black'), (q3_f, 'black'), (max_f, 'black')],
#              [(min_t, 'black'), (q1_t, 'black'), (median_t, 'black'), (q3_t, 'black'), (max_t, 'black')]]):

#             for val, color in values_colors:
#                 ax.annotate(f'{val:.1f}', xy=(val, 0.85), xycoords=('data', 'axes fraction'),
#                             fontsize=8, color=color, ha='center', va='bottom')

#         scenario_positions.append((ax_fft, ax_kf))

#     fig.suptitle(title, fontsize=14)

#     # Justera layouten för att få plats med vänstermarginal
#     plt.subplots_adjust(left=0.2, top=0.92, bottom=0.05)

#     # Lägg till scenariotitlar efter layoutjustering
#     for i, (ax_fft, _) in enumerate(scenario_positions):
#         pos = ax_fft.get_position()
#         center_y = (pos.y0 + pos.y1) / 2
#         fig.text(0.1, center_y, f'Scenario {i+1}', ha='right', va='center', fontsize=12, weight='bold')

#     plt.show()



# def bar_plot(title, times, start_value = 0):
#     times = np.array(times) - start_value
#     labels = [f'Körning {n+1}' for n in range(len(times))]
#     mean = times.sum()/len(times)

#     plt.bar(labels, times, label='Individuell tid')
#     plt.axhline(mean, color='red', linestyle='--', label=f'Medelvärde: {mean:.3f} s')
#     plt.xticks(rotation=20)
    
#     print(f'max: {max(times)}, min: {min(times)}')
    
#     plt.ylim(max(min(times)-0.05,0) , max(times)+0.15)
#     plt.ylabel('Tid [s]')
#     plt.title(title)
#     plt.legend(loc='upper left')
#     plt.show()

# def bar_plot(title, tests):
#     data_list = []
#     labels_list = []
#     for data in tests:
#         labels = [f'Scenario {n+1}' for n in range(len(data))]
#         print(f'max: {max(data)}, min: {min(data)}')
#         data_list.extend(data)
#         labels_list.extend(labels)

#     data_list = np.array(data_list)
#     labels_list = np.array(labels_list)
    
#     print(data_list, labels_list)
    
#     plt.bar(labels_list, data_list)

#     # plt.xticks(rotation=20)
#     # # plt.ylim(max(min(data)-0.05,0) , max(data)+0.15)
#     # plt.ylabel('Tid [s]')
#     # plt.title(title)
#     # plt.legend(loc='upper left')
#     plt.show()









import numpy as np
import matplotlib.pyplot as plt
import itertools

def bar_plot(title, tests, names):
    
    data_list = []
    labels_list = []
    colors = []

    color_cycle = itertools.cycle(plt.cm.tab10.colors)  # 10 färger

    for i, data in enumerate(tests):
        color = next(color_cycle)
        labels = [f'{names[i][0]}{j+1}' for j in range(len(data))]

        print(f'Test {i+1} max: {max(data)}, min: {min(data)}')

        data_list.extend(data)
        labels_list.extend(labels)
        colors.extend([tuple(color)] * len(data))  # Konvertera till tuple

    data_list = np.array(data_list)
    labels_list = np.array(labels_list)

    # Plot
    plt.figure(figsize=(6, 5))
    bars = plt.bar(labels_list, data_list, color=colors)
    
    for bar in bars:
        height = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2, height * 1.01, f'{height:.4f}', 
                ha='center', va='bottom', fontsize=8)

    # Skapa legend
    unique_colors = list(dict.fromkeys(colors))
    legend_labels = [f"{names[i]}" for i in range(len(unique_colors))]
    legend_handles = [plt.Rectangle((0,0),1,1, color=col) for col in unique_colors]
    plt.legend(legend_handles, legend_labels,loc='lower left')
 
    #plt.xticks(rotation=45, ha='right')
    #plt.ylabel('Tid [s]')
    plt.title(title)
    plt.tight_layout()
    plt.show()














