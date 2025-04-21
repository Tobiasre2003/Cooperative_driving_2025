from data_structures import *
from read_csv_file import *
import numpy as np
import matplotlib.pyplot as plt 
import math


def start_time(paths):
    starting_time = min([paths[name][1][3] for name in paths.keys()])
    starting_index_dict = {}
    
    for name in paths:
        for i in range(len(paths[name])):
            time = paths[name][i][3]
            if starting_time - time:
                if abs(starting_time-time) < abs(starting_time - paths[name][max(i-1, 0)][3]):
                    starting_index_dict[name] = i
                else:
                    starting_index_dict[name] = max(i-1, 0)
                break
    
    return starting_index_dict

def get_points(files):
    time_dict = {}
    data = get_data(files, ['mti'])
    
    for file in files:
        time_dict[file] = {}
        
        for i in range(len(data[file]['mti'])):
            mti = data[file]['mti'][i]
            
            if not mti == None and not 'entery_range' in time_dict[file].keys():
                time_dict[file]['entery_range'] = data[file]['time'][i]
                
            elif mti == 0 and 'entery_range' in time_dict[file].keys()  and not 'entry' in time_dict[file].keys():
                time_dict[file]['entry'] = data[file]['time'][i]
                
            elif mti == None and 'entry' in time_dict[file].keys() and not 'exit' in time_dict[file].keys():
                time_dict[file]['exit'] = data[file]['time'][i]

    return time_dict


def ttc(paths, starting_index, n):
    times = {}
    for name in paths:
        index = starting_index[name] + n
        ang = paths[name][index][1]
        velocity_vector = Vector(math.cos(ang), math.sin(ang))
        times[name] = {}
        
        for name_2 in paths:
            if name == name_2: continue
            index_2 = starting_index[name_2] + n
            
            dist = paths[name][index][0].distance_between_points(paths[name_2][index_2][0])
            dist /= 1100
            collision_vector = paths[name_2][index_2][0] - paths[name][index][0]
            collision_vector = Vector(collision_vector.x, collision_vector.y)
            speed = collision_vector.scalar_projection(velocity_vector)
            speed = max(speed, 0)
            
            time = dist / speed if not speed == 0 else math.inf
            
            times[name][name_2] = time
    
    return times       


def plot_ttc(files, markers = False):
    paths, size = get_paths(files)
    starting_index = start_time(paths)
    latest_start = max(list(starting_index.values()))
    
    ttcs = {}
    time_lines = {}
    
    for n in range(size - latest_start):
        times = ttc(paths, starting_index, n)
        
        for name in times:
            t = min(list(times[name].values()))
            
            if not name in ttcs:
                ttcs[name] = [t] 
                time_lines[name] = [paths[name][n][3]]
            else:
                ttcs[name].append(t)
                time_lines[name].append(paths[name][n][3])
        
    
    for name in ttcs:
        y = np.array(ttcs[name])
        x = np.array(time_lines[name])
        
        label = name
        label = "bot 4" if "bot 4" in name else label
        label = "bot 5" if "bot 5" in name else label
        
        plt.plot(x, y, label = label)
          
        
    if markers:
        markers = get_points(files)
        for file in markers.keys():
            name = file
            name = "bot 4" if "bot 4" in file else name
            name = "bot 5" if "bot 5" in file else name
            for marker_type in markers[file].keys():
                time = markers[file][marker_type]
                plt.plot(np.array([time]), np.array([0]), label = name + " - " + marker_type, marker = 'o', ms = 5)
        
        
    plt.legend()
    plt.show()



if __name__ == '__main__': 
    files = get_files(2) 
    plot_ttc(files, True)
    
    