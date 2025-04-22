import math
import csv
import os
import math
from tkinter import filedialog
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

def get_time(case):
    file = get_files(1)[0]
    time = get_time_diff(*case, file)
    print(file, ':', time)
    return time

def get_mean(case:tuple, number_of_files:int):
    total = 0
    for _ in range(number_of_files):
        total += get_time(case)
    
    mean = total / number_of_files
    print('Mean : ', mean)
    return mean

def get_diff(case):
    a = get_time(case)
    b = get_time(case)
    diff = abs(a-b)
    print('Diff : ', diff)
    return diff



cases = {
    'intersection_1' : (Point(3867, 8029), Point(1815, 3442)), 
    'intersection_2' : (Point(3382, 411), Point(3382, 6300)), 
}


get_diff(cases['intersection_2'])
