import math

class Point:
    def __init__(self,x:float, y:float, z:float = 0):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return f'(X: {self.x}, Y: {self.y})'
    
    def __repr__(self):
        return f'(X: {self.x}, Y: {self.y})'
    
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
    
    def distance_between_points_in_list(self, list:list) -> float:
        try:
            prev_point = self
            tot_dist = 0
            for point in list:
                tot_dist += prev_point.distance_between_points(point)
                prev_point = point
            return tot_dist
        except: 
            return None

    def flip(self):
            return Point(self.y, self.x)


class Vector:
    def __init__(self, x:float, y:float):
        self.x = x
        self.y = y
    
    def __repr__(self):
        return f'(X: {self.x}, Y: {self.y})'

    def __add__(self, vector):
        try: return Vector(self.x+vector.x, self.y+vector.y)
        except: return Vector(self.x+vector, self.y+vector)
    
    def __sub__(self, vector):
        try: return Vector(self.x-vector.x, self.y-vector.y)
        except: return Vector(self.x-vector, self.y-vector)
    
    def __mul__(self, vector):
        try: return Vector(self.x*vector.x, self.y*vector.y)
        except: return Vector(self.x*vector, self.y*vector)

    def abs(self) -> float:
        return math.sqrt(self.x**2 + self.y**2)

    def dot_product(self, vector) -> float:
        return self.x * vector.x + self.y * vector.y

    def scalar_projection(self, vector) -> float:
        length = self.abs() 
        if length == 0: return math.inf
        return self.dot_product(vector) / length
    
    def get_orthogonal_vector(self):
        return Vector(self.y, -self.x)
    
    def get_unit_vector(self):
        length = self.abs() 
        if length == 0: length = 1
        return self*(1/length)
    
    def to_point(self) -> Point: 
        return Point(self.x, self.y)
        
    def vector_projection(self, vector):
        return self*self.scalar_projection(vector)
    
    def flip(self):
        return Vector(self.y, self.x)

    
class Object:
    def __init__(self, pos:Point, theta:float, borders:list = [], velocity_vector:Vector = Vector(0,0)):
        self.pos = pos
        self.direction = theta
        self.borders = borders
        self.velocity_vector = velocity_vector.get_unit_vector()
        
    def update(self, pos:Point, theta:float):
        self.pos = pos
        self.direction = theta
        
    def rotation_matrix(self, sign:int = 1) -> tuple:
        return (Vector(math.cos(sign*self.direction), math.sin(sign*self.direction)), Vector(-math.sin(sign*self.direction), math.cos(sign*self.direction)))
    
    def get_global_velocity_vector(self):
        rotation_matrix = self.rotation_matrix(-1)
        x = rotation_matrix[0].dot_product(self.velocity_vector)
        y = rotation_matrix[1].dot_product(self.velocity_vector)
        return Vector(x, y).get_unit_vector()
    
    def point_from_local_to_global(self, local_point:Point) -> Point:
        if local_point == None: return None
        rotation_matrix = self.rotation_matrix(-1)
        x = rotation_matrix[0].dot_product(local_point) + self.pos.x
        y = rotation_matrix[1].dot_product(local_point) + self.pos.y
        return Point(x, y)
    
    def from_local_to_global(self) -> list:
        borders = []
        for local_point in self.borders:
            borders.append(self.point_from_local_to_global(local_point))
        return borders
    
    def point_from_globl_to_local(self, global_point:Point) -> Point:
        rotation_matrix = self.rotation_matrix()
        x = rotation_matrix[0].dot_product(global_point) - rotation_matrix[0].dot_product(self.pos)
        y = rotation_matrix[1].dot_product(global_point) - rotation_matrix[1].dot_product(self.pos)
        return Point(x, y)
    
    def from_globl_to_local(self, global_borders:list) -> list:
        borders = []
        for global_point in global_borders:
            borders.append(self.point_from_globl_to_local(global_point))
        return borders
    
    def update_borders(self, borders):
        self.borders = borders
    
    def point_in_object(self, local_point:Point):
        prev_point = self.borders[-1]
        sign = 0
        for point in self.borders:
            lp = local_point - prev_point 
            vector = Vector(point.x-prev_point.x, point.y-prev_point.y).get_orthogonal_vector()
            dist = vector.scalar_projection(lp)
            prev_point = point
            if sign == 0: sign = math.copysign(1, dist)
            elif not sign == math.copysign(1, dist): return False
        return True and not sign == 0
    
    def overlapping(self, obj):
        is_overlapping = False
        obj_borders = self.from_globl_to_local(obj.from_local_to_global())
        
        for point in obj_borders:
            if self.point_in_object(point):
                is_overlapping = True
        
        return is_overlapping
        
    def get_outer_points(self):
        side_min_point = self.pos
        side_max_point = self.pos
        front_max_point = self.pos
        side_min_dist = math.inf
        side_max_dist = -math.inf
        front_max_dist = -math.inf

        for point in self.borders:
            side_dist = self.velocity_vector.get_orthogonal_vector().get_unit_vector().dot_product(point)
            front_dist = self.velocity_vector.get_unit_vector().dot_product(point)
            
            if side_dist>side_max_dist: 
                side_max_point = point
                side_max_dist = side_dist
            if side_dist<side_min_dist: 
                side_min_point = point
                side_min_dist = side_dist
            if front_dist>front_max_dist:
                front_max_dist = front_dist
                front_max_point = point
        
        return side_min_point, side_max_point, self.velocity_vector.vector_projection(front_max_point).to_point()
    
    def collision_course(self, obj):
        p1, p2, front_point = self.get_outer_points()
        sign = 0
        closest_distance = math.inf
        collision = False
        self_collision_point = None
        
        for point in self.from_globl_to_local(obj.from_local_to_global()):
            point = Point(point.y, point.x)
            dist_from_vector_1 = self.velocity_vector.get_orthogonal_vector().get_unit_vector().dot_product(point+p1)
            dist_from_vector_2 = self.velocity_vector.get_orthogonal_vector().get_unit_vector().dot_product(point+p2)
            
            if dist_from_vector_1*dist_from_vector_2>0: 
                if sign==0: sign = math.copysign(1,dist_from_vector_1)
                if sign != math.copysign(1,dist_from_vector_1):
                    if self.velocity_vector.dot_product(point) >= 0: collision = True
                
            if (dist_from_vector_1<=0 and dist_from_vector_2>=0): 
                if self.velocity_vector.dot_product(point) >= 0: collision = True
            
            if collision:    
                obj_collision_point = self.velocity_vector.vector_projection(point).to_point()
                distance = obj_collision_point.distance_between_points(front_point)
                if distance < closest_distance: 
                    closest_distance = distance
                    self_collision_point = obj_collision_point
                    
        
        try: front_point = front_point.flip()
        except: front_point = None
        
        try: self_collision_point = self_collision_point.flip()
        except: self_collision_point = None
        
        
        return self.point_from_local_to_global(front_point).distance_between_points(self.point_from_local_to_global(self_collision_point)), (self.point_from_local_to_global(front_point), self.point_from_local_to_global(self_collision_point))

    
    def crossing_vector(self, self_point:Point, obj_point:Point, self_vel:Vector, obj_vel:Vector):
        try:
            if not obj_vel.x == 0:
                self_dist = (obj_point.y-self_point.y+obj_vel.y*((self_point.x-obj_point.x)/obj_vel.x))/(self_vel.y-(obj_vel.y*self_vel.x)/obj_vel.x)
                return (self_vel*self_dist).to_point() + self_point 
            elif not self_vel.x == 0:
                self_dist = (self_point.y-obj_point.y+self_vel.y*((obj_point.x-self_point.x)/self_vel.x))/(obj_vel.y-(self_vel.y*obj_vel.x)/self_vel.x)
                return (obj_vel*self_dist).to_point() + obj_point 
        except ZeroDivisionError:
            return None
     
    def moving_collision_course(self, obj):
        self_p1, self_p2, front_point = self.get_outer_points()
        obj_p1, obj_p2, obj_front_point = obj.get_outer_points()
        
        self_p1 = self_p1.flip()
        self_p2 = self_p2.flip()
        front_point = front_point.flip()
        obj_front_point = obj_front_point.flip()
        obj_p1 = obj_p1.flip()
        obj_p2 = obj_p2.flip()
        
        self_p1 = self.point_from_local_to_global(self_p1)
        self_p2 = self.point_from_local_to_global(self_p2)
        front_point = self.point_from_local_to_global(front_point)
        obj_front_point = obj.point_from_local_to_global(obj_front_point)
        obj_p1 = obj.point_from_local_to_global(obj_p1)
        obj_p2 = obj.point_from_local_to_global(obj_p2)
        
        self_vel = self.get_global_velocity_vector().flip()
        obj_vel = obj.get_global_velocity_vector().flip()
        
        self_vel.y = -self_vel.y
        obj_vel.y = -obj_vel.y
        
        crossing_point_1 = self.crossing_vector(self_p1, obj_p2, self_vel, obj_vel)
        crossing_point_2 = self.crossing_vector(self_p2, obj_p1, self_vel, obj_vel)
        
        try: 
            obj_sign_1 = math.copysign(1, obj_vel.scalar_projection(crossing_point_1-obj_front_point)) 
            sign_1 = math.copysign(1, self_vel.scalar_projection(crossing_point_1-front_point))
        except: 
            sign_1 = -1
            obj_sign_1 = -1
        
        try: 
            obj_sign_2 = math.copysign(1, obj_vel.scalar_projection(crossing_point_2-obj_front_point))
            sign_2 = math.copysign(1, self_vel.scalar_projection(crossing_point_2-front_point))
        except: 
            obj_sign_2 = -1
            sign_2 = -1
        
        dist_1 = front_point.distance_between_points(crossing_point_1)
        dist_2 = front_point.distance_between_points(crossing_point_2)
        
        obj_dist_1 = obj_front_point.distance_between_points(crossing_point_1)
        obj_dist_2 = obj_front_point.distance_between_points(crossing_point_2)
        
        if sign_1 < 0 or obj_sign_1 < 0: dist_1 = math.inf    #or obj_dist_1 > dist_1: dist_1 = math.inf 
        if sign_2 < 0 or obj_sign_2 < 0: dist_2 = math.inf   #or obj_dist_2 > dist_2: dist_2 = math.inf
        
        return dist_1, dist_2, (crossing_point_1, crossing_point_2)
        

class Bot:
    def __init__(self, id:int, path:list[list[Point, float]] = []):
        self.id = id
        self.point = Point(0,0)
        self.theta = 0
        self.obj = Object(self.point, self.theta, [Point(210,160),Point(-210,160),Point(-210,-160),Point(210,-160)], Vector(0,1))
        self.path = path
        self.path_index = 0
        self.speed = 0
        self.collision_points = {}
    
    def update(self, point, theta):
        self.point = point
        self.theta = theta
        self.obj.pos = point
        self.obj.direction = theta
    
    def load_point(self, n):
        self.path_index = n
        point = self.path[n][0]
        theta = self.path[n][1]
        self.speed = self.path[n][2]
        self.update(point, theta)
        
    def dist_to_collision(self, bot):
        dist, collision_points = self.obj.collision_course(bot.obj)
        if dist == None: dist = math.inf 
        dist1, dist2, moving_collision_points = self.obj.moving_collision_course(bot.obj)
        dist_list = [dist1, dist2]
        dist_list.sort()


        if not bot.id in self.collision_points.keys():
            self.collision_points[bot.id] = {}
        
        if not (collision_points[0] == None):
        
            self.collision_points[bot.id]['front'] = collision_points[0]
            
            if dist <= dist1 and dist <= dist2:
                if not collision_points[1] == None:
                    self.collision_points[bot.id]['collision_course'] = collision_points[1]
                else:
                    self.collision_points[bot.id]['collision_course'] = self.collision_points[bot.id]['front']
                self.collision_points[bot.id]['moving_collision_course_1'] = self.collision_points[bot.id]['front']
                self.collision_points[bot.id]['moving_collision_course_2'] = self.collision_points[bot.id]['front']
                
            elif dist1 < dist and dist1 <= dist2:
                if not moving_collision_points[0] == None:
                    self.collision_points[bot.id]['moving_collision_course_1'] = moving_collision_points[0]
                else:
                    self.collision_points[bot.id]['moving_collision_course_1'] = self.collision_points[bot.id]['front']
                self.collision_points[bot.id]['collision_course'] = self.collision_points[bot.id]['front']
                self.collision_points[bot.id]['moving_collision_course_2'] = self.collision_points[bot.id]['front']
            
            elif dist2 < dist and dist2 < dist1:  
                if not moving_collision_points[1] == None:
                    self.collision_points[bot.id]['moving_collision_course_2'] = moving_collision_points[1]
                else:
                    self.collision_points[bot.id]['moving_collision_course_2'] = self.collision_points[bot.id]['front']
                self.collision_points[bot.id]['collision_course'] = self.collision_points[bot.id]['front']
                self.collision_points[bot.id]['moving_collision_course_1'] = self.collision_points[bot.id]['front']
        else:
            self.collision_points[bot.id]['front'] = Point(0, 0)
            self.collision_points[bot.id]['collision_course'] = Point(0, 0)
            self.collision_points[bot.id]['moving_collision_course_1'] = Point(0, 0)
            self.collision_points[bot.id]['moving_collision_course_2'] = Point(0, 0)
            
        return dist, dist_list[0]
    
    
    
class Intersection:
    class Intersection_section:
        def __init__(self, outer, center_point:Point, dx:float, dy:float, n:int, theta:float):
            self.n = n
            self.outer = outer
            self.center_point = center_point
            self.obj = Object(self.center_point, theta, [Point(-dx/2,-dy/2),Point(-dx/2,dy/2),Point(dx/2,dy/2),Point(dx/2,-dy/2)])

    def __init__(self, name:str, p1:Point, p2:Point, theta:float = 0, parts_in_x:int = 2, parts_in_y:int = 2, bot_range:float = None):
        self.name = name
        self.range = bot_range if not bot_range == None else max(abs(p1.x-p2.x),abs(p1.y-p2.y))*2
        self.p1 = p1
        self.p2 = p2
        self.center_point = (p1+p2)*0.5
        dx = abs((p2-p1).x)
        dy = abs((p2-p1).y)
        self.parts = []
        self.obj = Object(self.center_point, theta, [Point(-dx/2,-dy/2),Point(-dx/2,dy/2),Point(dx/2,dy/2),Point(dx/2,-dy/2)])
        
        part_x_length = abs((p2-p1).x)/parts_in_x
        part_y_length = abs((p2-p1).y)/parts_in_y
        
        for y in range(parts_in_y):
            for x in range(parts_in_x):
                n = int(x + y*parts_in_x)
                part_n_center_point = self.center_point + Point(-abs((p2-p1).x)/2+(part_x_length)*(x+0.5), -abs((p2-p1).y)/2+(part_y_length)*(y+0.5))
                self.parts.append(self.Intersection_section(self, part_n_center_point, part_x_length, part_y_length, n, theta))



    