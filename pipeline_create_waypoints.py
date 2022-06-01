import sys, math, os, yaml
import matplotlib.pyplot as plt
import numpy as np


# CONSTANTS:

DESIRED_VEL = 0.4
TIME_STEP = 0.2


# CLASSES:

class Path():
    def __init__(self, filename, res):
            # create path with filename
            self.filename = filename
            self.vals = self.create_path()

            self.x_vals = [i[0] for i in self.vals]             # these are used for graphing later
            self.y_vals = [i[1] for i in self.vals]

            # creating more data points:
            self.add_points(res)                                   # updates self.vals

            # creating waypoints:
            self.waypoints = WaypointList(self.vals)

    def create_path(self):
        flat_path = np.loadtxt(self.filename)
        size_x = int(flat_path.shape[0]/2)
        size_y = 2
        path = np.asarray(flat_path).reshape(size_x, size_y)
        return path.tolist()

    def add_points(self, res):
        new_arr = []
        for i in range(len(self.vals)):
            if i == len(self.vals) - 1:                  # adding final point
                new_arr.append(self.vals[i])
            else:
                temp_arr = self.calc_points(self.vals[i], self.vals[i+1], res)
                new_arr = new_arr + temp_arr
        self.vals = new_arr

    @staticmethod
    def calc_points(start, end, resolution):
        # variables:
        points = []
        x1, y1 = start
        x2, y2 = end

        # determining number of points being added between given coords
        distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)*resolution              # in metres? #TODO: resolution should be a param in this equation

        if distance <= DESIRED_VEL*TIME_STEP:                               # if only original point must be added
            num_added_points = 1                                            # accounts for addition of original point (x1, y1)
        else:
            num_added_points = round((distance/DESIRED_VEL)/TIME_STEP)      # accounts for addition of new points and original point (x1, y1)

        # creating arrays:
        if x1 != x2:                                # slope will be defined 
            # calculating slope and b:
            slope = (y2 - y1)/(x2 - x1)
            b = y1 - slope*x1

            # determining step:
            step = (x2-x1)/(num_added_points)           # divides space so that original point (x1, y1) and new points are all added 

            # adding values:
            for i in np.arange(x1, x2, step):
                points.append([i, (slope*i) + b])

        elif y1 != y2:                                       # slope will be undefined
            # determining step:
            step = (y2-y1)/(num_added_points)           # divides space so that original point (x1, y1) and new points are all added 

            # adding values:
            for i in np.arange(y1, y2, step):
                points.append([x1, i])

        return points


class WaypointList():

    class Waypoint():
        def __init__(self, coords):
            self.x = coords[0]
            self.y = coords[1]
            self.theta = 0
            self.theta_dot = 0
            self.dy = 0
            self.dx = 0

    def __init__(self, vals):
        self.list = []
        for i in range(len(vals)):
            self.list.append(self.Waypoint(vals[i]))
        self.remove_duplicates()
        self.calc_thetas()
        self.calc_theta_dot()
        self.calc_dx_dy()

    def remove_duplicates(self):
        skipped = 0
        for i in range(len(self.list)-1):
            curr_point, next_point = self.list[i-skipped], self.list[i+1-skipped]
            if curr_point.x == next_point.x and curr_point.y == next_point.y:
                self.list.pop(i-skipped)
                skipped += 1

    def calc_thetas(self):
        for i in range(len(self.list)):
            if i == len(self.list) - 1:     # i.e. no next point
                self.list[i].theta = self.list[i-1].theta
                break
            else:
                curr_point, next_point = self.list[i], self.list[i+1]
                x1, y1 = curr_point.x, curr_point.y
                x2, y2 = next_point.x, next_point.y
                if x1 == x2:
                    if y2 > y1:                                 # note this might not work when y axis is reversed
                        theta = math.radians(-90)
                        # theta = math.radians(90)
                    else:
                        theta = math.radians(90)
                        # theta = math.radians(-90)
                else:

                    theta = -math.atan((y2-y1)/(x2-x1))
                    # theta = math.atan((y2-y1)/(x2-x1))

                    if x1>x2:                   # converting from between -90 and 90 to between -180 and 180 (depending on direction of new point on x-axis)
                        ogTheta = theta
                        theta += 3.14
                        if theta > 3.14:
                            theta = -3.14 + ogTheta


                curr_point.theta = theta

    def calc_theta_dot(self):
        for i in range(1, len(self.list)):
            curr_point, prev_point = self.list[i], self.list[i-1]

            theta_dot = (curr_point.theta - prev_point.theta)/TIME_STEP
            curr_point.theta_dot = theta_dot

    def calc_dx_dy(self):
        for waypoint in self.list:
            waypoint.dx = DESIRED_VEL*math.cos(waypoint.theta)
            waypoint.dy = DESIRED_VEL*math.sin(waypoint.theta)


# MAIN:

def main():

    if len(sys.argv) != 4:
        print(str(sys.argv[0]) + " requires 3 arguments --> path_file map_file yaml_file")
        exit()

    # variables:
    path_file = sys.argv[1]
    map_file = sys.argv[2]
    yaml_file = sys.argv[3]

    # get map_size info
    with open(map_file) as file:
        vals = file.readline().rstrip().rsplit(' ')                     # gets string array of [size_x, size_y]
        map_size = (int(vals[0]), int(vals[1]))

    # load map:
    flat_map = np.loadtxt(map_file, skiprows=2)                         # skips line with map size
    map = np.asarray(flat_map).reshape(map_size[1], map_size[0])

    # load yaml:
    with open(yaml_file, "r") as file:
        try:
            yaml_dict = yaml.safe_load(file)
            offset = yaml_dict["origin"]
            resolution = yaml_dict["resolution"]

        except yaml.YAMLError as exc:
            print(exc)

    # determine offset values:
    offset_x = -offset[0]/resolution
    offset_y = map_size[1] + offset[1]/resolution

    # create waypoints folder:
    dir_name = "waypoints/"
    if not os.path.exists(dir_name):                                    # if waypoint folder doesn't exist, create it
        os.mkdir(dir_name)
    dir_name = str(dir_name)+str(map_file.rsplit('/', 1)[1].rsplit('.', 1)[0])+"/"               
    if not os.path.exists(dir_name):                                    # if waypoint folder for this map doesn't exist, create it
        os.mkdir(dir_name)

    # load path:
    path = Path(path_file, resolution)

    # create waypoints file:
    file_name = str(dir_name)+"waypoints.csv"
    file = open(file_name, "w")
    file.write("x,y,theta,x_dot,y_dot,theta_dot\n")
    for j in path.waypoints.list:

        # writing waypoint data to file
        file.write(str(round(resolution*(j.x-offset_x), 4))+","+str(round(resolution*(offset_y-j.y), 4))+","+str(round(j.theta, 4))+","+str(round(j.dx, 4))+","+str(round(j.dy, 4))+",0\n")
    
    file.close()

    # saving image of path:
    plt.imshow(map)
    plt.scatter(path.x_vals, path.y_vals)
    plt.savefig(str(dir_name)+"waypoints.png", bbox_inches='tight')
    plt.clf()                                                           # clears current figure

    # print(f"Created {len(path.waypoints.list)} waypoints.")
    
    # # displaying paths
    # plt.imshow(map)
    # plt.plot(path.x_vals, path.y_vals)
    # plt.show()

    sys.stdout.write(str(file_name)+" "+str(TIME_STEP))

if __name__ == '__main__':
    main()
