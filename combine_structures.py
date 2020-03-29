
from random import randint
from random import uniform
from random import shuffle
from random import choice
from copy import deepcopy
import itertools
import os
from math import sqrt, ceil, atan, atan2, cos, sin, pi, degrees, radians, tan

# blocks number and size
blocks = {'1':[0.84,0.84], '2':[0.85,0.43], '3':[0.43,0.85], '4':[0.43,0.43],
          '5':[0.22,0.22], '6':[0.43,0.22], '7':[0.22,0.43], '8':[0.85,0.22],
          '9':[0.22,0.85], '10':[1.68,0.22], '11':[0.22,1.68],
          '12':[2.06,0.22], '13':[0.22,2.06],
          '14':[0.82,0.82],'15':[0.82,0.82],'16':[0.8,0.8],'17':[0.45,0.45]}

# blocks number and name
# (blocks 3, 7, 9, 11 and 13) are their respective block names rotated 90 derees clockwise
# blocks 3, 7, 9, 11 and 13 are vertical blocks
block_names = {'1':"SquareHole", '2':"RectFat", '3':"RectFat", '4':"SquareSmall",
               '5':"SquareTiny", '6':"RectTiny", '7':"RectTiny", '8':"RectSmall",
               '9':"RectSmall",'10':"RectMedium",'11':"RectMedium",
               '12':"RectBig",'13':"RectBig", '14':"TriangleHole",
               '15':"Triangle", '16':"Circle", '17':"CircleSmall"}

# additional objects number and name
additional_objects = {'1':"TriangleHole", '2':"Triangle", '3':"Circle", '4':"CircleSmall"}

# additional objects number and size
additional_object_sizes = {'1':[0.82,0.82],'2':[0.82,0.82],'3':[0.8,0.8],'4':[0.45,0.45]}

# blocks number and probability of being selected
probability_table_blocks = {'1':0.10, '2':0.10, '3':0.10, '4':0.05,
                            '5':0.02, '6':0.05, '7':0.05, '8':0.10,
                            '9':0.05, '10':0.16, '11':0.04,
                            '12':0.16, '13':0.02}

probability_table_blocks2 = {'1':0, '2':0.1, '3':0, '4':0,
                            '5':0, '6':0, '7':0, '8':0.3,
                            '9':0, '10':0.3, '11':0,
                            '12':0.3, '13':0}

probability_table_roof = {'1':0, '2':0, '3':0, '4':0,
                          '5':0, '6':0, '7':0, '8':0.2,
                          '9':0, '10':0.4, '11':0,
                          '12':0.4, '13':0}

probability_table_internal = {'1':0.11, '2':0, '3':0.3, '4':0.15,
                          '5':0.03, '6':0.03, '7':0.12, '8':0,
                          '9':0.2, '10':0, '11':0.03,
                          '12':0, '13':0.03}

# probability of roof to choose different material or not (just the degree of willing)
probability_table_roof_type = {'1':0.7, '2':0.3}

probability_table_internal_sp = {'1':0.3, '2':0.3, '3':0.3, '4':0}

# materials that are available
materials = ["wood", "stone", "ice"]

# bird types number and name
bird_names = {'1':"BirdRed", '2':"BirdBlue", '3':"BirdYellow", '4':"BirdBlack", '5':"BirdWhite"}

# bird types number and probability of being selected
bird_probabilities = {'1': 0, '2': 0.3, '3': 0.3, '4': 0.4, '5': 0}

TNT_block_probability = 0.3

pig_size = [0.5,0.5]    # size of pigs

platform_size = [0.62,0.62]     # size of platform sections

edge_buffer = 0.11      # buffer uesd to push edge blocks further into the structure center (increases stability)

absolute_ground = -3.5          # the position of ground within level

minimum_height_gap = 3.5        # y distance min between platforms
platform_distance_buffer = 0.4  # x_distance min between platforms / y_distance min between platforms and ground structures

# defines the levels area (ie. space within which structures/platforms can be placed)
level_width_min = -3.0
level_width_max = 9.0
level_height_min = -1.0         # only used by platforms, ground structures use absolute_ground to determine their lowest point
level_height_max = 6.0

pig_precision = 0.01                # how precise to check for possible pig positions on ground

min_ground_width = 2.5                      # minimum amount of space allocated to ground structure
ground_structure_height_limit = ((level_height_max - minimum_height_gap) - absolute_ground)/1.5    # desired height limit of ground structures

# used for trajectory estimation and identifying reachable blocks
trajectory_accuracy = 0.5
number_shots = 50
slingshot_x = -7.7
slingshot_y = -1.0
MAX_X = 20
launchAngle =   [0.13,  0.215, 0.296, 0.381, 0.476, 0.567, 0.657, 0.741, 0.832, 0.924, 1.014, 1.106, 1.197]
changeAngle =   [0.052, 0.057, 0.063, 0.066, 0.056, 0.054, 0.050, 0.053, 0.042, 0.038, 0.034, 0.029, 0.025]
launchVelocity = [2.9,   2.88,  2.866, 2.838, 2.810, 2.800, 2.790, 2.773, 2.763, 2.745, 2.74, 2.735, 2.73]
scale = 1.0
scaleFactor = 1.65

# used when adding hills/slopes
max_slope_angle = 30.
max_slope_height = 1.5
max_slope_increase = 1.0
add_slopes = True


def get_release_angle(release_y):
    """get release angle"""
    release_X = -100
    refX, refY = 191, 344
    return -atan2(refY - int(release_y), refX - release_X)


def launchToActual(theta):
    i = 1
    while (i < len(launchAngle)):
        if (theta > launchAngle[i - 1] and theta < launchAngle[i]):
            return theta + changeAngle[i - 1]
        i = i + 1
    return theta + changeAngle[len(launchAngle) - 1]


def getVelocity(theta):
    if (theta < launchAngle[0]):
        return scaleFactor * launchVelocity[0]
    i = 1
    while (i < len(launchAngle)):
        if (theta < launchAngle[i]):
            return scaleFactor * launchVelocity[i - 1]
        i = i + 1
    return scaleFactor * launchVelocity[len(launchVelocity) - 1]


def find_trajectory(release_x, release_y):
    theta = atan2(release_y, release_x)
    theta = launchToActual(theta)
    velocity = getVelocity(theta)
    ux = velocity * cos(theta)
    uy = velocity * sin(theta)
    a = -0.5 / (ux * ux)
    b = uy / ux
    x = 0.0
    trajectory = []
    while (x < MAX_X):
        xn = x * scale
        y = (a * xn * xn + b * xn) * scale
        trajectory.append([round(x, 10), round(y, 10)])
        x = x + trajectory_accuracy
    return trajectory


def find_release_point(theta):
    release = [(-100.0 * cos(theta)), (-100.0 * sin(theta))]
    return release






def read_level_property(file_name):
    property = {}
    width = float(file_name.readline())
    center = float(file_name.readline())
    property["left_most"] = round(center - width/2, 10)
    property["right_most"] = round(center + width/2, 10)
    height = float(file_name.readline())
    ground = float(file_name.readline())
    property["highest"] = round(ground + height, 10)
    property["lowest"] = ground
    file_name.close()
    return property

def read_level_xml(file_name):
    item_lines = []
    birds_lines = []
    birds_start = 0
    start = 0
    line = file_name.readline()
    while line:
        line = file_name.readline()

        if "<Birds>" in line:
            birds_start = 1
            continue
        if "</Birds>" in line:
            birds_start = 0
        if "<GameObjects>" in line:
            start = 1
            continue
        if "</GameObjects>" in line:
            start = 0
            break

        if start:
            item_lines.append(line)
        if birds_start:
            birds_lines.append(line)
    file_name.close()
    return item_lines

def read_test_result(file_name):
    line = file_name.readline()
    bird_type = line.split(" ")[0]
    shoot_angles = []
    while line:
        shoot_angles.append(line.split(" ")[1])
        line = file_name.readline()
    file_name.close()
    return bird_type, shoot_angles


def write_level_xml(current_level, buildings, levels_birds):
    level_number = str(current_level).zfill(2)
    f = open("combined/level-" + level_number + ".xml", "w")

    f.write('<?xml version="1.0" encoding="utf-16"?>\n')
    f.write('<Level width ="2">\n')
    f.write('<Camera x="0" y="2" minWidth="20" maxWidth="30">\n')
    f.write('<Score highScore ="89500">\n')
    f.write('<Birds>\n')

    for birds in levels_birds:  # order of birds may be changed later
        for bird in birds:
            f.write('<Bird type="%s"/>\n' % bird_names[str(int(bird)+1)])
            if uniform(0.0, 1.0) < 0.4:
                f.write('<Bird type="%s"/>\n' % bird_names[str(randint(2,5))])

    f.write('</Birds>\n')
    f.write('<Slingshot x="-8" y="-2.5">\n')
    f.write('<GameObjects>\n')

    for building in buildings:
        for j in building:
            f.write(j)

    f.write('</GameObjects>\n')
    f.write('</Level>\n')

    f.close()

    print(level_number)


def no_overlap(current_level, previous_levels):
    overlap = False
    for i in previous_levels:
        if i["lowest"] - 2.0 > current_level["highest"] or current_level["lowest"] - 2.0 > i["highest"]:
            # print("no overlap on y: ")
            # print(current_level)
            # print(i)
            continue
        # for later, we know there are overlap on y axis
        if (current_level["left_most"] <= i["left_most"] <= current_level["right_most"] + 1) or \
           (i["left_most"] <= current_level["left_most"] <= i["right_most"] + 1):
            overlap = True
            # print("detect overlap")
            break
    return not overlap


def trajectory_analysis(target_level_angles, possible_block_level):
    overlap = True
    for i in target_level_angles:
        t_overlap = False
        angle = get_release_angle(i)
        release_point = find_release_point(angle)
        trajectory = find_trajectory(release_point[0], release_point[1])

        for point in trajectory:
            point[0] = round(point[0] + slingshot_x, 10)
            point[1] = round(point[1] + slingshot_y, 10)

        for j in range(len(trajectory)):
            point = trajectory[j]
            if j > 0:
                assert trajectory[j][0] > trajectory[j-1][0]
            if possible_block_level["left_most"] < point[0] < possible_block_level["right_most"] and \
                    possible_block_level["lowest"] < point[0] < possible_block_level["highest"]:
                t_overlap = True
                break
        if not t_overlap:
            overlap = False
            break
    return overlap


for i in range(1, 50):
    original_level_number = []
    structures_property = []
    levels = []
    birds = []
    j = 0
    tries = 0

    good_buildings = "good buildings"
    good_buildings_number = []
    all_files = []
    for parent, dirnames, filenames in os.walk(good_buildings):
        all_files = filenames
    for filename in all_files:
        if ".xml" in filename:
            good_buildings_number.append(filename.split("-")[1].split(".")[0])

    new_level_number = i
    number_of_buildings = randint(2,4)
    shoot_angle = []
    while j < number_of_buildings:
        level_number = choice(good_buildings_number)
        f_level = open("good buildings/level-"+level_number+".xml", "r")
        f_property = open("good buildings/level-"+level_number+".txt", "r")
        if level_number[0] == '0':
            f_bird = open("good buildings/level_result-"+level_number[1]+".txt", "r")
        else:
            f_bird = open("good buildings/level_result-"+level_number+".txt", "r")
        item_lines = read_level_xml(f_level)
        level_property = read_level_property(f_property)
        bird_type, shooting_angles = read_test_result(f_bird)
        if no_overlap(level_property, structures_property) and (len(structures_property) == 0 or not trajectory_analysis(shooting_angles, structures_property[-1])):
            structures_property.append(level_property)
            levels.append(item_lines)
            birds.append(bird_type)
            original_level_number.append(level_number)
            shoot_angle.append(shooting_angles)
            j += 1
        tries += 1
        if tries > 1000:
            original_level_number = []
            structures_property = []
            levels = []
            birds = []
            j = 0
            tries = 0

    write_level_xml(new_level_number, levels, birds)
    print(original_level_number)
    print(shoot_angle)

