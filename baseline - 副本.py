
from random import randint
from random import uniform
from random import shuffle
from copy import deepcopy
import itertools
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
bird_probabilities = {'1': 0, '2': 0, '3': 1, '4': 0, '5': 0}

TNT_block_probability = 0.3

pig_size = [0.5,0.5]    # size of pigs

platform_size = [0.62,0.62]     # size of platform sections

edge_buffer = 0.11      # buffer uesd to push edge blocks further into the structure center (increases stability)

absolute_ground = -3.5          # the position of ground within level

max_peaks = 5           # maximum number of peaks a structure can have (up to 5)
min_peak_split = 10     # minimum distance between two peak blocks of structure
max_peak_split = 50     # maximum distance between two peak blocks of structure

minimum_height_gap = 3.5        # y distance min between platforms
platform_distance_buffer = 0.4  # x_distance min between platforms / y_distance min between platforms and ground structures

# defines the levels area (ie. space within which structures/platforms can be placed)
level_width_min = -3.0
level_width_max = 9.0
level_height_min = -2.0         # only used by platforms, ground structures use absolute_ground to determine their lowest point
level_height_max = 6.0

pig_precision = 0.01                # how precise to check for possible pig positions on ground

min_ground_width = 2.5                      # minimum amount of space allocated to ground structure
ground_structure_height_limit = ((level_height_max - minimum_height_gap) - absolute_ground)/1.5    # desired height limit of ground structures

max_attempts = 100                          # number of times to attempt to place a platform before abandoning it

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

# these functions are all usd by the trajectory estimator (please don't change)

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


#ccw -> counter-clockwise
def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
# determines if two lines intersect
def line_intersects_line(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
# determines if the line formed by two points intersects block
def line_intersects_block(point1, point2, block):
    return (line_intersects_line(point1, point2, [block[1] - (blocks[str(block[0])][0] / 2.0),
                                                  block[2] - (blocks[str(block[0])][1] / 2.0)],
                                 [block[1] + (blocks[str(block[0])][0] / 2.0),
                                  block[2] - (blocks[str(block[0])][1] / 2.0)]) or
            line_intersects_line(point1, point2, [block[1] + (blocks[str(block[0])][0] / 2.0),
                                                  block[2] - (blocks[str(block[0])][1] / 2.0)],
                                 [block[1] + (blocks[str(block[0])][0] / 2.0),
                                  block[2] + (blocks[str(block[0])][1] / 2.0)]) or
            line_intersects_line(point1, point2, [block[1] + (blocks[str(block[0])][0] / 2.0),
                                                  block[2] + (blocks[str(block[0])][1] / 2.0)],
                                 [block[1] - (blocks[str(block[0])][0] / 2.0),
                                  block[2] + (blocks[str(block[0])][1] / 2.0)]) or
            line_intersects_line(point1, point2, [block[1] - (blocks[str(block[0])][0] / 2.0),
                                                  block[2] + (blocks[str(block[0])][1] / 2.0)],
                                 [block[1] - (blocks[str(block[0])][0] / 2.0),
                                  block[2] - (blocks[str(block[0])][1] / 2.0)]))


#####################################################################


# generates a list of all possible subsets for structure bottom

def generate_subsets(current_tree_bottom):     
    current_distances = []
    subsets = []
    current_point = 0
    while current_point < len(current_tree_bottom)-1:
        current_distances.append(current_tree_bottom[current_point+1][1] - current_tree_bottom[current_point][1])
        current_point = current_point + 1

    # remove similar splits causesd by floating point imprecision
    for i in range(len(current_distances)):
        current_distances[i] = round(current_distances[i],10)

    split_points = list(set(current_distances))         # all possible x-distances between bottom blocks

    for i in split_points:      # subsets based on differences between x-distances
        current_subset = []
        start_point = 0
        end_point = 1
        for j in current_distances:
            if j >= i:
                current_subset.append(current_tree_bottom[start_point:end_point])
                start_point = end_point
            end_point = end_point + 1

        current_subset.append(current_tree_bottom[start_point:end_point])

        subsets.append(current_subset)

    subsets.append([current_tree_bottom])

    return subsets




# finds the center positions of the given subset

def find_subset_center(subset):
    if len(subset)%2 == 1:
        return subset[(len(subset)-1)//2][1]
    else:
        return (subset[len(subset)//2][1] - subset[(len(subset)//2)-1][1])/2.0 + subset[(len(subset)//2)-1][1]




# finds the edge positions of the given subset

def find_subset_edges(subset):
    edge1 = subset[0][1] - (blocks[str(subset[0][0])][0])/2.0 + edge_buffer
    edge2 = subset[-1][1] + (blocks[str(subset[-1][0])][0])/2.0 - edge_buffer
    return[edge1,edge2]




# checks that positions for new block dont overlap and support the above blocks

def check_valid(grouping,choosen_item,current_tree_bottom,new_positions):

    # check no overlap
    i = 0
    while i < len(new_positions)-1:
        if (new_positions[i] + (blocks[str(choosen_item)][0])/2) > (new_positions[i+1] - (blocks[str(choosen_item)][0])/2):
            return False
        i = i + 1

    # check if each structural bottom block's edges supported by new blocks
    for item in current_tree_bottom:
        edge1 = item[1] - (blocks[str(item[0])][0])/2
        edge2 = item[1] + (blocks[str(item[0])][0])/2
        edge1_supported = False
        edge2_supported = False
        for new in new_positions:
            if ((new - (blocks[str(choosen_item)][0])/2) <= edge1 and (new + (blocks[str(choosen_item)][0])/2) >= edge1):
                edge1_supported = True
            if ((new - (blocks[str(choosen_item)][0])/2) <= edge2 and (new + (blocks[str(choosen_item)][0])/2) >= edge2):
                edge2_supported = True
        if edge1_supported == False or edge2_supported == False:
                return False
    return True


def check_no_overlap(new_positions):
    i = 0
    while i < len(new_positions) - 1:
        if round((new_positions[i][1] + (blocks[str(new_positions[i][0])][0]) / 2) - 0.01, 10) > (
                round(new_positions[i + 1][1] - (blocks[str(new_positions[i+1][0])][0]) / 2, 10)):
            print(new_positions)
            print(new_positions[i][1] + (blocks[str(new_positions[i][0])][0]) / 2)
            print(new_positions[i + 1][1] - (blocks[str(new_positions[i+1][0])][0]) / 2)
            return False
        i = i + 1
    return True


# check if new block can be placed under center of bottom row blocks validly

def check_center(grouping,choosen_item,current_tree_bottom):
    new_positions = []
    for subset in grouping:
        new_positions.append(find_subset_center(subset))
    return check_valid(grouping,choosen_item,current_tree_bottom,new_positions)




# check if new block can be placed under edges of bottom row blocks validly

def check_edge(grouping,choosen_item,current_tree_bottom):
    new_positions = []
    for subset in grouping:
        new_positions.append(find_subset_edges(subset)[0])
        new_positions.append(find_subset_edges(subset)[1])
    return check_valid(grouping,choosen_item,current_tree_bottom,new_positions)




# check if new block can be placed under both center and edges of bottom row blocks validly

def check_both(grouping,choosen_item,current_tree_bottom):
    new_positions = []
    for subset in grouping:
        new_positions.append(find_subset_edges(subset)[0])
        new_positions.append(find_subset_center(subset))
        new_positions.append(find_subset_edges(subset)[1])
    return check_valid(grouping,choosen_item,current_tree_bottom,new_positions)




# choose a random item/block from the blocks dictionary based on probability table

def choose_item(table):
    ran_num = uniform(0.0,1.0)
    selected_num = 0
    while ran_num > 0:
        selected_num = selected_num + 1
        ran_num = ran_num - table[str(selected_num)]
    return selected_num




# finds the width of the given structure

def find_structure_width(structure):
    min_x = 999999.9
    max_x = -999999.9
    for block in structure:
        if round((block[1]-(blocks[str(block[0])][0]/2)),10) < min_x:
            min_x = round((block[1]-(blocks[str(block[0])][0]/2)),10)
        if round((block[1]+(blocks[str(block[0])][0]/2)),10) > max_x:
            max_x = round((block[1]+(blocks[str(block[0])][0]/2)),10)
    return (round(max_x - min_x,10))



   
# finds the height of the given structure

def find_structure_height(structure):
    min_y = 999999.9
    max_y = -999999.9
    for block in structure:
        if round((block[2]-(blocks[str(block[0])][1]/2)),10) < min_y:
            min_y = round((block[2]-(blocks[str(block[0])][1]/2)),10)
        if round((block[2]+(blocks[str(block[0])][1]/2)),10) > max_y:
            max_y = round((block[2]+(blocks[str(block[0])][1]/2)),10)
    return (round(max_y - min_y,10))




# adds a new row of blocks to the bottom of the structure

def add_new_row(current_tree_bottom, total_tree, block_type=0, recu_time=0):

    groupings = generate_subsets(current_tree_bottom)   # generate possible groupings of bottom row objects
    if block_type == 0:
        choosen_item = choose_item(probability_table_blocks)# choosen block for new row
    else:
        choosen_item = choose_item(probability_table_blocks2)
    center_groupings = []                               # collection of viable groupings with new block at center
    edge_groupings = []                                 # collection of viable groupings with new block at edges
    both_groupings = []                                 # collection of viable groupings with new block at both center and edges
    
    # check if new block is viable for each grouping in each position
    for grouping in groupings:
        if check_center(grouping,choosen_item,current_tree_bottom):             # check if center viable
            center_groupings.append(grouping)
        if check_edge(grouping,choosen_item,current_tree_bottom):               # check if edges viable
            edge_groupings.append(grouping)
        if check_both(grouping,choosen_item,current_tree_bottom):               # check if both center and edges viable
            both_groupings.append(grouping)

    # randomly choose a configuration (grouping/placement) from the viable options
    total_options = len(center_groupings) + len(edge_groupings) + len(both_groupings)   #total number of options
    if total_options > 0:
        option = randint(1,total_options)
        if option > len(center_groupings) + len(edge_groupings):
            selected_grouping = both_groupings[option- (len(center_groupings) + len(edge_groupings) + 1)]
            placement_method = 2
        elif option > len(center_groupings):
            selected_grouping = edge_groupings[option- (len(center_groupings) + 1)]
            placement_method = 1
        else:
            selected_grouping = center_groupings[option-1]
            placement_method = 0

        # construct the new bottom row for structure using selected block/configuration
        new_bottom = []
        for subset in selected_grouping:
            if placement_method == 0:
                new_bottom.append([choosen_item, find_subset_center(subset)])
            if placement_method == 1:
                new_bottom.append([choosen_item, find_subset_edges(subset)[0]])
                new_bottom.append([choosen_item, find_subset_edges(subset)[1]])
            if placement_method == 2:
                new_bottom.append([choosen_item, find_subset_edges(subset)[0]])
                new_bottom.append([choosen_item, find_subset_center(subset)])
                new_bottom.append([choosen_item, find_subset_edges(subset)[1]])

        for i in new_bottom:
            i[1] = round(i[1], 10)      # round all values to prevent floating point inaccuracy from causing errors

        current_tree_bottom = new_bottom
        total_tree.append(current_tree_bottom)      # add new bottom row to the structure
        return total_tree, current_tree_bottom      # return the new structure
    
    else:
        if recu_time > 100 and block_type != 0:
            return False, False
        return add_new_row(current_tree_bottom, total_tree, block_type, recu_time+1) # choose a new block and try again if no options available


def add_yellow_bird_new_row(current_tree_bottom, total_tree):
    groupings = generate_subsets(
        current_tree_bottom)  # generate possible groupings of bottom row objects, this is the previous row
    choosen_item = 3  # choosen block for new row
    center_groupings = []  # collection of viable groupings with new block at center
    edge_groupings = []  # collection of viable groupings with new block at edges
    both_groupings = []  # collection of viable groupings with new block at both center and edges

    # check if new block is viable for each grouping in each position
    for grouping in groupings:
        if check_center(grouping, choosen_item, current_tree_bottom):  # check if center viable
            center_groupings.append(grouping)
        if check_edge(grouping, choosen_item, current_tree_bottom):  # check if edges viable
            edge_groupings.append(grouping)
        if check_both(grouping, choosen_item, current_tree_bottom):  # check if both center and edges viable
            both_groupings.append(grouping)

    # randomly choose a configuration (grouping/placement) from the viable options
    total_options = len(center_groupings) + len(edge_groupings) + len(both_groupings)  # total number of options
    if total_options > 0:
        option = randint(1, total_options)
        if option > len(center_groupings) + len(edge_groupings):
            selected_grouping = both_groupings[option - (len(center_groupings) + len(edge_groupings) + 1)]
            placement_method = 2
        elif option > len(center_groupings):
            selected_grouping = edge_groupings[option - (len(center_groupings) + 1)]
            placement_method = 1
        else:
            selected_grouping = center_groupings[option - 1]
            placement_method = 0

        # construct the new bottom row for structure using selected block/configuration
        new_bottom = []
        for subset in selected_grouping:
            if placement_method == 0:
                new_bottom.append([choosen_item, find_subset_center(subset)])
            if placement_method == 1:
                new_bottom.append([choosen_item, find_subset_edges(subset)[0]])
                new_bottom.append([choosen_item, find_subset_edges(subset)[1]])
            if placement_method == 2:
                new_bottom.append([choosen_item, find_subset_edges(subset)[0]])
                new_bottom.append([choosen_item, find_subset_center(subset)])
                new_bottom.append([choosen_item, find_subset_edges(subset)[1]])

        for i in new_bottom:
            i[1] = round(i[1], 10)  # round all values to prevent floating point inaccuracy from causing errors

        if len(new_bottom) > 1:
            if (new_bottom[0][1] + 0.86) < new_bottom[1][1]:
                new_bottom.append([choosen_item, round(new_bottom[0][1] + 0.43, 10)])
                new_bottom = sorted(new_bottom, key=lambda block: block[1])
        else:
            new_bottom.append([choosen_item, round(new_bottom[0][1] + 0.43, 10)])
            new_bottom = sorted(new_bottom, key=lambda block: block[1])

        current_tree_bottom = new_bottom
        total_tree.append(current_tree_bottom)  # add new bottom row to the structure

        total_tree, current_tree_bottom = add_new_row(current_tree_bottom, total_tree, 2)
        if not total_tree:
            return False, False

        return total_tree, current_tree_bottom  # return the new structure
    else:
        return False, False


# creates the peaks (first row) of the structure

def make_peaks(center_point):

    current_tree_bottom = []        # bottom blocks of structure
    number_peaks = randint(1,max_peaks)     # this is the number of peaks the structure will have
    top_item = choose_item(probability_table_blocks)    # this is the item at top of structure

    if number_peaks == 1:
        current_tree_bottom.append([top_item,center_point])     

    if number_peaks == 2:
        distance_apart_extra = round(randint(min_peak_split,max_peak_split)/100.0,10)
        current_tree_bottom.append([top_item,round(center_point - (blocks[str(top_item)][0]*0.5) - distance_apart_extra,10)] )
        current_tree_bottom.append([top_item,round(center_point + (blocks[str(top_item)][0]*0.5) + distance_apart_extra,10)] )

    if number_peaks == 3:
        distance_apart_extra = round(randint(min_peak_split,max_peak_split)/100.0,10)
        current_tree_bottom.append([top_item,round(center_point - (blocks[str(top_item)][0]) - distance_apart_extra,10)] )
        current_tree_bottom.append([top_item,round(center_point,10)])
        current_tree_bottom.append([top_item,round(center_point + (blocks[str(top_item)][0]) + distance_apart_extra,10)] )

    if number_peaks == 4:
        distance_apart_extra = round(randint(min_peak_split,max_peak_split)/100.0,10)
        current_tree_bottom.append([top_item,round(center_point - (blocks[str(top_item)][0]*1.5) - (distance_apart_extra*2),10)] )
        current_tree_bottom.append([top_item,round(center_point - (blocks[str(top_item)][0]*0.5) - distance_apart_extra,10)] )
        current_tree_bottom.append([top_item,round(center_point + (blocks[str(top_item)][0]*0.5) + distance_apart_extra,10)] )
        current_tree_bottom.append([top_item,round(center_point + (blocks[str(top_item)][0]*1.5) + (distance_apart_extra*2),10)] )

    if number_peaks == 5:
        distance_apart_extra = round(randint(min_peak_split,max_peak_split)/100.0,10)
        current_tree_bottom.append([top_item,round(center_point - (blocks[str(top_item)][0]*2.0) - (distance_apart_extra*2),10)] )
        current_tree_bottom.append([top_item,round(center_point - (blocks[str(top_item)][0]) - distance_apart_extra,10)] )
        current_tree_bottom.append([top_item,round(center_point,10)])
        current_tree_bottom.append([top_item,round(center_point + (blocks[str(top_item)][0]) + distance_apart_extra,10)] )
        current_tree_bottom.append([top_item,round(center_point + (blocks[str(top_item)][0]*2.0) + (distance_apart_extra*2),10)] )
    return current_tree_bottom


def get_width(block_number):
    return round(blocks[str(block_number)][0], 10)

def make_bottom(center, width, strong = 1):
    total_building = []

    roof_material = choose_item(probability_table_roof)
    if width < 1:
        roof_material = 2
    roof_width = blocks[str(roof_material)][0]
    if roof_width < 1:
        row_material = choose_item(probability_table_internal)
        while blocks[str(roof_material)][0] - blocks[str(row_material)][0] < 0.05:
            row_material = choose_item(probability_table_internal)
    else:
        ran_num = uniform(0.0, 1.0)
        if ran_num > 0.95:  # will be hard, implmented later
            row_material = 14  # for special blocks
        else:
            row_material = choose_item(probability_table_internal)

    # add roof
    new_top = []
    roof_num = ceil(width / roof_width)
    half_rn, mod = divmod(roof_num, 2)
    if mod == 0:
        for i in range(roof_num):
            center_distance = i - half_rn + 1  # number of blocks between current and center
            current_posi = round((center + (center_distance * roof_width) - roof_width / 2), 10)
            new_top.append([roof_material, current_posi])
    else:
        for i in range(roof_num):
            center_distance = i - half_rn  # number of blocks between current and center
            current_posi = round((center + (center_distance * roof_width)), 10)
            new_top.append([roof_material, current_posi])

    # 3 methods in total, single support for weak, double for strong
    if strong == 1:  # strong is 1 means strong structure, else weak
        if roof_width < 1:
            craft_method = 2  # place bottom at 2 edges
        else:
            ran_num = uniform(0.0, 1.0)
            if ran_num > 0.5:
                craft_method = 2
            else:
                craft_method = 3  # place bottoms at two sides but not edge
    else:
        craft_method = 1
        if row_material > 13:
            craft_method = 2

    # add bottom
    new_bottom = []
    if craft_method == 2:
        new_bottom.append([row_material, round(new_top[0][1] - roof_width / 2, 10)])
    for i in new_top:
        if craft_method == 1:
            new_bottom.append([row_material, round(i[1], 10)])
        elif craft_method == 2:
            if new_bottom[-1][1] + blocks[str(new_bottom[-1][0])][0] - 0.1 < round(i[1] - roof_width / 2, 10):
                new_bottom.append([row_material, round(i[1] - roof_width / 2 + blocks[str(new_bottom[-1][0])][0]/2, 10)])
            new_bottom.append([row_material, round(i[1] + roof_width / 2, 10)])
        else:
            new_bottom.append([row_material, round(i[1] - roof_width / 4, 10)])
            new_bottom.append([row_material, round(i[1] + roof_width / 4, 10)])

    total_building.append(new_bottom)
    total_building.append(new_top)
    current_top = deepcopy(new_top)

    return total_building, current_top

# round problem not solved in this function

def add_new_building_row(pre_top, total_building, center, width, strong=1):

    pre_left = round(pre_top[0][1] - blocks[str(pre_top[0][0])][0]/2, 10)
    pre_right = round(pre_top[-1][1] + blocks[str(pre_top[-1][0])][0]/2, 10)

    start = 0
    pre_block_right = -999
    new_top = []
    roof_material = []
    repeat = 0

    # decide roof material combination
    # material in one roof can vary, whether fixed (1), 2 types or anything (3)
    # try to use as less type of material as possible
    div, mod = divmod(width, 0.85)
    div = int(div)
    if mod > 0.43:  # this means add one more block will not exceed the width too much
        ran_num = uniform(0.0, 1.0)
        if ran_num > 0.8:
            for i in range(div+1):
                roof_material.append(2)
            real_width = (div+1) * blocks[str(2)][0]
    if not roof_material:
        base_material = choose_item(probability_table_roof)
        width_copy = width
        while width_copy > 0:  # try to add block until just exceed
            next_material = base_material
            while blocks[str(next_material)][0] - width_copy > 0.45:
                next_material = next_material - 2
            width_copy = width_copy - blocks[str(next_material)][0]
            roof_material.append(next_material)
        real_width = width - width_copy
        # width_copy is the difference of the chosen result and the input (desired) width
        # should also equal to sum(blocks[str(roof_material)]) (not this but this meaning)
    shuffle(roof_material)

    # place roof
    new_left = round(center - real_width/2, 10)  # start point
    for i in roof_material:
        current_position = round(new_left + blocks[str(i)][0]/2, 10)
        new_top.append([i, current_position])
        new_left = new_left + blocks[str(i)][0]
    new_right = new_left  # end point
    new_left = round(center - real_width/2, 10)  # start point

    # add bottom
    new_bottom = []
    ## first_material, likely to be the same in the whole level
    first_material = choose_item(probability_table_internal)
    fm_width = blocks[str(first_material)][0]  # first_material width
    ## second material
    if first_material == 9:
        second_material = 3
    elif first_material == 5:
        second_material = 6
    elif first_material == 7:
        second_material = 4
    else:
        second_material = 0
    # second material is the back up wider block for first material in the situation that we need to support two blocks
    ## handle the first block to deal with wider roof
    if new_left < pre_left:
        # new roof's left side is "lefter" than the previous one
        start = 1
        # means we already handle the first roof block
        if (strong == 0 and get_width(first_material) > 0.25) or (get_width(first_material) >= get_width(new_top[0][0])):
            if new_top[0][1] < pre_left:
                return add_new_building_row(pre_top, total_building, center, width, strong)
            new_bottom.append([first_material, new_top[0][1]])
        else:
            if round(new_top[0][1] - blocks[str(first_material)][0]/2, 10) < pre_left:
                return add_new_building_row(pre_top, total_building, center, width, strong)
            new_bottom.append([first_material, round(new_top[0][1] - blocks[str(first_material)][0]/2, 10)])
            new_bottom.append([first_material, round(new_top[0][1] + blocks[str(first_material)][0]/2, 10)])
        pre_block_right = new_bottom[-1][1] + blocks[str(new_bottom[-1][0])][0] / 2
    ## decide the construction method of the lower level
    # 3 methods in total, single support for weak, double for strong
    if strong == 1:  # strong is 1 means strong structure, else weak
        ran_num = uniform(0.0, 1.0)
        if ran_num < 0.5:
            craft_method = 2  # place bottoms at two edges
        else:
            craft_method = 3  # place bottoms at two sides but not edge
        # if blocks[str(first_material)][0] < 0.25:
        #     craft_method = 3
        # let's do mutation in method 2
    else:
        ran_num = uniform(0.0, 1.0)
        if ran_num < 0.8:
            craft_method = 1
        else:
            craft_method = 2
            repeat = 1
    ## add the first block for method 2
    if craft_method == 2 and start == 0 and get_width(new_top[0][0]) >= get_width(first_material):
        new_bottom.append([first_material, round(new_top[0][1] - blocks[str(new_top[0][0])][0] / 2 + get_width(first_material)/2, 10)])
        pre_block_right = round(new_bottom[-1][1] + blocks[str(new_bottom[-1][0])][0]/2, 10)
    ## stop add bottom if already complete
    if start > len(new_top)-1:
        total_building.append(new_bottom)
        total_building.append(new_top)
        current_top = deepcopy(new_top)
        return total_building, current_top
    ## main add loop
    for i in range(start, len(new_top)-1):
        if new_top[i][0] == 6:
            if first_material == 1:
                current_material = 3
            else:
                current_material = first_material
            if craft_method == 1 or craft_method == 3:
                if pre_block_right > new_top[i][1]:
                    continue  # no need to add more blocks
                new_block_left = round(new_top[i][1] - blocks[str(current_material)][0] / 2, 10)
                if pre_block_right > new_block_left:  # overlap
                    new_bottom.append([current_material, round(pre_block_right + blocks[str(current_material)][0] / 2, 10)])
                else:
                    new_bottom.append([current_material, new_top[i][1]])
            else:
                # craft method = 2
                if pre_block_right >= round(new_top[i][1] + get_width(new_top[i][0])/2, 10):
                    continue
                new_block_left = round(new_top[i][1] + get_width(new_top[i][0])/2 - blocks[str(current_material)][0] / 2, 10)
                if pre_block_right > new_block_left:
                    new_bottom.append([current_material, round(pre_block_right + blocks[str(current_material)][0] / 2, 10)])
                else:
                    new_bottom.append([current_material, round(new_top[i][1] + blocks[str(new_top[i][0])][0] / 2, 10)])
            continue
        if craft_method == 1:  # only put one block under it
            # a condition that should be deal in somewhere else
            # blocks[str(new_top[i][0])][0] < blocks[str(first_material)][0]-0.02 or
            if pre_block_right > new_top[i][1]:
                continue  # no need to add more blocks
            new_block_left = new_top[i][1] - blocks[str(first_material)][0]/2
            if pre_block_right > new_block_left:  # overlap
                new_bottom.append([first_material, round(pre_block_right + blocks[str(first_material)][0]/2, 10)])
            else:
                if get_width(first_material) > 0.25:
                    new_bottom.append([first_material, new_top[i][1]])
                else:
                    random_num = uniform(0.0, 1.0)
                    if second_material != 0 and random_num > 0.5:
                        new_bottom.append([second_material, new_top[i][1]])
                    else:
                        new_bottom.append([first_material, round(new_top[i][1] - get_width(first_material)/2, 10)])
                        new_bottom.append([first_material, round(new_top[i][1] + get_width(first_material)/2, 10)])
                        # no more detailed overlap check for it
                        # no avoid of long blocks used
        elif craft_method == 2:
            if pre_block_right <= round(new_top[i][1] - blocks[str(new_top[i][0])][0] / 2, 10):
                # for some reason, no left, so add left
                new_bottom.append([first_material, round(new_top[i][1] - blocks[str(new_top[i][0])][0]/2 + blocks[str(first_material)][0]/2, 10)])
            pre_block_right = round(new_bottom[-1][1] + blocks[str(new_bottom[-1][0])][0] / 2, 10)
            if pre_block_right + 0.02 >= new_top[i][1] + get_width(new_top[i][0]) / 2:
                # 0.02 is for the considerarion of block 1 is 0.84 while roof can be 0.85
                continue  # the current bottom is already able to support the next roof, continue
            if get_width(first_material) > 0.25:
                new_bottom.append([first_material, round(new_top[i][1] + blocks[str(new_top[i][0])][0] / 2, 10)])
            else:
                random_num = uniform(0.0, 1.0)
                if second_material != 0 and random_num > 0.5:
                    new_bottom.append([second_material, round(new_top[i][1] + blocks[str(new_top[i][0])][0] / 2, 10)])
                else:
                    new_bottom.append([first_material, round(new_top[i][1] + blocks[str(new_top[i][0])][0] / 2 - get_width(first_material) / 2, 10)])
        else:
            if get_width(new_top[i][0]) + 0.02 >= 2 * get_width(first_material):
                if get_width(new_top[i][0]) > 1.5 and blocks[str(first_material)][1] > 0.82 and blocks[str(first_material)][1] < 0.86:
                    print("sp")
                    ran_num = uniform(0.0, 1.0)
                    if ran_num > 0.8:
                        current_material = 14
                    else:
                        current_material = first_material
                else:
                    current_material = first_material
                if pre_block_right < round(new_top[i][1] - blocks[str(new_top[i][0])][0] / 4, 10):
                    new_block_left = round(new_top[i][1] - (blocks[str(new_top[i][0])][0] / 4) - get_width(first_material)/2, 10)
                    if pre_block_right > new_block_left:
                        if first_material == 1:
                            new_bottom.append([3, round(new_top[i][1] - blocks[str(new_top[i][0])][0] / 4, 10)])
                        else:
                            new_bottom.append([first_material, round(new_top[i][1] - blocks[str(new_top[i][0])][0] / 4, 10)])
                    else:
                        new_bottom.append([current_material, round(new_top[i][1] - blocks[str(new_top[i][0])][0] / 4, 10)])
                new_bottom.append([current_material, round(new_top[i][1] + blocks[str(new_top[i][0])][0] / 4, 10)])
            else:
                new_bottom.append([first_material, new_top[i][1]])

        pre_block_right = round(new_bottom[-1][1] + blocks[str(new_bottom[-1][0])][0]/2, 10)

    ## handle the last roof
    pre_block_right = round(new_bottom[-1][1] + blocks[str(new_bottom[-1][0])][0]/2, 10)
    if new_top[-1][1] > pre_block_right:
        if craft_method == 1:
            if new_top[-1][1] > pre_right:
                return add_new_building_row(pre_top, total_building, center, width, strong)
            if get_width(first_material) > 0.25:
                new_bottom.append([first_material, new_top[-1][1]])
            else:
                random_num = uniform(0.0, 1.0)
                if second_material != 0 and random_num > 0.5:
                    new_bottom.append([second_material, new_top[-1][1]])
                else:
                    new_bottom.append([first_material, round(new_top[-1][1] - get_width(first_material) / 2, 10)])
                    new_bottom.append([first_material, round(new_top[-1][1] + get_width(first_material) / 2, 10)])
        elif craft_method == 2:
            if pre_block_right - 0.02 <= round(new_top[-1][1] - blocks[str(new_top[-1][0])][0] / 2, 10):
                new_bottom.append([first_material, round(new_top[-1][1] - blocks[str(new_top[-1][0])][0]/2 + blocks[str(first_material)][0]/2, 10)])
            else:
                print("last no left:")
                print(new_bottom)
                print(new_top)
            if new_top[-1][1] + blocks[str(new_top[-1][0])][0]/2 > pre_right:
                if new_top[-1][1] + blocks[str(new_top[-1][0])][0]/4 > pre_right:
                    return add_new_building_row(pre_top, total_building, center, width, strong)
                else:
                    new_bottom.append([first_material, round(new_top[-1][1] + blocks[str(new_top[-1][0])][0] / 4, 10)])
            else:
                new_bottom.append([first_material, round(new_top[-1][1] + blocks[str(new_top[-1][0])][0] / 2, 10)])
        else:  # currently crafting methods == 3
            if round(new_top[-1][1] - blocks[str(new_top[-1][0])][0] / 2, 10) + 0.02 >= pre_block_right:
                new_bottom.append([first_material, round(new_top[-1][1] - blocks[str(new_top[-1][0])][0] / 4, 10)])
            if new_top[-1][1] + blocks[str(new_top[-1][0])][0] / 4 > pre_right:
                return add_new_building_row(pre_top, total_building, center, width, strong)
            else:
                new_bottom.append([first_material, round(new_top[-1][1] + blocks[str(new_top[-1][0])][0] / 4, 10)])

    if not check_no_overlap(new_bottom):
        return add_new_building_row(pre_top, total_building, center, width, strong)
    if repeat == 1:
        total_building.append(new_bottom)
    total_building.append(new_bottom)
    total_building.append(new_top)
    current_top = deepcopy(new_top)

    return total_building, current_top


def make_building(absolute_ground, center_point, max_width, max_height, sp_height):

    # init
    sp_layers = 1
    total_height = 4
    init_width = 5

    # creat building
    total_building, current_top = make_bottom(center_point, init_width)
    width = init_width
    center = center_point
    strong = 1
    for i in range(total_height):
        if width > 2.5:
            width = width - uniform(0.0, 0.5)
        ran_num = uniform(0.0, 1)  # for shifting center
        if width > 3.5 and ran_num > 0.9:
            shiftting_center = uniform(-1.0, 1.0)
            center = center + shiftting_center
            width = width - abs(shiftting_center)
        # if width >= 5:
        #     ran_num = uniform(0.0, 1.0)
        #     if ran_num > 0.8:
        #         for j in range(i, total_height):
        #             total_building, current_top = add_new_building_row(current_top, total_building, center, width)
        ran_num = uniform(0.0, 1)  # for shifting center
        if ran_num <0.2 and strong == 1:
            strong = 0
        else:
            strong = 1
        total_building, current_top = add_new_building_row(current_top, total_building, center, width, strong)

    # later part
    complete_locations = []
    ground = absolute_ground
    for row in total_building:
        for item in row:
            complete_locations.append([item[0], item[1], round((((blocks[str(item[0])][1]) / 2) + ground), 10)])
        ground = ground + (blocks[str(item[0])][1])

    print("Width:", find_structure_width(complete_locations))
    print("Height:", find_structure_height(complete_locations))
    print("Block number:", len(complete_locations))  # number blocks present in the structure

    # identify all possible pig positions on top of blocks (maximum 2 pigs per block, checks center before sides)
    possible_pig_positions = []
    for block in complete_locations:
        block_width = round(blocks[str(block[0])][0], 10)
        block_height = round(blocks[str(block[0])][1], 10)
        pig_width = pig_size[0]
        pig_height = pig_size[1]

        if blocks[str(block[0])][0] < pig_width:  # dont place block on edge if block too thin
            test_positions = [[round(block[1], 10), round(block[2] + (pig_height / 2) + (block_height / 2), 10)]]
        else:
            test_positions = [[round(block[1], 10), round(block[2] + (pig_height / 2) + (block_height / 2), 10)],
                              [round(block[1] + (block_width / 3), 10),
                               round(block[2] + (pig_height / 2) + (block_height / 2), 10)],
                              [round(block[1] - (block_width / 3), 10),
                               round(block[2] + (pig_height / 2) + (block_height / 2),
                                     10)]]  # check above centre of block
        for test_position in test_positions:
            valid_pig = True
            for i in complete_locations:
                if (round((test_position[0] - pig_width / 2), 10) < round((i[1] + (blocks[str(i[0])][0]) / 2), 10) and
                        round((test_position[0] + pig_width / 2), 10) > round((i[1] - (blocks[str(i[0])][0]) / 2),
                                                                              10) and
                        round((test_position[1] + pig_height / 2), 10) > round((i[2] - (blocks[str(i[0])][1]) / 2),
                                                                               10) and
                        round((test_position[1] - pig_height / 2), 10) < round((i[2] + (blocks[str(i[0])][1]) / 2),
                                                                               10)):
                    valid_pig = False
            if valid_pig == True:
                possible_pig_positions.append(test_position)

    # identify all possible pig positions on ground within structure
    left_bottom = total_building[0][0]
    right_bottom = total_building[0][-1]
    test_positions = []
    x_pos = left_bottom[1]

    while x_pos < right_bottom[1]:
        test_positions.append([round(x_pos, 10), round(absolute_ground + (pig_height / 2), 10)])
        x_pos = x_pos + pig_precision

    for test_position in test_positions:
        valid_pig = True
        for i in complete_locations:
            if (round((test_position[0] - pig_width / 2), 10) < round((i[1] + (blocks[str(i[0])][0]) / 2), 10) and
                    round((test_position[0] + pig_width / 2), 10) > round((i[1] - (blocks[str(i[0])][0]) / 2), 10) and
                    round((test_position[1] + pig_height / 2), 10) > round((i[2] - (blocks[str(i[0])][1]) / 2), 10) and
                    round((test_position[1] - pig_height / 2), 10) < round((i[2] + (blocks[str(i[0])][1]) / 2), 10)):
                valid_pig = False
        if valid_pig == True:
            possible_pig_positions.append(test_position)

    # randomly choose a pig position and remove those that overlap it, repeat until no more valid positions
    final_pig_positions = []
    while len(possible_pig_positions) > 0:
        pig_choice = possible_pig_positions.pop(randint(1, len(possible_pig_positions)) - 1)
        final_pig_positions.append(pig_choice)
        new_pig_positions = []
        for i in possible_pig_positions:
            if (round((pig_choice[0] - pig_width / 2), 10) >= round((i[0] + pig_width / 2), 10) or
                    round((pig_choice[0] + pig_width / 2), 10) <= round((i[0] - pig_width / 2), 10) or
                    round((pig_choice[1] + pig_height / 2), 10) <= round((i[1] - pig_height / 2), 10) or
                    round((pig_choice[1] - pig_height / 2), 10) >= round((i[1] + pig_height / 2), 10)):
                new_pig_positions.append(i)
        possible_pig_positions = new_pig_positions

    print("Pig number:", len(final_pig_positions))  # number of pigs present in the structure
    print("")

    return complete_locations, final_pig_positions, sp_layers


# recursively adds rows to base of strucutre until max_width or max_height is passed
# once this happens the last row added is removed and the structure is returned

def make_structure(absolute_ground, center_point, max_width, max_height, sp_height):
    
    total_tree = []                 # all blocks of structure (so far)

    # creates the first row (peaks) for the structure, ensuring that max_width restriction is satisfied
    current_tree_bottom = make_peaks(center_point)
    if max_width > 0.0:
        while find_structure_width(current_tree_bottom) > max_width:
            current_tree_bottom = make_peaks(center_point)

    total_tree.append(current_tree_bottom)

    angle = 0
    release_point = find_release_point(angle)
    trajectory = find_trajectory(release_point[0], release_point[1])
    for point in trajectory:
        point[0] = round(point[0] + slingshot_x, 10)
        point[1] = round(point[1] + slingshot_y, 10)

    # recursively add more rows of blocks to the level structure
    structure_width = find_structure_width(current_tree_bottom)
    structure_height = (blocks[str(current_tree_bottom[0][0])][1])/2
    if max_height > 0.0 or max_width > 0.0:
        pre_total_tree = [current_tree_bottom]
        while structure_height < max_height and structure_width < max_width:
            total_tree, current_tree_bottom = add_new_row(current_tree_bottom, total_tree)
            complete_locations = []
            ground = absolute_ground
            for row in reversed(total_tree):
                for item in row:
                    complete_locations.append([item[0],item[1],round((((blocks[str(item[0])][1])/2)+ground),10)])
                ground = ground + (blocks[str(item[0])][1])
            structure_height = find_structure_height(complete_locations)
            structure_width = find_structure_width(complete_locations)
            if structure_height > max_height or structure_width > max_width:
                total_tree = deepcopy(pre_total_tree)
            else:
                pre_total_tree = deepcopy(total_tree)

    # previous: make structure vertically correct (add y position to blocks)
    # Shutong: find the layer of the bird target
    complete_locations = []
    ground = absolute_ground
    sp_layers = -1
    layers = 0
    total_layers = len(total_tree)
    for row in reversed(total_tree):
        for item in row:
            if (round((((blocks[str(item[0])][1])/2)+ground),10) + blocks[str(item[0])][1]) > sp_height and (round((((blocks[str(item[0])][1])/2)+ground),10) - blocks[str(item[0])][1]) < sp_height:
                sp_layers = layers
            complete_locations.append([item[0],item[1],round((((blocks[str(item[0])][1])/2)+ground),10)])
        ground = ground + (blocks[str(item[0])][1])
        layers = layers + 1

    # find the splayer and delete the current tree under it (include itself)
    sp_layers = total_layers - sp_layers - 1
    layers = 0
    new_total_tree = []
    new_current_bottom = []
    for row in total_tree:
        new_total_tree.append(row)
        if layers == sp_layers:
            new_current_bottom = deepcopy(row)
            break
        layers = layers + 1
    total_tree = deepcopy(new_total_tree)
    current_tree_bottom = deepcopy(new_current_bottom)

    # creat the new building
    print(total_tree)
    print(current_tree_bottom)
    total_tree, current_tree_bottom = add_yellow_bird_new_row(current_tree_bottom, total_tree)
    if not total_tree:
        return make_structure(absolute_ground, center_point, max_width, max_height)
    complete_locations = []
    ground = absolute_ground
    for row in reversed(total_tree):
        for item in row:
            complete_locations.append([item[0], item[1], round((((blocks[str(item[0])][1]) / 2) + ground), 10)])
        ground = ground + (blocks[str(item[0])][1])
    structure_height = find_structure_height(complete_locations)
    structure_width = find_structure_width(complete_locations)

    # add more rows under the sp row
    pre_total_tree = deepcopy(total_tree)
    while structure_height < max_height and structure_width < max_width:
        total_tree, current_tree_bottom = add_new_row(current_tree_bottom, total_tree)
        complete_locations = []
        ground = absolute_ground
        for row in reversed(total_tree):
            for item in row:
                complete_locations.append([item[0], item[1], round((((blocks[str(item[0])][1]) / 2) + ground), 10)])
            ground = ground + (blocks[str(item[0])][1])
        structure_height = find_structure_height(complete_locations)
        structure_width = find_structure_width(complete_locations)
        if structure_height > max_height or structure_width > max_width:
            total_tree = deepcopy(pre_total_tree)
        else:
            pre_total_tree = deepcopy(total_tree)

    # calculate the height
    # make structure vertically correct (add y position to blocks)
    complete_locations = []
    ground = absolute_ground
    for row in reversed(total_tree):
        for item in row:
            complete_locations.append([item[0], item[1], round((((blocks[str(item[0])][1]) / 2) + ground), 10)])
        ground = ground + (blocks[str(item[0])][1])

    print("Width:", find_structure_width(complete_locations))
    print("Height:", find_structure_height(complete_locations))
    print("Block number:", len(complete_locations))  # number blocks present in the structure


    # identify all possible pig positions on top of blocks (maximum 2 pigs per block, checks center before sides)
    possible_pig_positions = []
    for block in complete_locations:
        block_width = round(blocks[str(block[0])][0],10)
        block_height = round(blocks[str(block[0])][1],10)
        pig_width = pig_size[0]
        pig_height = pig_size[1]

        if blocks[str(block[0])][0] < pig_width:      # dont place block on edge if block too thin
            test_positions = [[round(block[1],10),round(block[2] + (pig_height/2) + (block_height/2),10)]]
        else:
            test_positions = [ [round(block[1],10),round(block[2] + (pig_height/2) + (block_height/2),10)],
                               [round(block[1] + (block_width/3),10),round(block[2] + (pig_height/2) + (block_height/2),10)],
                               [round(block[1] - (block_width/3),10),round(block[2] + (pig_height/2) + (block_height/2),10)]]     #check above centre of block
        for test_position in test_positions:
            valid_pig = True
            for i in complete_locations:
                if ( round((test_position[0] - pig_width/2),10) < round((i[1] + (blocks[str(i[0])][0])/2),10) and
                     round((test_position[0] + pig_width/2),10) > round((i[1] - (blocks[str(i[0])][0])/2),10) and
                     round((test_position[1] + pig_height/2),10) > round((i[2] - (blocks[str(i[0])][1])/2),10) and
                     round((test_position[1] - pig_height/2),10) < round((i[2] + (blocks[str(i[0])][1])/2),10)):
                    valid_pig = False
            if valid_pig == True:
                possible_pig_positions.append(test_position)


    #identify all possible pig positions on ground within structure
    left_bottom = total_tree[-1][0]
    right_bottom = total_tree[-1][-1]
    test_positions = []
    x_pos = left_bottom[1]

    while x_pos < right_bottom[1]:
        test_positions.append([round(x_pos,10),round(absolute_ground + (pig_height/2),10)])
        x_pos = x_pos + pig_precision

    for test_position in test_positions:
        valid_pig = True
        for i in complete_locations:
            if ( round((test_position[0] - pig_width/2),10) < round((i[1] + (blocks[str(i[0])][0])/2),10) and
                 round((test_position[0] + pig_width/2),10) > round((i[1] - (blocks[str(i[0])][0])/2),10) and
                 round((test_position[1] + pig_height/2),10) > round((i[2] - (blocks[str(i[0])][1])/2),10) and
                 round((test_position[1] - pig_height/2),10) < round((i[2] + (blocks[str(i[0])][1])/2),10)):
                valid_pig = False
        if valid_pig == True:
            possible_pig_positions.append(test_position)


    #randomly choose a pig position and remove those that overlap it, repeat until no more valid positions
    final_pig_positions = []
    while len(possible_pig_positions) > 0:
        pig_choice = possible_pig_positions.pop(randint(1,len(possible_pig_positions))-1)
        final_pig_positions.append(pig_choice)
        new_pig_positions = []
        for i in possible_pig_positions:
            if ( round((pig_choice[0] - pig_width/2),10) >= round((i[0] + pig_width/2),10) or
                 round((pig_choice[0] + pig_width/2),10) <= round((i[0] - pig_width/2),10) or
                 round((pig_choice[1] + pig_height/2),10) <= round((i[1] - pig_height/2),10) or
                 round((pig_choice[1] - pig_height/2),10) >= round((i[1] + pig_height/2),10)):
                new_pig_positions.append(i)
        possible_pig_positions = new_pig_positions

    print("Pig number:", len(final_pig_positions))     # number of pigs present in the structure
    print("")

    return complete_locations, final_pig_positions, sp_layers


# divide the available ground space between the chosen number of ground structures

def create_ground_structures():
    angle = 0
    release_point = find_release_point(angle)
    trajectory = find_trajectory(release_point[0], release_point[1])
    point_num = 0
    posi = -1 # design to intersect at posi
    for point in trajectory:
        point[0] = round(point[0] + slingshot_x, 10)
        point[1] = round(point[1] + slingshot_y, 10)
        if point[1] <= posi:
            intersect = point_num
        point_num = point_num + 1
    structure_width = 3

    # determine the area available to each ground structure
    ground_positions = []
    ground_widths = []
    for j in range(1):
        ground_positions.append(6)
        ground_widths.append(structure_width)

    print("number ground structures:", len(ground_positions))
    print("")

    # creates a ground structure for each defined area 
    complete_locations = []
    final_pig_positions = []
    for i in range(len(ground_positions)):
        max_width = ground_widths[i]
        max_height = ground_structure_height_limit
        center_point = ground_positions[i]
        complete_locations2, final_pig_positions2, sp_layer = make_building(absolute_ground, center_point, max_width, max_height, trajectory[intersect][1])
        complete_locations = complete_locations + complete_locations2
        final_pig_positions = final_pig_positions + final_pig_positions2

    return len(ground_positions), complete_locations, final_pig_positions, sp_layer # shutong: should be sp_layers for multiply buildings




# creates a set number of platforms within the level
# automatically reduced if space not found after set number of attempts

def create_platforms(number_platforms, complete_locations, final_pig_positions):

    platform_centers = []
    attempts = 0            # number of attempts so far to find space for platform
    final_platforms = []
    while len(final_platforms) < number_platforms:
        platform_width = randint(4,7)
        platform_position = [uniform(level_width_min+((platform_width*platform_size[0])/2.0), level_width_max-((platform_width*platform_size[0])/2.0)),
                             uniform(level_height_min, (level_height_max - minimum_height_gap))]
        temp_platform = []

        if platform_width == 1:
            temp_platform.append(platform_position)     

        if platform_width == 2:
            temp_platform.append([platform_position[0] - (platform_size[0]*0.5),platform_position[1]])
            temp_platform.append([platform_position[0] + (platform_size[0]*0.5),platform_position[1]])

        if platform_width == 3:
            temp_platform.append([platform_position[0] - (platform_size[0]),platform_position[1]])
            temp_platform.append(platform_position) 
            temp_platform.append([platform_position[0] + (platform_size[0]),platform_position[1]])

        if platform_width == 4:
            temp_platform.append([platform_position[0] - (platform_size[0]*1.5),platform_position[1]])
            temp_platform.append([platform_position[0] - (platform_size[0]*0.5),platform_position[1]])
            temp_platform.append([platform_position[0] + (platform_size[0]*0.5),platform_position[1]])
            temp_platform.append([platform_position[0] + (platform_size[0]*1.5),platform_position[1]])

        if platform_width == 5:
            temp_platform.append([platform_position[0] - (platform_size[0]*2.0),platform_position[1]])
            temp_platform.append([platform_position[0] - (platform_size[0]),platform_position[1]])
            temp_platform.append(platform_position) 
            temp_platform.append([platform_position[0] + (platform_size[0]),platform_position[1]])
            temp_platform.append([platform_position[0] + (platform_size[0]*2.0),platform_position[1]])

        if platform_width == 6:
            temp_platform.append([platform_position[0] - (platform_size[0]*2.5),platform_position[1]])
            temp_platform.append([platform_position[0] - (platform_size[0]*1.5),platform_position[1]])
            temp_platform.append([platform_position[0] - (platform_size[0]*0.5),platform_position[1]])
            temp_platform.append([platform_position[0] + (platform_size[0]*0.5),platform_position[1]])
            temp_platform.append([platform_position[0] + (platform_size[0]*1.5),platform_position[1]])
            temp_platform.append([platform_position[0] + (platform_size[0]*2.5),platform_position[1]])

        if platform_width == 7:
            temp_platform.append([platform_position[0] - (platform_size[0]*3.0),platform_position[1]])
            temp_platform.append([platform_position[0] - (platform_size[0]*2.0),platform_position[1]])
            temp_platform.append([platform_position[0] - (platform_size[0]),platform_position[1]])
            temp_platform.append(platform_position) 
            temp_platform.append([platform_position[0] + (platform_size[0]),platform_position[1]])
            temp_platform.append([platform_position[0] + (platform_size[0]*2.0),platform_position[1]])
            temp_platform.append([platform_position[0] + (platform_size[0]*3.0),platform_position[1]])
            
        overlap = False
        for platform in temp_platform:

            if (((platform[0]-(platform_size[0]/2)) < level_width_min) or ((platform[0]+(platform_size[0])/2) > level_width_max)):
                overlap = True
            
            for block in complete_locations:
                if ( round((platform[0] - platform_distance_buffer - platform_size[0]/2),10) <= round((block[1] + blocks[str(block[0])][0]/2),10) and
                     round((platform[0] + platform_distance_buffer + platform_size[0]/2),10) >= round((block[1] - blocks[str(block[0])][0]/2),10) and
                     round((platform[1] + platform_distance_buffer + platform_size[1]/2),10) >= round((block[2] - blocks[str(block[0])][1]/2),10) and
                     round((platform[1] - platform_distance_buffer - platform_size[1]/2),10) <= round((block[2] + blocks[str(block[0])][1]/2),10)):
                    overlap = True
                    
            for pig in final_pig_positions:
                if ( round((platform[0] - platform_distance_buffer - platform_size[0]/2),10) <= round((pig[0] + pig_size[0]/2),10) and
                     round((platform[0] + platform_distance_buffer + platform_size[0]/2),10) >= round((pig[0] - pig_size[0]/2),10) and
                     round((platform[1] + platform_distance_buffer + platform_size[1]/2),10) >= round((pig[1] - pig_size[1]/2),10) and
                     round((platform[1] - platform_distance_buffer - platform_size[1]/2),10) <= round((pig[1] + pig_size[1]/2),10)):
                    overlap = True

            for platform_set in final_platforms:
                for platform2 in platform_set:
                    if ( round((platform[0] - platform_distance_buffer - platform_size[0]/2),10) <= round((platform2[0] + platform_size[0]/2),10) and
                         round((platform[0] + platform_distance_buffer + platform_size[0]/2),10) >= round((platform2[0] - platform_size[0]/2),10) and
                         round((platform[1] + platform_distance_buffer + platform_size[1]/2),10) >= round((platform2[1] - platform_size[1]/2),10) and
                         round((platform[1] - platform_distance_buffer - platform_size[1]/2),10) <= round((platform2[1] + platform_size[1]/2),10)):
                        overlap = True

            for platform_set2 in final_platforms:
                for i in platform_set2:
                    if i[0]+platform_size[0] > platform[0] and i[0]-platform_size[0] < platform[0]:
                        if i[1]+minimum_height_gap > platform[1] and i[1]-minimum_height_gap < platform[1]:
                            overlap = True
                            
        if overlap == False:
            final_platforms.append(temp_platform)
            platform_centers.append(platform_position)

        attempts = attempts + 1
        if attempts > max_attempts:
            attempts = 0
            number_platforms = number_platforms - 1
            
    print("number platforms:", number_platforms)
    print("")

    return number_platforms, final_platforms, platform_centers




# create sutiable structures for each platform

def create_platform_structures(final_platforms, platform_centers, complete_locations, final_pig_positions):
    current_platform = 0
    for platform_set in final_platforms:
        platform_set_width = len(platform_set)*platform_size[0]

        above_blocks = []
        for platform_set2 in final_platforms:
            if platform_set2 != platform_set:
                for i in platform_set2:
                    if i[0]+platform_size[0] > platform_set[0][0] and i[0]-platform_size[0] < platform_set[-1][0] and i[1] > platform_set[0][1]:
                        above_blocks.append(i)

        min_above = level_height_max
        for j in above_blocks:
            if j[1] < min_above:
                min_above = j[1]

        center_point = platform_centers[current_platform][0]
        absolute_ground = platform_centers[current_platform][1] + (platform_size[1]/2)

        max_width = platform_set_width
        max_height = (min_above - absolute_ground)- pig_size[1] - platform_size[1]
        
        complete_locations2, final_pig_positions2 = make_structure(absolute_ground, center_point, max_width, max_height)
        complete_locations = complete_locations + complete_locations2
        final_pig_positions = final_pig_positions + final_pig_positions2

        current_platform = current_platform + 1

    return complete_locations, final_pig_positions




# remove random pigs until number equals the desired amount

def remove_unnecessary_pigs(number_pigs):
    removed_pigs = []
    while len(final_pig_positions) > number_pigs:
              remove_pos = randint(0,len(final_pig_positions)-1)
              removed_pigs.append(final_pig_positions[remove_pos])
              final_pig_positions.pop(remove_pos)
    return final_pig_positions, removed_pigs




# add pigs on the ground until number equals the desired amount

def add_necessary_pigs(number_pigs):
    while len(final_pig_positions) < number_pigs:
        test_position = [uniform(level_width_min, level_width_max),absolute_ground]
        pig_width = pig_size[0]
        pig_height = pig_size[1]
        valid_pig = True
        for i in complete_locations:
            if ( round((test_position[0] - pig_width/2),10) < round((i[1] + (blocks[str(i[0])][0])/2),10) and
                 round((test_position[0] + pig_width/2),10) > round((i[1] - (blocks[str(i[0])][0])/2),10) and
                 round((test_position[1] + pig_height/2),10) > round((i[2] - (blocks[str(i[0])][1])/2),10) and
                 round((test_position[1] - pig_height/2),10) < round((i[2] + (blocks[str(i[0])][1])/2),10)):
                valid_pig = False
        for i in final_pig_positions:
            if ( round((test_position[0] - pig_width/2),10) < round((i[0] + (pig_width/2)),10) and
                 round((test_position[0] + pig_width/2),10) > round((i[0] - (pig_width/2)),10) and
                 round((test_position[1] + pig_height/2),10) > round((i[1] - (pig_height/2)),10) and
                 round((test_position[1] - pig_height/2),10) < round((i[1] + (pig_height/2)),10)):
                valid_pig = False
        if valid_pig == True:
            final_pig_positions.append(test_position)
    return final_pig_positions




# choose the number of birds based on the number of pigs and structures present within level

def choose_number_birds(final_pig_positions,number_ground_structures,number_platforms):
    number_birds = int(ceil(len(final_pig_positions)/2))
    if (number_ground_structures + number_platforms) >= number_birds:
        number_birds = number_birds + 1
    number_birds = number_birds + 1         # adjust based on desired difficulty        
    return number_birds




# identify all possible triangleHole positions on top of blocks

def find_trihole_positions(complete_locations):
    possible_trihole_positions = []
    for block in complete_locations:
        block_width = round(blocks[str(block[0])][0],10)
        block_height = round(blocks[str(block[0])][1],10)
        trihole_width = additional_object_sizes['1'][0]
        trihole_height = additional_object_sizes['1'][1]

        # don't place block on edge if block too thin
        if blocks[str(block[0])][0] < trihole_width:
            test_positions = [ [round(block[1],10),round(block[2] + (trihole_height/2) + (block_height/2),10)]]
        else:
            test_positions = [ [round(block[1],10),round(block[2] + (trihole_height/2) + (block_height/2),10)],
                               [round(block[1] + (block_width/3),10),round(block[2] + (trihole_height/2) + (block_height/2),10)],
                               [round(block[1] - (block_width/3),10),round(block[2] + (trihole_height/2) + (block_height/2),10)] ]
        
        for test_position in test_positions:
            valid_position = True
            for i in complete_locations:
                if ( round((test_position[0] - trihole_width/2),10) < round((i[1] + (blocks[str(i[0])][0])/2),10) and
                     round((test_position[0] + trihole_width/2),10) > round((i[1] - (blocks[str(i[0])][0])/2),10) and
                     round((test_position[1] + trihole_height/2),10) > round((i[2] - (blocks[str(i[0])][1])/2),10) and
                     round((test_position[1] - trihole_height/2),10) < round((i[2] + (blocks[str(i[0])][1])/2),10)):
                    valid_position = False
            for j in final_pig_positions:
                if ( round((test_position[0] - trihole_width/2),10) < round((j[0] + (pig_size[0]/2)),10) and
                     round((test_position[0] + trihole_width/2),10) > round((j[0] - (pig_size[0]/2)),10) and
                     round((test_position[1] + trihole_height/2),10) > round((j[1] - (pig_size[1]/2)),10) and
                     round((test_position[1] - trihole_height/2),10) < round((j[1] + (pig_size[1]/2)),10)):
                    valid_position = False
            for j in final_TNT_positions:
                if ( round((test_position[0] - trihole_width/2),10) < round((j[0] + (pig_size[0]/2)),10) and
                     round((test_position[0] + trihole_width/2),10) > round((j[0] - (pig_size[0]/2)),10) and
                     round((test_position[1] + trihole_height/2),10) > round((j[1] - (pig_size[1]/2)),10) and
                     round((test_position[1] - trihole_height/2),10) < round((j[1] + (pig_size[1]/2)),10)):
                    valid_position = False
            for i in final_platforms:
                for j in i:
                    if ( round((test_position[0] - trihole_width/2),10) < round((j[0] + (platform_size[0]/2)),10) and
                         round((test_position[0] + trihole_width/2),10) > round((j[0] - (platform_size[0]/2)),10) and
                         round((test_position[1] + platform_distance_buffer + trihole_height/2),10) > round((j[1] - (platform_size[1]/2)),10) and
                         round((test_position[1] - platform_distance_buffer - trihole_height/2),10) < round((j[1] + (platform_size[1]/2)),10)):
                        valid_position = False
            if valid_position == True:
                possible_trihole_positions.append(test_position)
                        
    return possible_trihole_positions




# identify all possible triangle positions on top of blocks

def find_tri_positions(complete_locations):
    possible_tri_positions = []
    for block in complete_locations:
        block_width = round(blocks[str(block[0])][0],10)
        block_height = round(blocks[str(block[0])][1],10)
        tri_width = additional_object_sizes['2'][0]
        tri_height = additional_object_sizes['2'][1]
        
        # don't place block on edge if block too thin
        if blocks[str(block[0])][0] < tri_width:
            test_positions = [ [round(block[1],10),round(block[2] + (tri_height/2) + (block_height/2),10)]]
        else:
            test_positions = [ [round(block[1],10),round(block[2] + (tri_height/2) + (block_height/2),10)],
                               [round(block[1] + (block_width/3),10),round(block[2] + (tri_height/2) + (block_height/2),10)],
                               [round(block[1] - (block_width/3),10),round(block[2] + (tri_height/2) + (block_height/2),10)] ]
        
        for test_position in test_positions:
            valid_position = True
            for i in complete_locations:
                if ( round((test_position[0] - tri_width/2),10) < round((i[1] + (blocks[str(i[0])][0])/2),10) and
                     round((test_position[0] + tri_width/2),10) > round((i[1] - (blocks[str(i[0])][0])/2),10) and
                     round((test_position[1] + tri_height/2),10) > round((i[2] - (blocks[str(i[0])][1])/2),10) and
                     round((test_position[1] - tri_height/2),10) < round((i[2] + (blocks[str(i[0])][1])/2),10)):
                    valid_position = False
            for j in final_pig_positions:
                if ( round((test_position[0] - tri_width/2),10) < round((j[0] + (pig_size[0]/2)),10) and
                     round((test_position[0] + tri_width/2),10) > round((j[0] - (pig_size[0]/2)),10) and
                     round((test_position[1] + tri_height/2),10) > round((j[1] - (pig_size[1]/2)),10) and
                     round((test_position[1] - tri_height/2),10) < round((j[1] + (pig_size[1]/2)),10)):
                    valid_position = False
            for j in final_TNT_positions:
                if ( round((test_position[0] - tri_width/2),10) < round((j[0] + (pig_size[0]/2)),10) and
                     round((test_position[0] + tri_width/2),10) > round((j[0] - (pig_size[0]/2)),10) and
                     round((test_position[1] + tri_height/2),10) > round((j[1] - (pig_size[1]/2)),10) and
                     round((test_position[1] - tri_height/2),10) < round((j[1] + (pig_size[1]/2)),10)):
                    valid_position = False
            for i in final_platforms:
                for j in i:
                    if ( round((test_position[0] - tri_width/2),10) < round((j[0] + (platform_size[0]/2)),10) and
                         round((test_position[0] + tri_width/2),10) > round((j[0] - (platform_size[0]/2)),10) and
                         round((test_position[1] + platform_distance_buffer + tri_height/2),10) > round((j[1] - (platform_size[1]/2)),10) and
                         round((test_position[1] - platform_distance_buffer - tri_height/2),10) < round((j[1] + (platform_size[1]/2)),10)):
                        valid_position = False
                        
            if blocks[str(block[0])][0] < tri_width:      # as block not symmetrical need to check for support
                valid_position = False
            if valid_position == True:
                possible_tri_positions.append(test_position)

    return possible_tri_positions




# identify all possible circle positions on top of blocks (can only be placed in middle of block)

def find_cir_positions(complete_locations):
    possible_cir_positions = []
    for block in complete_locations:
        block_width = round(blocks[str(block[0])][0],10)
        block_height = round(blocks[str(block[0])][1],10)
        cir_width = additional_object_sizes['3'][0]
        cir_height = additional_object_sizes['3'][1]

        # only checks above block's center
        test_positions = [ [round(block[1],10),round(block[2] + (cir_height/2) + (block_height/2),10)]]
        
        for test_position in test_positions:
            valid_position = True
            for i in complete_locations:
                if ( round((test_position[0] - cir_width/2),10) < round((i[1] + (blocks[str(i[0])][0])/2),10) and
                     round((test_position[0] + cir_width/2),10) > round((i[1] - (blocks[str(i[0])][0])/2),10) and
                     round((test_position[1] + cir_height/2),10) > round((i[2] - (blocks[str(i[0])][1])/2),10) and
                     round((test_position[1] - cir_height/2),10) < round((i[2] + (blocks[str(i[0])][1])/2),10)):
                    valid_position = False
            for j in final_pig_positions:
                if ( round((test_position[0] - cir_width/2),10) < round((j[0] + (pig_size[0]/2)),10) and
                     round((test_position[0] + cir_width/2),10) > round((j[0] - (pig_size[0]/2)),10) and
                     round((test_position[1] + cir_height/2),10) > round((j[1] - (pig_size[1]/2)),10) and
                     round((test_position[1] - cir_height/2),10) < round((j[1] + (pig_size[1]/2)),10)):
                    valid_position = False
            for j in final_TNT_positions:
                if ( round((test_position[0] - cir_width/2),10) < round((j[0] + (pig_size[0]/2)),10) and
                     round((test_position[0] + cir_width/2),10) > round((j[0] - (pig_size[0]/2)),10) and
                     round((test_position[1] + cir_height/2),10) > round((j[1] - (pig_size[1]/2)),10) and
                     round((test_position[1] - cir_height/2),10) < round((j[1] + (pig_size[1]/2)),10)):
                    valid_position = False
            for i in final_platforms:
                for j in i:
                    if ( round((test_position[0] - cir_width/2),10) < round((j[0] + (platform_size[0]/2)),10) and
                         round((test_position[0] + cir_width/2),10) > round((j[0] - (platform_size[0]/2)),10) and
                         round((test_position[1] + platform_distance_buffer + cir_height/2),10) > round((j[1] - (platform_size[1]/2)),10) and
                         round((test_position[1] - platform_distance_buffer - cir_height/2),10) < round((j[1] + (platform_size[1]/2)),10)):
                        valid_position = False
            if valid_position == True:
                possible_cir_positions.append(test_position)

    return possible_cir_positions




# identify all possible circleSmall positions on top of blocks

def find_cirsmall_positions(complete_locations):
    possible_cirsmall_positions = []
    for block in complete_locations:
        block_width = round(blocks[str(block[0])][0],10)
        block_height = round(blocks[str(block[0])][1],10)
        cirsmall_width = additional_object_sizes['4'][0]
        cirsmall_height = additional_object_sizes['4'][1]

        # don't place block on edge if block too thin
        if blocks[str(block[0])][0] < cirsmall_width:
            test_positions = [ [round(block[1],10),round(block[2] + (cirsmall_height/2) + (block_height/2),10)]]
        else:
            test_positions = [ [round(block[1],10),round(block[2] + (cirsmall_height/2) + (block_height/2),10)],
                               [round(block[1] + (block_width/3),10),round(block[2] + (cirsmall_height/2) + (block_height/2),10)],
                               [round(block[1] - (block_width/3),10),round(block[2] + (cirsmall_height/2) + (block_height/2),10)] ]
        
        for test_position in test_positions:
            valid_position = True
            for i in complete_locations:
                if ( round((test_position[0] - cirsmall_width/2),10) < round((i[1] + (blocks[str(i[0])][0])/2),10) and
                     round((test_position[0] + cirsmall_width/2),10) > round((i[1] - (blocks[str(i[0])][0])/2),10) and
                     round((test_position[1] + cirsmall_height/2),10) > round((i[2] - (blocks[str(i[0])][1])/2),10) and
                     round((test_position[1] - cirsmall_height/2),10) < round((i[2] + (blocks[str(i[0])][1])/2),10)):
                    valid_position = False
            for j in final_pig_positions:
                if ( round((test_position[0] - cirsmall_width/2),10) < round((j[0] + (pig_size[0]/2)),10) and
                     round((test_position[0] + cirsmall_width/2),10) > round((j[0] - (pig_size[0]/2)),10) and
                     round((test_position[1] + cirsmall_height/2),10) > round((j[1] - (pig_size[1]/2)),10) and
                     round((test_position[1] - cirsmall_height/2),10) < round((j[1] + (pig_size[1]/2)),10)):
                    valid_position = False
            for j in final_TNT_positions:
                if ( round((test_position[0] - cirsmall_width/2),10) < round((j[0] + (pig_size[0]/2)),10) and
                     round((test_position[0] + cirsmall_width/2),10) > round((j[0] - (pig_size[0]/2)),10) and
                     round((test_position[1] + cirsmall_height/2),10) > round((j[1] - (pig_size[1]/2)),10) and
                     round((test_position[1] - cirsmall_height/2),10) < round((j[1] + (pig_size[1]/2)),10)):
                    valid_position = False
            for i in final_platforms:
                for j in i:
                    if ( round((test_position[0] - cirsmall_width/2),10) < round((j[0] + (platform_size[0]/2)),10) and
                         round((test_position[0] + cirsmall_width/2),10) > round((j[0] - (platform_size[0]/2)),10) and
                         round((test_position[1] + platform_distance_buffer + cirsmall_height/2),10) > round((j[1] - (platform_size[1]/2)),10) and
                         round((test_position[1] - platform_distance_buffer - cirsmall_height/2),10) < round((j[1] + (platform_size[1]/2)),10)):
                        valid_position = False
            if valid_position == True:
                possible_cirsmall_positions.append(test_position)

    return possible_cirsmall_positions




# finds possible positions for valid additional block types

def find_additional_block_positions(complete_locations):
    possible_trihole_positions = []
    possible_tri_positions = []
    possible_cir_positions = []
    possible_cirsmall_positions = []
    if trihole_allowed == True:
        possible_trihole_positions = find_trihole_positions(complete_locations)
    if tri_allowed == True:
        possible_tri_positions = find_tri_positions(complete_locations)
    if cir_allowed == True:
        possible_cir_positions = find_cir_positions(complete_locations)
    if cirsmall_allowed == True:
        possible_cirsmall_positions = find_cirsmall_positions(complete_locations)
    return possible_trihole_positions, possible_tri_positions, possible_cir_positions, possible_cirsmall_positions




# combine all possible additonal block positions into one set

def add_additional_blocks(possible_trihole_positions, possible_tri_positions, possible_cir_positions, possible_cirsmall_positions):
    all_other = []
    for i in possible_trihole_positions:
        all_other.append(['1',i[0],i[1]])
    for i in possible_tri_positions:
        all_other.append(['2',i[0],i[1]])
    for i in possible_cir_positions:
        all_other.append(['3',i[0],i[1]])
    for i in possible_cirsmall_positions:
        all_other.append(['4',i[0],i[1]])

    #randomly choose an additional block position and remove those that overlap it
    #repeat untill no more valid position

    selected_other = []
    while (len(all_other) > 0):
        chosen = all_other.pop(randint(0,len(all_other)-1))
        selected_other.append(chosen)
        new_all_other = []
        for i in all_other:
            if ( round((chosen[1] - (additional_object_sizes[chosen[0]][0]/2)),10) >= round((i[1] + (additional_object_sizes[i[0]][0]/2)),10) or
                 round((chosen[1] + (additional_object_sizes[chosen[0]][0]/2)),10) <= round((i[1] - (additional_object_sizes[i[0]][0]/2)),10) or
                 round((chosen[2] + (additional_object_sizes[chosen[0]][1]/2)),10) <= round((i[2] - (additional_object_sizes[i[0]][1]/2)),10) or
                 round((chosen[2] - (additional_object_sizes[chosen[0]][1]/2)),10) >= round((i[2] + (additional_object_sizes[i[0]][1]/2)),10)):
                new_all_other.append(i)
        all_other = new_all_other

    return selected_other




# remove restricted block types from the available selection

def remove_blocks(restricted_blocks):
    total_prob_removed = 0.0
    new_prob_table = deepcopy(probability_table_blocks)
    for block_name in restricted_blocks:
        for key,value in block_names.items():
            if value == block_name:
                total_prob_removed = total_prob_removed + probability_table_blocks[key]
                new_prob_table[key] = 0.0
    new_total = 1.0 - total_prob_removed
    for key, value in new_prob_table.items():
        new_prob_table[key] = value/new_total
    return new_prob_table




# add TNT blocks based on removed pig positions

def add_TNT(potential_positions):
    final_TNT_positions = []
    for position in potential_positions:
        if (uniform(0.0,1.0) < TNT_block_probability):
            final_TNT_positions.append(position)
    return final_TNT_positions


# set the material of each block

def set_materials(complete_locations, sp_layers):
    assigned_materials = []
    print(complete_locations)

    layers = 0
    current_height = complete_locations[0][2]
    for ii in complete_locations:
        if round(current_height, 10) != round(ii[2], 10):
            layers = layers + 1
        if layers == sp_layers or layers == sp_layers + 1:
            assigned_materials.append(materials[0])
        else:
            assigned_materials.append(materials[1])
    final_materials = []
    for i in assigned_materials:
        final_materials.append(i)

    return final_materials

# write level out in desired xml format

def write_level_xml(complete_locations, selected_other, final_pig_positions, final_TNT_positions, final_platforms, number_birds, current_level, restricted_combinations):

    f = open("level-%s.xml" % current_level, "w")

    f.write('<?xml version="1.0" encoding="utf-16"?>\n')
    f.write('<Level width ="2">\n')
    f.write('<Camera x="0" y="2" minWidth="20" maxWidth="30">\n')
    f.write('<Birds>\n')
    for i in range(number_birds):   # bird type is chosen using probability table
        f.write('<Bird type="%s"/>\n' % bird_names[str(choose_item(bird_probabilities))])
    f.write('</Birds>\n')
    f.write('<Slingshot x="-8" y="-2.5">\n')
    f.write('<GameObjects>\n')

    ii = 0
    for i in complete_locations:
        # shutong: material choosen here
        material = materials[randint(0,len(materials)-1)]       # material is chosen randomly
        while [material,block_names[str(i[0])]] in restricted_combinations:     # if material if not allowed for block type then pick again
            material = materials[randint(0,len(materials)-1)]
        rotation = 0
        if (i[0] in (3,7,9,11,13)):
            rotation = 90
        f.write('<Block type="%s" material="%s" x="%s" y="%s" rotation="%s" />\n' % (block_names[str(i[0])], material, str(i[1]), str(i[2]), str(rotation)))
        ii = ii + 1

    print("finish writing location")

    for i in selected_other:
        # shutong: material choosen here
        material = materials[randint(0,len(materials)-1)]       # material is chosen randomly
        while [material,additional_objects[str(i[0])]] in restricted_combinations:      # if material if not allowed for block type then pick again
            material = materials[randint(0,len(materials)-1)]
        if i[0] == '2':
            facing = randint(0,1)
            f.write('<Block type="%s" material="%s" x="%s" y="%s" rotation="%s" />\n' % (additional_objects[i[0]], material, str(i[1]), str(i[2]), str(facing*90.0)))
        else:
            f.write('<Block type="%s" material="%s" x="%s" y="%s" rotation="0" />\n' % (additional_objects[i[0]], material, str(i[1]), str(i[2])))

    for i in final_pig_positions:
        f.write('<Pig type="BasicSmall" material="" x="%s" y="%s" rotation="0" />\n' % (str(i[0]),str(i[1])))

    for i in final_platforms:
        for j in i:
            f.write('<Platform type="Platform" material="" x="%s" y="%s" />\n' % (str(j[0]),str(j[1])))

    for i in final_TNT_positions:
        f.write('<TNT type="" material="" x="%s" y="%s" rotation="0" />\n' % (str(i[0]),str(i[1])))
        
    f.write('</GameObjects>\n')
    f.write('</Level>\n')

    f.close()




# generate levels using input parameters

backup_probability_table_blocks = deepcopy(probability_table_blocks)
backup_materials = deepcopy(materials)

FILE = open("parameters1.txt", 'r')
checker = FILE.readline()
finished_levels = 0
while (checker != ""):
    if checker == "\n":
        checker = FILE.readline()
    else:
        number_levels = int(deepcopy(checker))              # the number of levels to generate
        restricted_combinations = FILE.readline().split(',')      # block type and material combination that are banned from the level
        for i in range(len(restricted_combinations)):
            restricted_combinations[i] = restricted_combinations[i].split()     # if all materials are baned for a block type then do not use that block type
        pig_range = FILE.readline().split(',')
        time_limit = int(FILE.readline())                   # time limit to create the levels, shouldn't be an issue for most generators (approximately an hour for 10 levels)
        checker = FILE.readline()

        restricted_blocks = []                              # block types that cannot be used with any materials
        for key,value in block_names.items():
            completely_restricted = True
            for material in materials:
                if [material,value] not in restricted_combinations:
                    completely_restricted = False
            if completely_restricted == True:
                restricted_blocks.append(value)

        probability_table_blocks = deepcopy(backup_probability_table_blocks)
        trihole_allowed = True
        tri_allowed = True
        cir_allowed = True
        cirsmall_allowed = True
        TNT_allowed = True

        probability_table_blocks = remove_blocks(restricted_blocks)     # remove restricted block types from the structure generation process
        if "TriangleHole" in restricted_blocks:
            trihole_allowed = False
        if "Triangle" in restricted_blocks:
            tri_allowed = False
        if "Circle" in restricted_blocks:
            cir_allowed = False
        if "CircleSmall" in restricted_blocks:
            cirsmall_allowed = False

        for current_level in range(number_levels):

            number_ground_structures = 1                     # number of ground structures
            number_platforms = 0                             # number of platforms (reduced automatically if not enough space)
            number_pigs = 4  # number of pigs (if set too large then can cause program to infinitely loop)

            if (current_level+finished_levels+4) < 10:
                level_name = "0"+str(current_level+finished_levels+4)
            else:
                level_name = str(current_level+finished_levels+4)
            
            number_ground_structures, complete_locations, final_pig_positions, sp_layer = create_ground_structures()
            number_platforms, final_platforms, platform_centers = create_platforms(number_platforms,complete_locations,final_pig_positions)
            complete_locations, final_pig_positions = create_platform_structures(final_platforms, platform_centers, complete_locations, final_pig_positions)
            print("finsih create structure")
            final_pig_positions, removed_pigs = remove_unnecessary_pigs(number_pigs)
            print("finsih removed_pigs")
            final_pig_positions = add_necessary_pigs(number_pigs)
            print("final_pig_positions")
            final_TNT_positions = add_TNT(removed_pigs)
            print("final_TNT_positions")
            number_birds = 2
            possible_trihole_positions, possible_tri_positions, possible_cir_positions, possible_cirsmall_positions = find_additional_block_positions(complete_locations)
            selected_other = add_additional_blocks(possible_trihole_positions, possible_tri_positions, possible_cir_positions, possible_cirsmall_positions)
            #final_materials = set_materials(complete_locations, sp_layer)
            write_level_xml(complete_locations, selected_other, final_pig_positions, final_TNT_positions, final_platforms, number_birds, level_name, restricted_combinations)
        finished_levels = finished_levels + number_levels



    
