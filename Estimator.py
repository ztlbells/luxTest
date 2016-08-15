import sys
tools_path                      = '../sumo-0.26.0-tools'
sys.path.append(tools_path)
import traci
#import car_graph as smt
import numpy as np

import Map
import csv               
############# zutian's part ###########
def updateEstimate(measurements, USE_ROBUST_ESTIMATOR, loop_noise_sigma, routes_list):
    # update the travel time for each edge, and then add them up, returning a list of travel time for each route
    #print routes_list
    travelTimeEstimate      = list()
    numberOfEdges           = len(measurements)
 
    #print measurements
    for index in range(len(routes_list)) :
            travelTimeEstimate.append(Map.Map(routeId = routes_list[index], travelTime = 0, gridlock = 0, totalLength = 0))

    for edgeCounter in range(0, numberOfEdges):
        for index in range(len(routes_list)) :
            if measurements[edgeCounter].id in traci.route.getEdges(routes_list[index]) :
                travelTimeEstimate[index].travelTime += get_travel_time_edge(measurements[edgeCounter].measurements, measurements[edgeCounter].id,\
                                                        measurements[edgeCounter].MaxSpeed, measurements[edgeCounter].length)[0]
                travelTimeEstimate[index].gridlock += get_travel_time_edge(measurements[edgeCounter].measurements, measurements[edgeCounter].id,\
                                                        measurements[edgeCounter].MaxSpeed, measurements[edgeCounter].length)[1]
                travelTimeEstimate[index].totalLength += measurements[edgeCounter].length

    return travelTimeEstimate

ZERO_VELOCITY = 10**(-7) 

def get_travel_time_edge(edgeMeasurement, edgeID, MaxSpeed, length):

    sum_velo = 0
    gridlock = 0
    #stand_still_flag = 0
    occupancy = len(edgeMeasurement)
    #print "number of lanes :" + str(get_number_of_lane(edge))
    if occupancy == 0 :# get_number_of_lane(edge) :
        #max speed
        average_velo = MaxSpeed #traci.lane.getMaxSpeed(edgeID + "_0")
    else :

        #csv_file = file ("./output_checking_velocity.csv","a")
        #writer = csv.writer(csv_file, quoting = csv.QUOTE_ALL)

        for vehicle_counter in range(0, occupancy):
            sum_velo +=  edgeMeasurement[vehicle_counter].vel

        average_velo = max(sum_velo / occupancy, ZERO_VELOCITY)

    if average_velo <= 0.1 :
        gridlock = 1

    #print edgeID,"-vel:", average_velo

    return [length / average_velo, gridlock]#, stand_still_flag

def get_edge_infos(edgeMeasurement, edgeID, MaxSpeed, length):
    sum_velo = 0
    gridlock = 0
    occupancy = len(edgeMeasurement)
    if occupancy == 0 :
        average_velo = MaxSpeed 
    else :
        for vehicle_counter in range(0, occupancy):
            sum_velo +=  edgeMeasurement[vehicle_counter].vel

        average_velo = max(sum_velo / occupancy, ZERO_VELOCITY)

    if average_velo <= 0.1 :
        gridlock = 1

    significance = float(occupancy) / float(length)
    return [significance, gridlock]



############# original codes ###########

def estimateTravelTimePerEdge(maxSpeed, edgeDist, edgeMeasurement):
    velo_tolerance = 10**(-7)
    avg_velo = estimate_avg_velocity(maxSpeed, edgeMeasurement)
    avg_time = float(edgeDist)/max(avg_velo, velo_tolerance)
    return avg_time
    
def robust_estimateTravelTimePerEdge(maxSpeed, edgeDist, edgeMeasurement, loop_noise_sigma):
    velo_tolerance = 10**(-7)
    avg_velo = robust_estimate_avg_velocity(maxSpeed, edgeMeasurement, loop_noise_sigma)
    avg_time = float(edgeDist)/max(avg_velo, velo_tolerance)
    return avg_time
    
    
def estimate_avg_velocity(maxSpeed, edgeMeasurement):
    occupancy = len(edgeMeasurement)
    if occupancy == 0:
        return maxSpeed
    else:
        sum_velo = 0
        for vehicle_counter in range(0, occupancy):
            sum_velo +=  edgeMeasurement[vehicle_counter].vel
        avg_velo = sum_velo/float(occupancy)
        return max(avg_velo,0)


def actual_avg_velocity(edgeMeasurement):
    actual_sum_velo = 0
    observed_sum_velo = 0
    total = 0
    for i in range(0, len(edgeMeasurement)):
        observed_sum_velo += edgeMeasurement[i]['vel'] 
        if edgeMeasurement[i]['type'] != 'CarSybil':
            total += 1
            actual_sum_velo  +=  edgeMeasurement[i]['actual_vel'] 
    if total == 0:
        print 'No cars in the edge'
    actual_avg_velo = actual_sum_velo/float(total)
    observed_avg_velo = observed_sum_velo/float(len(edgeMeasurement))
    return [actual_avg_velo, observed_avg_velo]
        

def robust_estimate_avg_velocity(maxSpeed, edgeMeasurement, loop_noise_sigma):
    occupancy = len(edgeMeasurement)
    if occupancy <= 1:  # NOTE CHANGE THIS !!!!!!!!!!!!!!!!!!!!!!!!!!!
        return maxSpeed
    else:
        [car_list, assignment_list, velocity_ack_list] = smt.assignments(edgeMeasurement)
        #print 'total assignments = ', len(assignment_list)
        total_cars_measured = len(car_list)
        min_residue = 10**7 # max possible residue
        min_index  = -1
        [actual_avg_velo, observed_avg_velo] = actual_avg_velocity(edgeMeasurement)
        # add noise to it ?? # remove this later
        noise  = np.random.normal(0, loop_noise_sigma, 1)
        actual_avg_velo += noise # add noise of sigma 3
        ###################################################### remove this later ######
        candidate_avg_velo_list = np.zeros(len(assignment_list))
        for i in range(0, len(assignment_list)):
            #print 'checking asssignment', i
            sum_velo = 0
            estimated_total_cars = 0
            for j in range(0, total_cars_measured):
                if float(assignment_list[i][j])  == 0:
                    addition_flag = 0
                    secure_velo = car_list[j]['vel']
                else:
                    [addition_flag, secure_velo] = compute_velo_flag(assignment_list[i], velocity_ack_list[j], car_list, j) 
                
                sum_velo += secure_velo*addition_flag 
                estimated_total_cars += addition_flag # add cars which are real and pass vel. check
            if estimated_total_cars == 0:
                candidate_avg_velo_list[i] = maxSpeed
            else:
                candidate_avg_velo_list[i] = sum_velo/estimated_total_cars
            residue = abs(actual_avg_velo - candidate_avg_velo_list[i])
            if residue < min_residue:
                min_residue = residue
                min_index = i
        if min_index == -1:
            print 'Warning: subset selection failed...'
            return observed_avg_velo
        else:
            return candidate_avg_velo_list[min_index]
            
            
def compute_velo_flag(assignment_i, velocity_ack_list_j, car_list, j):
    valid_neighbors_index_list = []
    #purged_ack_list = [] 
    purged_velo_list = []
    back_real_index = find_nearest_real(j, -1 , assignment_i)
    front_real_index = find_nearest_real(j, 1 , assignment_i)
    if back_real_index != -1:
        valid_neighbors_index_list.append(back_real_index)
    if front_real_index != -1:
        valid_neighbors_index_list.append(front_real_index)
    if len(valid_neighbors_index_list) == 0:
        return [0, car_list[j]['vel']] # return the velocity flag = 0 and reported velocity  
    for ack in velocity_ack_list_j:
        if ack.reported_by in valid_neighbors_index_list:
            #purged_ack_list.append(ack)
            purged_velo_list.append(ack.reported_velocity)
    if len(purged_velo_list) == 0:
        return [0, car_list[j]['vel']] # return the velocity flag = 0 and reported velocity
    purged_velo_list.append(car_list[j]['vel']) # add the self reported velocity to the list
    [flag, final_velo] = velo_majority(purged_velo_list)
    return [flag, final_velo]
            
def velo_majority(velo_list):
    if len(velo_list) == 1:
        print 'warning: singleton set sent to majority function!'
    counts_dict = {}
    for velo in velo_list:
        if velo in counts_dict:
            counts_dict[velo] += 1
        else:
            counts_dict[velo] = 1
    if len(counts_dict) == 1:
        #print 'absolute majority with', counts_dict[velo_list[0]], 'votes'
        return [1, velo_list[0] ]
    counts_list = []
    for key in counts_dict:
        pair = [key, counts_dict[key] ]
        counts_list.append(pair)
    sorted_counts_list = sorted(counts_list, key=lambda x: x[1], reverse=True)
    if sorted_counts_list[0][1] > sorted_counts_list[1][1]:
        return [1, sorted_counts_list[0][0] ]
    else:
        return [0, sorted_counts_list[0][0] ]
           
     
def find_nearest_real(current_index, direction , assignment_i):
    index = current_index + direction
    while index >= 0 and index < len(assignment_i):
        if assignment_i[index] == True:
            return index
        else:
            index = index + direction # if direction is +1 then increment, if -1 then decrement
    return -1

##### testing ########
#A = {'measurements': [{'actual_vel': 4.147112735779956, 'vel': 4.147112735779956, 'type': 'CarHonest', 'pos': 5.56546033990106, 'id': 'f1.79'}, {'actual_vel': 4.757895925198682, 'vel': 4.757895925198682, 'type': 'CarHonest', 'pos': 20.03420357868449, 'id': 'f1.78'}, {'actual_vel': 3.964054173557088, 'vel': 1000.964054173557088, 'type': 'CarDishonest', 'pos': 33.22525508193114, 'id': 'f2.23'},{'actual_vel': 1000.964054173557088, 'vel': 1000.964054173557088 , 'type': 'CarSybil', 'pos': 33.22525508193114, 'id': 'Sybil_f2.23'}, {'actual_vel': 4.090966165531427, 'vel': 4.090966165531427, 'type': 'CarHonest', 'pos': 46.94966694839486, 'id': 'f1.77'}],'id': 'edge_4S_5S'}
#A = {'measurements': [{'actual_vel': 4.147112735779956, 'vel': 4.147112735779956, 'type': 'CarHonest', 'pos': 5.56546033990106, 'id': 'f1.79'}, {'actual_vel': 4.757895925198682, 'vel': 4.757895925198682, 'type': 'CarHonest', 'pos': 20.03420357868449, 'id': 'f1.78'}, {'actual_vel': 4.090966165531427, 'vel': 4.090966165531427, 'type': 'CarHonest', 'pos': 46.94966694839486, 'id': 'f1.77'}],'id': 'edge_4S_5S'}
#print robust_estimate_avg_velocity(20.0, A['measurements'], 1)
# the front most car is actually getting voted out 





