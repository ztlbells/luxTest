# June 15, 2016
# for sybil attacks, we start with edges 4H to 5H, and 4S to 5S

import sys
tools_path                      = '../sumo-0.26.0-tools'
sys.path.append(tools_path)
import traci
import numpy as np
import Map

# COMMMENT: need to store the car type to add attacks depending on the car type
# Need to have a code convention for sybil attack: use Sybil_ before vehicle id 

############### clean measurements #################
def generateCleanSensorMeasurements(edgeIDs):
    groundTruthMeasurements = list()
    # Collect ground truth measurmenets for each vehicle per link
    for edgeID in edgeIDs :
        if edgeID[0:2] == "-3" or edgeID[0:3] == "--3":
            edgeMeasurement = list()
            for vehID in traci.edge.getLastStepVehicleIDs(edgeID):
            #print "append", vehID,"to",edgeID
                edgeMeasurement.append(Map.Map(id = vehID, type=traci.vehicle.getTypeID(vehID), pos= traci.vehicle.getLanePosition(vehID), \
                vel=traci.vehicle.getSpeed(vehID)))
            groundTruthMeasurements.append(Map.Map(id = edgeID, measurements = edgeMeasurement, length = traci.lane.getLength(edgeID+"_0"), \
                                                MaxSpeed = traci.lane.getMaxSpeed(edgeID+"_0"), gridLock = 0, significance = 0))

    return groundTruthMeasurements
           
############## for simulating sybil attacks ###################
def generateSybilVelo(attackType, parentVelo, edgeID, deviation):
    deviation = float(deviation)
    sybilVelo = min(parentVelo + deviation, traci.lane.getMaxSpeed(edgeID+"_0"))  # report faster
    return sybilVelo
        


def generateDishonestSybilSensorMeasurements(attackType, deviation, num_Sybil, attacked_edges_list, edgeIDs, dishonesty):
    # num_sybil is the number of sybil cars inserted
    # adds deviation to the velocity of the dishonest car also, along with Sybil cars
    measurements = list()
    # Collect ground truth measurmenets for each vehicle per link
    for edgeID in edgeIDs:
        edgeMeasurement = list()

        # before getting the type ID, we should sample the vehicles first
        # 0 = honest, 1 = dishonest, 2 = sybil (will be added later)
        DH = np.random.binomial(2, dishonesty, len(traci.edge.getLastStepVehicleIDs(edgeID)))

        

        for index in range(len(traci.edge.getLastStepVehicleIDs(edgeID))):

            vehID = traci.edge.getLastStepVehicleIDs(edgeID)[index]
            SUMO_velocity = traci.vehicle.getSpeed(vehID) # get speed from SUMO 
            if DH[index] == 0 :
                type_ID = "CarHonest" # get type ID
            else :
                type_ID = "CarDishonest"
            SUMO_pos = traci.vehicle.getLanePosition(vehID)

            if type_ID == 'CarDishonest' and (edgeID in attacked_edges_list):

                # first generate the sybil car details
                sybil_vehID = 'Sybil_' + vehID   # add a sybil_ in before dishonest's id
                sybil_type = 'CarSybil'
                sybil_pos = SUMO_pos
                sybil_vel = generateSybilVelo(attackType, SUMO_velocity , edgeID , deviation )
                dishonest_vel = sybil_vel

                # first add the dishonest car in the list with the new velocity
                edgeMeasurement.append(Map.Map(id = vehID, type=type_ID, pos= SUMO_pos, vel=dishonest_vel, actual_vel=SUMO_velocity))

                # vel is the reported vel, actual_vel is the true velocity
                # now add the sybil cars one step at a time 
                added_Sybil_cars = 0

                #print vehID," is dishonest so it creates ", num_Sybil," sybil cars on 150018085#0"

                while added_Sybil_cars < num_Sybil:                   
                    added_sybil_id = sybil_vehID + '_' + str(added_Sybil_cars+1)
                    edgeMeasurement.append(Map.Map(id = added_sybil_id, type = sybil_type, pos= sybil_pos, vel =  sybil_vel, actual_vel= sybil_vel ) )
                    #print added_sybil_id," velocity :",sybil_vel,"SUMO_velocity :",SUMO_velocity
                    added_Sybil_cars += 1 

                    #print "sybil car added"  
                    #print edgeMeasurement

            else:
                edgeMeasurement.append(Map.Map(id = vehID, type=type_ID, pos= SUMO_pos, vel= SUMO_velocity, actual_vel=SUMO_velocity ))

            
        
            #print "for edge ",edgeID,"edge measurements:",edgeMeasurement     
        measurements.append(Map.Map(id = edgeID, measurements = edgeMeasurement))

    #if flag == 1 :
        #print measurements


        
    return measurements
    
 