import os, sys
import subprocess
import matplotlib
import time
import Estimator as estimator
import TrafficRouter as router
import Sensors as sensors
#import ReportGenerator as report
import Map
start_time = time.time()
print 'Executing python code ...'
print("--- %s seconds ---" % (time.time() - start_time))
####### ADD "sumo-tools" (and TraCI) to the path
tools_path                      = '../sumo-0.26.0-tools'
sys.path.append(tools_path)
import traci
import csv

####### params ######
'''deviation                       = int(sys.argv[1])
loop_noise_sigma                = float(sys.argv[2])
depart_time                     = int(sys.argv[3])
num_Sybil                       = int(sys.argv[4])
d_prob                          = float(sys.argv[5])'''

####### GUI Options #######
USE_GUI                         = 0   # 0 = no GUI, 1 = open GUI

####### GLOBAL VARIABLES  #######
PORT                            = 8813
EXAMPLE_FILE                    = "../LuSTScenario-master/scenario/LuSTScenario.due.complete.mobility.actuated.sumocfg"
SIMULATIN_STEP_COUNT            = 1     # controls how many simulation steps we skip before reourting
MAX_SIMULATION_STEPS            = 86400
USE_GROUNDTRUTH                 = 0    # <only for routing> used to switch between secure routing (0) and ground truth routing (1)
USE_NOISY                       = 1     # used to choose between noisy (1) and non-noisy (0) measurements
noise_sigma                     = 3  # variance of noise added to velocities
USE_SYBIL                       = 1 # 1 for sybil, 0 for 
USE_ROBUST_ESTIMATOR            = 1 # 0 for non robust estimation, 1 for robust (SMT chain)
attackType                      = 'congest_highway'
loop_noise_sigma                = 0.1

def get_all_edges(route_list) :
    # finds all of the edges for the routes, returns a list of edges here
    # in order to be more efficient when checking, converting the list into a set
    edgeIDs = set()
    for route in route_list :
        for edge in traci.route.getEdges(route) :
            if edge not in edgeIDs :
                edgeIDs.add(edge)
    return edgeIDs

def get_shared_edges(route1, route2) :
    sharedEdgesID = set()
    route1EdgesID = set(traci.route.getEdges(route1))
    route2EdgesID = set(traci.route.getEdges(route2))
    for edge in route1EdgesID :
        if edge in route2EdgesID :
            sharedEdgesID.add(edge)
    return sharedEdgesID

def get_total_number_of_vehicles(route, sharedEdgesID) :
    uniqueEdgesID = set(traci.route.getEdges(route)) - sharedEdgesID
    totalNumber = 0
    for edge in uniqueEdgesID :
        totalNumber += traci.edge.getLastStepVehicleNumber(edge)
    return totalNumber

####### EXECUTE TraCI CONTROL LOOP #######
def run():
    traci.init(PORT)
    step = 0
    simCount = 0
    

    # in the initial bologna scenario, we have only 2 routes
    #route_list = ["route1","route2","route3","route4"]
    edgeIDs = traci.edge.getIDList()#get_all_edges(route_list)
    #sharedEdgesID = get_shared_edges(route_list[0],route_list[1])
    edgeInfos = {}

    csv_file = file ("./significance&probOfCongestion_EachEdge_0815-12:51.csv","w")
    save_measurements = file ("./saved_measurements.csv","w")
    process_track = file ("./simCount.csv","w")

    writer = csv.writer(csv_file, quoting = csv.QUOTE_ALL)
    save_measurements_writer = csv.writer(save_measurements, quoting = csv.QUOTE_ALL)
    process_track_writer = csv.writer(process_track, quoting = csv.QUOTE_ALL)
    #open .csv file to write mean value into, for plotting
    #csv_file = file ("./gridlockNumber(00&06)_0211.csv","w")
    #writer = csv.writer(csv_file, quoting = csv.QUOTE_ALL)
    
    while traci.simulation.getMinExpectedNumber() > 0 and simCount < MAX_SIMULATION_STEPS:
        # SIMULATE THE SYSTEM
        step = step + 1
        simCount = simCount + 1
        traci.simulationStep()
    
        if step < SIMULATIN_STEP_COUNT:
            continue
        else:
            step = 0
        process_track_writer.writerow([str(simCount)])

        # probability of congestions..
        # significance (number of cars / mile)

        print simCount,"-",str(time.time() - start_time)
        measurements = sensors.generateCleanSensorMeasurements(edgeIDs)
        save_measurements_writer.writerow([str(simCount),str(measurements)])

        numberOfEdges = len(measurements)

        if simCount == 1 :
            for edgeCounter in range(0, numberOfEdges):
                edgeInfos[edgeIDs[edgeCounter]] = {}
                edgeInfos[edgeIDs[edgeCounter]]["significance"] = 0
                edgeInfos[edgeIDs[edgeCounter]]["gridlock"] = 0


        for edgeCounter in range(0, numberOfEdges):
                [significance, gridlock] = estimator.get_edge_infos(measurements[edgeCounter].measurements, \
                    measurements[edgeCounter].id, measurements[edgeCounter].MaxSpeed, measurements[edgeCounter].length)
                edgeInfos[edgeIDs[edgeCounter]]["significance"] += significance
                edgeInfos[edgeIDs[edgeCounter]]["gridlock"] += gridlock
   
        writer.writerow(["======"+str(simCount)+"======"])
        for edgeCounter in range(0, numberOfEdges):
            if edgeIDs[edgeCounter][0:2] == "-3" or edgeIDs[edgeCounter][0:3] == "--3" :
                writer.writerow([str(edgeIDs[edgeCounter]),str(edgeInfos[edgeIDs[edgeCounter]]["significance"]),\
                                                        str(edgeInfos[edgeIDs[edgeCounter]]["gridlock"])])
            

        #travelTimeEstimate = estimator.updateEstimate(cleanMeasurements, USE_ROBUST_ESTIMATOR, loop_noise_sigma, route_list)
        #print travelTimeEstimate
        #print "===", simCount,"==="#,cleanMeasurements
        #for index in range(len(route_list)) :
            #print travelTimeEstimate[index],"total edges:",len(get_all_edges([route_list[index]]))
        #writer.writerow([str(simCount),str(travelTimeEstimate[0].travelTime),str(travelTimeEstimate[1].travelTime),\
                        #str(float(travelTimeEstimate[0].gridlock) / 33.0), str(float(travelTimeEstimate[1].gridlock) / 45.0) ])
        #print "======"

        #totalNumber1 = get_total_number_of_vehicles(route_list[0], sharedEdgesID)
        #totalNumber2 = get_total_number_of_vehicles(route_list[1], sharedEdgesID)

        #print "1:",totalNumber1,"2:",totalNumber2
        #writer.writerow([str(simCount),str(totalNumber1),str(totalNumber2)])

    csv_file.close()
    save_measurements.close()
    process_track.close()
    traci.close()
    sys.stdout.flush()


####### MAIN ENTRY #######
if __name__ == "__main__":
    if(USE_GUI == 0):
        sumoBinary = "sumo"
    else:
        sumoBinary = "sumo-gui"
    out_file  = "./output.xml"
    sumoProcess = subprocess.Popen([sumoBinary, "-c", EXAMPLE_FILE, "--remote-port", str(PORT), '--tripinfo-output', \
                                    out_file ], stdout=sys.stdout, stderr=sys.stderr)
    run()
    sumoProcess.wait()
    print 'End'
    print("--- %s seconds ---" % (time.time() - start_time))