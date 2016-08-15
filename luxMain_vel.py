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
    route_list = ["route1","route2","route3","route4"]
    edgeIDs = get_all_edges(route_list)
    sharedEdgesID = get_shared_edges(route_list[0],route_list[1])


    #open .csv file to write mean value into, for plotting
    csv_file = file ("./gridlockNumber(4)_1244.csv","w")
    writer = csv.writer(csv_file, quoting = csv.QUOTE_ALL)
    
    while traci.simulation.getMinExpectedNumber() > 0 and simCount < MAX_SIMULATION_STEPS:
        # SIMULATE THE SYSTEM
        step = step + 1
        simCount = simCount + 1
        traci.simulationStep()
    
        if step < SIMULATIN_STEP_COUNT:
            continue
        else:
            step = 0

        cleanMeasurements = sensors.generateCleanSensorMeasurements(edgeIDs)
        travelTimeEstimate = estimator.updateEstimate(cleanMeasurements, USE_ROBUST_ESTIMATOR, loop_noise_sigma, route_list)
        #print "===", simCount,"==="#,cleanMeasurements
        #for index in range(len(route_list)) :
            #print travelTimeEstimate[index],"total edges:",len(get_all_edges([route_list[index]]))
        writer.writerow([str(simCount),str(travelTimeEstimate[0].travelTime),str(travelTimeEstimate[1].travelTime),\
                        str(travelTimeEstimate[2].travelTime), str(travelTimeEstimate[3].travelTime),\
                        str(float(travelTimeEstimate[0].gridlock) / 33.0), str(float(travelTimeEstimate[1].gridlock) / 45.0),\
                        str(float(travelTimeEstimate[2].gridlock) / 90.0), str(float(travelTimeEstimate[3].gridlock) / 70.0) ])
        #print "======"

        #totalNumber1 = get_total_number_of_vehicles(route_list[0], sharedEdgesID)
        #totalNumber2 = get_total_number_of_vehicles(route_list[1], sharedEdgesID)

        #print "1:",totalNumber1,"2:",totalNumber2
        #writer.writerow([str(simCount),str(totalNumber1),str(totalNumber2)])

    csv_file.close()
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