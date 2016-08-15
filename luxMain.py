import os, sys
import subprocess
import matplotlib
import time
start_time = time.time()
print 'Executing python code ...'
print("--- %s seconds ---" % (time.time() - start_time))
####### ADD "sumo-tools" (and TraCI) to the path
tools_path                      = '../sumo-0.26.0-tools'
sys.path.append(tools_path)
import traci
import csv

####### GUI Options
USE_GUI                         = 0   # 0 = no GUI, 1 = open GUI

####### GLOBAL VARIABLES  #######
PORT                            = 8813
EXAMPLE_FILE                    = "../LuSTScenario-master/scenario/LuSTScenario.due.complete.mobility.actuated.sumocfg"
SIMULATIN_STEP_COUNT            = 1     # controls how many simulation steps we skip before reourting
MAX_SIMULATION_STEPS            = 86400

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
    route_list = ["route1","route2"]
    edgeIDs = get_all_edges(route_list)
    sharedEdgesID = get_shared_edges(route_list[0],route_list[1])


    #open .csv file to write mean value into, for plotting
    csv_file = file ("./totalNumberOfVehicles(00&06)_1253.csv","w")
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


        totalNumber1 = get_total_number_of_vehicles(route_list[0], sharedEdgesID)
        totalNumber2 = get_total_number_of_vehicles(route_list[1], sharedEdgesID)

        #print "1:",totalNumber1,"2:",totalNumber2
        writer.writerow([str(simCount),str(totalNumber1),str(totalNumber2)])

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