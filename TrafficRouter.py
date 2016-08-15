#router, based upon the travelTime
def travel_time_router(travelTime_estimate) :
	#print "travelTime_estimate",travelTime_estimate
	if travelTime_estimate[0]['travelTime'] <= travelTime_estimate[1]['travelTime'] :
		return "route1"
	else :
		return "route2"
