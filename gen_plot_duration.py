
import matplotlib.pyplot as plt
import csv
import os, sys

#gen plot
plt.close()
plt.style.use('ggplot')

output_filename = "./gridlockNumber(00&06)_0211.csv"
output_file 		= file (output_filename, "r")

time=[]
#totalNumber1 = []
#totalNumber2 = []
travelTime1 = []
travelTime2 = []
gridlockNumber1 = []
gridlockNumber2 = []

reader = csv.reader(output_file)

gridlock1 = 0
gridlock2 = 0 
for line in reader :
    if line != [] :
    	time.append(float(line[0]) / 3600.0)
        travelTime1.append(float(line[1]) / 3600.0)#(min(float(line[1]), 1000.0))
        travelTime2.append(float(line[2]) / 3600.0)#(min(float(line[2]), 1000.0))
        gridlockNumber1.append(float(line[3]) * 33)
        if float(line[3]) > 0.0 :
        	gridlock1 += 1
        if float(line[4]) > 0.0 :
        	gridlock2 += 1
        gridlockNumber2.append(float(line[4]) * 45)
        #print line[0],line[1],line[2]

output_file.close()

print gridlock1, gridlock2
print float(gridlock1)/86400.0
print float(gridlock2)/86400.0
#plt.figure()
#title_name = 'Luxemburg - totalNumberOfVehicles for 2 routes'
#title_name = 'Luxemburg - number of congestions for arterial roads'
#plt.title(title_name)


#plt.plot(time, travelTime1, label = 'highway', linestyle = '-', color = "r")
#plt.plot(time, travelTime2, label = 'arterial roads',linestyle = '-', color = "c")

#plt.plot(time, gridlockNumber1, label = 'highway', linestyle = '-', color = "r")
#plt.plot(time, gridlockNumber2, label = 'arterial roads',linestyle = '-', color = "c")

#plt.legend(fontsize = 10) 

#plt.xlabel('time(unit:hr)', fontsize = 15)
#plt.ylabel('number of cogestions', fontsize = 15)

#plt.show()