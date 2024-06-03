#saves the data that fc_read_data gives through serial port as csv and a binary file
#needs access to serial port so close any other program that uses the esp serial port and reset esp before running

import csv
import serial
import pickle
header = ["temp", "press", "filter_alt", "gyro_x", "gyro_y", "gyro_z", "mag_x", "mag_y", "mag_z", "accel_x", "accel_y", "accel_z", "bat_volt", "time"]


myport = serial.Serial("COM5",9600)

#first initialise the data transaction
myport.write(bytes('A',"ascii"))
mystr = myport.readline().decode().strip()
print(mystr)
no_of_flights = int(mystr[16:])
myport.readline()#ignore the extra newline
flights=[]

for i in range(no_of_flights):
    flight={}
    mystr = myport.readline().decode().strip()
    while ("Flight" not in mystr):
        mystr = myport.readline().decode().strip()
    print(mystr)
    mystr = myport.readline().decode().strip()
    mylist = mystr.split(',')
    flight["initial_press"] = float(mylist[0])
    flight["initial_temp"] = float(mylist[1])
    flight["apogee"] = float(mylist[2])
    flight["apogee_time"] = int(mylist[3])

    print("Initial Pressure: {}Pa\nInitial Temp: {}C\nApogee :{}m".format(mylist[0], mylist[1], mylist[2]))
    x = flight["apogee_time"]//1000000
    y = x//60
    x = x - y * 60
    z = y//60
    y = y - z * 60
    print(f"Apogee Time: {z}hrs {y}mins {x}s")
    myport.readline()#ignore the extra newline

    mystr = myport.readline().decode().strip()
    if 'Failed' in mystr:
        print(mystr)
    else:
        #initialising lists
        for head in header:
            flight[head] = []
            
        while True:
            if (len(mystr) == 0):
                #This flight data ended
                break
            mylist = mystr.split(',')
            flight["temp"].append(float(mylist[0]))
            flight["press"].append(float(mylist[1]))
            flight["filter_alt"].append(float(mylist[2]))
            flight["gyro_x"].append(float(mylist[3]))
            flight["gyro_y"].append(float(mylist[4]))
            flight["gyro_z"].append(float(mylist[5]))
            flight["mag_x"].append(float(mylist[6]))
            flight["mag_y"].append(float(mylist[7]))
            flight["mag_z"].append(float(mylist[8]))
            flight["accel_x"].append(float(mylist[9]))
            flight["accel_y"].append(float(mylist[10]))
            flight["accel_z"].append(float(mylist[11]))
            flight["bat_volt"].append(float(mylist[12]))
            flight["time"].append(int(mylist[13]))
            mystr = myport.readline().decode().strip()
    flights.append(flight)

myport.close()
#saving
for i in range(no_of_flights):
    flight = flights[i]
    print("Saving flight", i)
    fname = input("Enter the name of binary file:")
    f = open(fname+".dat","wb")
    pickle.dump(flight, f)
    f.close()
    fname = input("Enter the name of csv file:")
    f = open(fname+".csv", "w", newline = '')
    writer = csv.writer(f)
    writer.writerow(header)
    for j in range(len(flight["temp"])):
        writer.writerow([flight[head][j] for head in header])
    f.close()
print("Done")

            
