import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import serial
import time
import keyboard
import os

# Create a figure and axis
fig, ax = plt.subplots()

# Set the background color to black
ax.set_facecolor('none')

def stop_program(e):
    print("Stopping the program...")
    # Perform any cleanup or termination tasks here
    # Then exit the program
    
    os._exit(1)
# keyboard.on_press(stop_program)


# Configure the serial port
serial_port = serial.Serial('COM5', 9600)  # Replace 'COMx' with your actual serial port




plt.ion()
fig, ax = plt.subplots(2,3,figsize = (8,8))
fig.subplots_adjust(hspace= 0.3, wspace= 0.3)
x = []
box_x = 0.5
box_y = 1.05


box = plt.Rectangle((box_x, box_y), width=0.2, height=0.1, fill=False, color='red', linewidth=2)
fig.patches.append(box)
fig.patch.set_facecolor('none')
#plotted
alt_y = []
pitch_y = []
roll_y = []
yaw_y = []
lin_z = []
temp = []

#non-plotted
lat_deg = []
lat_min = []
lat_sec = []
long_deg = []
long_min = []
long_sec = []





line, = ax[0][0].plot(x,alt_y)
line_2, = ax[0][1].plot(x,pitch_y)
line_3, = ax[0][2].plot(x,roll_y)
line_4, = ax[1][0].plot(x,yaw_y)
line_5, = ax[1][1].plot(x,lin_z)
line_6,  = ax[1][2].plot(x,temp)


output_folder= "C:/Users/karth/Unity Projects/Abhyuday/Assets"


def save_frame(): 

    plt.savefig("C:/Users/karth/Unity Projects/Abhyuday/Assets/frame.png", bbox_inches = 'tight', pad_inches = 0.1,transparent=True)


def update(i):
    x.append(i)
    try:
        while True:
            data = serial_port.readline().decode('utf8')
            if data:
                data=data.strip()
                datastring = data.split(',')
                print(datastring)
                alt_y.append(eval(datastring[0]))
                pitch_y.append(eval(datastring[1]))
                roll_y.append(eval(datastring[2]))
                yaw_y.append(eval(datastring[4]))
                lin_z.append(eval(datastring[5]))
                temp.append(eval(datastring[12]))
                break
    except KeyboardInterrupt:
        serial_port.close()
    line.set_data(x,alt_y)
    ax[0][0].relim()
    ax[0][0].autoscale_view()
    text = f'Altitude: {datastring[0]}'
    ax[0][0].annotate(text, xy=(0.5, 1.05), xycoords='axes fraction', fontsize=12, ha='center', va='center', color='white',bbox=dict(boxstyle='square,pad = 0.3', edgecolor='white', facecolor='black'))
    
    
    
                  
                     
    line_2.set_data(x,pitch_y)
    ax[0][1].relim()
    ax[0][1].autoscale_view()
    text = f'Pitch:                       {datastring[1]}'
    ax[0][1].annotate(text, xy=(0.5, 1.05), xycoords='axes fraction', fontsize=12, ha='center', va='center', color='white',bbox=dict(boxstyle='square,pad = 0.3', edgecolor='white', facecolor='black'))
    
    line_3.set_data(x,roll_y)
    ax[0][2].relim()
    ax[0][2].autoscale_view()
    text = (f'Roll:                       {datastring[2]}')
    ax[0][2].annotate(text, xy=(0.5, 1.05), xycoords='axes fraction', fontsize=12, ha='center', va='center', color='white',bbox=dict(boxstyle='square,pad = 0.3', edgecolor='white', facecolor='black'))
    
    
    
    line_4.set_data(x,yaw_y)
    ax[1][0].relim()
    ax[1][0].autoscale_view()
    text = (f'Yaw:                        {datastring[3]}')
    ax[1][0].annotate(text, xy=(0.5, 1.05), xycoords='axes fraction', fontsize=12, ha='center', va='center', color='white',bbox=dict(boxstyle='square,pad = 0.3', edgecolor='white', facecolor='black'))
    
    line_5.set_data(x,lin_z)
    ax[1][1].relim()
    ax[1][1].autoscale_view()
    text = (f'Linear Acceleration:         {datastring[4]}')
    ax[1][1].annotate(text, xy=(0.5, 1.05), xycoords='axes fraction', fontsize=12, ha='center', va='center', color='white',bbox=dict(boxstyle='square,pad = 0.3', edgecolor='white', facecolor='black'))
    
    
    line_6.set_data(x,temp)
    ax[1][2].relim()
    ax[1][2].autoscale_view()
    text = (f'Temperature:                  {datastring[11]}')
    ax[1][2].annotate(text, xy=(0.5, 1.05), xycoords='axes fraction', fontsize=12, ha='center', va='center', color='white',bbox=dict(boxstyle='square,pad = 0.3', edgecolor='white', facecolor='black'))
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    save_frame()
    fig.canvas.draw()
    fig.canvas.flush_events()
    
    
  


plt.grid()





for i in range(200):
    update(i)
    time.sleep(1)

