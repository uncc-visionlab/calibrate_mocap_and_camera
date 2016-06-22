import math
import matplotlib.pyplot as plt
from os import environ
from sys import version_info

if version_info[0] < 3:
    raise "Must be using Python 3"

def quatertoRPY(x,y,z,w):
    roll = math.atan2(2.0*(x*w + y*z), 1 - 2*(z*z + w*w))
    pitch = math.asin(2.0*(x*z - w*y))
    yaw = math.atan2(2.0*(x*y + z*w), 1 - 2*(y*y + z*z))

    if abs(yaw+pi) < 30*pi/180:
        yaw = yaw+2*pi
        
    return([roll,pitch,yaw])


def removeoutliers(poseparam, ulimit=None, llimit=None):
    newposeparam = []
    for i, value in enumerate(poseparam):
        if value is None:
            newposeparam.append(None)
        elif ulimit is not None and value > ulimit:
            newposeparam.append(None)
            if '%' not in strdata[i][0]:
                strdata[i][0] = '%' + strdata[i][0]
        elif llimit is not None and value < llimit:
            newposeparam.append(None)
            if '%' not in strdata[i][0]:
                strdata[i][0] = '%' + strdata[i][0]
        else:
            newposeparam.append(value)
    return(newposeparam)

pi = math.pi

inputfile = open(environ["HOME"] + '/calib_transforms.txt', 'r')
outputfile = open(environ["HOME"] + '/calib_transforms_no_outliers.m', 'w')

strdata = [line.strip("\n").split(" ") for line in inputfile.readlines()[1:]]
data = [[float(value) for value in row] for row in strdata]

x = []
y = []
z = []
roll = []
pitch = []
yaw = []

for row in data:
    x.append(row[0])
    y.append(row[1])
    z.append(row[2])
    
    ro,pit,ya = quatertoRPY(row[3],row[4],row[5],row[6])
    roll.append(ro*180/pi)
    pitch.append(pit*180/pi)
    yaw.append(ya*180/pi)

exit = False

while(exit != True):
        
    plt.subplot(3, 2, 1)
    plt.plot(range(len(x)), x, "b-")
    plt.ylabel("x")
    plt.xlabel("Measurement Index")

    plt.subplot(3, 2, 2)
    plt.plot(range(len(yaw)), yaw, "b-")
    plt.ylabel("yaw")
    plt.xlabel("Measurement Index")
    
    plt.subplot(3, 2, 3)
    plt.plot(range(len(y)), y, "b-")
    plt.ylabel("y")
    plt.xlabel("Measurement Index")
    
    plt.subplot(3, 2, 4)
    plt.plot(range(len(pitch)), pitch, "b-")
    plt.ylabel("pitch")
    plt.xlabel("Measurement Index")

    plt.subplot(3, 2, 5)
    plt.plot(range(len(z)), z, "b-")
    plt.ylabel("z")
    plt.xlabel("Measurement Index")

    plt.subplot(3, 2, 6)
    plt.plot(range(len(roll)), roll, "b-")
    plt.ylabel("roll")
    plt.xlabel("Measurement Index")
    
    plt.show()
    
    userparam = input("Select the parameter you would like to limit (x, y, z, yaw, pitch, roll) or \"exit\" to exit:\n")

    if userparam in ['x', 'y', 'z', 'yaw', 'pitch', 'roll']:
        upper = input("Select an upper limit for the data in " + userparam + " (or return for no limit):\n")
        lower = input("Select a lower limit for the data in " + userparam + " (or return for no limit):\n")
        
        if upper == "": upper = None
        else: upper = float(upper)
        if lower == "": lower = None
        else: lower = float(lower)
        
        if userparam == 'x':
            x = removeoutliers(x, upper, lower)
        
        elif userparam == 'y':
            y = removeoutliers(y, upper, lower)
            
        elif userparam == 'z':
            z = removeoutliers(z, upper, lower)
        
        elif userparam == 'roll':
            roll = removeoutliers(roll, upper, lower)
            
        elif userparam == 'pitch':
            pitch = removeoutliers(pitch, upper, lower)
        
        elif userparam == 'yaw':
            yaw = removeoutliers(yaw, upper, lower)            
    
    elif userparam == "exit":
        exit = True
    
    else: print("Please enter a valid parameter.\n")

outputfile.write("tf_cam_to_rgb_optical_calibration_data=[\n")

for row in strdata:
    outputfile.write(" ".join(row) + "\n")

outputfile.write("]")

print("Changes written to: " + outputfile.name)

inputfile.close()
outputfile.close()