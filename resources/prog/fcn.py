import os,pyfabrik;
from vectormath import Vector3
f= open("C:/Users/Dell/Desktop/BTP2/resources/prog/temp.txt","r")


## reading from temp file target values from user
i=0;
target = [];
for x in f:
    target.insert(i,int(x));
    i=i+1
f.close();


## setting initial values of joints
initial_joint_positions = [Vector3(10, 0, 0), Vector3(20, 0, 0), Vector3(30, 0, 0), Vector3(40, 0, 0)]
tolerance = 0.01

fab = pyfabrik.Fabrik3D(initial_joint_positions, tolerance)
fab.move_to(Vector3(target[0],target[1],target[2]))

## writing resulting joint angles value to angle.txt file
fwr = open("C:/Users/Dell/Desktop/BTP2/resources/prog/angle.txt","w")
for y in fab.angles_deg:
    y= y*(3.14/180);
    fwr.write(str(y))
    fwr.write("\n")
fwr.close();
