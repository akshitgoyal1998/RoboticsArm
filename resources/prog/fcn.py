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
##initial_joint_positions = [Vector3(12, 0, 17), Vector3(12, 0, 75), Vector3(36, 0, 203), Vector3(160, 0, 203),Vector3(213, 0,203)]
##initial_joint_positions = [Vector3(0, 0, 0), Vector3(0, 0, 58), Vector3(24, 0, 186), Vector3(148, 0, 186),Vector3(201, 0,186)]
initial_joint_positions = [Vector3(0, 0, 0), Vector3(25, 0, 0), Vector3(35, 0, 50), Vector3(40, 0, 100),Vector3(45, 0,100)]
tolerance = 10

fab = pyfabrik.Fabrik3D(initial_joint_positions, tolerance)
fab.move_to(Vector3(target[0],target[1],target[2]))

## writing resulting joint angles value to angle.txt file
fwr = open("C:/Users/Dell/Desktop/BTP2/resources/prog/angle.txt","w")
j=1;
for y in fab.angles_deg:
    #if (j==4) : y=y-180;
    if(j==3) : y=y-180;
    y= y*(3.14/180);
    #print (y)
    fwr.write(str(y))
    fwr.write("\n")
    j=j+1;
fwr.close();
