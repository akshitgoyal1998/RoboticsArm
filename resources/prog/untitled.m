clear;clc
fprintf('Inverse Kinematics Solver By Joao Matos \n')
fprintf('DASL at UNLV 2016 \n ') 
fprintf('Damped Least Squares Method ') 
fprintf('\n')
%Ask for the desired position xt=input('Enter the desired x position') yt=input('Enter the desired y position') zt=input('Enter the desired z position')
%Dasl arm lenghts L1=-11; L2=15; L3=10; L4=-21; L5=-8;
%Initial position th1 =0; th2=0; th3=0; th4=0; th5=0;
%initializing the variables to control the loop ev1=30; ev2=30; ev3=30; i=1; n=0; status=true; method=true;
%Ask if the user want orientation Control or not choosemethod=input('Press 1 to orientation Control , Or 2 to no orientation')
if (choosemethod==1)
     useorientation=true;
      NOuseorientation=false;
     
else
      NOuseorientation=true;
       useorientation=false;
end
abort=false; 
if (useorientation) 
    while (status)
 if (n>1000)
      fprintf('More than 1000 iterations')
      status=false;
      abort=true;
  end
  
  if(ev1<0.8 && ev2<0.8 && ev3<0.8)
      status=false;
  end 
     
%Homogeneous transforms from link to link
A1=[cosd(th1) 0 -sind(th1) 0;sind(th1) 0 cosd(th1) 0;0 -1 0 L1;0 0 0 1];
A2=[cosd(th2) -sind(th2) 0 L2*cosd(th2);sind(th2) cosd(th2) 0 L2*sind(th2);0 0 1 0;0 0 0 1];
A3=[cosd(th3) -sind(th3) 0 L3*cosd(th3);sind(th3) cosd(th3) 0 L3*sind(th3);0 0 1 0;0 0 0 1];
A4=[cosd(th4) 0 sind(th4) 0;sind(th4) 0 -cosd(th4) 0;0 1 0 0 ;0 0 0 1];
A5=[cosd(th5) -sind(th5) 0 0;sind(th5) cosd(th5) 0 0;0 0 1 L4+L5;0 0 0 1];
%Auxiliar transforms to calculate the jacobian A12=A1*A2;
A123=A1*A2*A3;
A1234=A1*A2*A3*A4;
A12345=A1*A2*A3*A4*A5;
% Calculating Jw (Jacobian angular velocity ) % Jw is (3xn) where n is the number of joints
%column1 0A0 , default k vector (0,0,1) jw(1,1)=0; jw(2,1)=0; jw(3,1)=1;
%COLUMN2 0A1 get value from A1 (3rd column rows 1,2,3) jw(1,2)=A1(1,3); jw(2,2)=A1(2,3); jw(3,2)=A1(3,3);
%COLUMN3 0A2 get value from A1.A2 (3rd column rows 1,2,3) jw(1,3)=A12(1,3); jw(2,3)=A12(2,3); jw(3,3)=A12(3,3);
%COLUMN4 0A3 get value from A1.A2.A3 (3rd column rows 1,2,3)
jw(1,4)=A123(1,3); jw(2,4)=A123(2,3); jw(3,4)=A123(3,3);
%COLUMN5 0A4 get value from A1.A2.A3.A4 (3rd column rows 1,2,3) jw(1,5)=A1234(1,3); jw(2,5)=A1234(2,3); jw(3,5)=A1234(3,3);
%Calculating Jv ( Jacobian linear velocity )
%Distance from end effector frame to base frame %Get value from 0A5 (4th column rows1,2,3) oeff(1,1)=A12345(1,4); oeff(2,1)=A12345(2,4); oeff(3,1)=A12345(3,4);
%Distance from base frame to base frame (=0) o00(1,1)=0; o00(2,1)=0; o00(3,1)=0;
%Distance from joint 1 frame to base frame %Get value from 0A1 (4th column rows1,2,3) o01(1,1)=A1(1,4); o01(2,1)=A1(2,4); o01(3,1)=A1(3,4);
%Distance from joint 2 frame to base frame %Get value from A1.A2 (4th column rows1,2,3) o02(1,1)=A12(1,4); o02(2,1)=A12(2,4); o02(3,1)=A12(3,4);
%Distance from joint 3 frame to base frame %Get value from A1.A2.A3 (4th column rows1,2,3) o03(1,1)=A123(1,4); o03(2,1)=A123(2,4); o03(3,1)=A123(3,4);
%Distance from joint 4 frame to base frame %Get value from A1.A2.A3.A4 (4th column rows1,2,3) o04(1,1)=A1234(1,4); o04(2,1)=A1234(2,4); o04(3,1)=A1234(3,4);
%distance in each column of Jv ( oeff - o0(i-1)) %calculate for the 5 columns rc1=oeff-o00; rc2=oeff-o01; rc3=oeff-o02; rc4=oeff-o03; rc5=oeff-o04;
%columns from jw to do the cross with the distance jwc1(1,1)=jw(1,1); jwc1(2,1)=jw(2,1); jwc1(3,1)=jw(3,1);
jwc2(1,1)=jw(1,2); jwc2(2,1)=jw(2,2); jwc2(3,1)=jw(3,2);
jwc3(1,1)=jw(1,3); jwc3(2,1)=jw(2,3); jwc3(3,1)=jw(3,3);
jwc4(1,1)=jw(1,4); jwc4(2,1)=jw(2,4); jwc4(3,1)=jw(3,4);
jwc5(1,1)=jw(1,5); jwc5(2,1)=jw(2,5); jwc5(3,1)=jw(3,5);
%Constructing Jv
%column1 %Get the cross product jv1=cross(jwc1,rc1); %fill the jv jv(1,1)=jv1(1,1); jv(2,1)=jv1(2,1); jv(3,1)=jv1(3,1);
%column2 %Get the cross product jv2=cross(jwc2,rc2); %fill the jv jv(1,2)=jv2(1,1); jv(2,2)=jv2(2,1); jv(3,2)=jv2(3,1);
%column3 %Get the cross product jv3=cross(jwc3,rc3); %fill the jv jv(1,3)=jv3(1,1); jv(2,3)=jv3(2,1); jv(3,3)=jv3(3,1);
%column4 %Get the cross product jv4=cross(jwc4,rc4); %fill the jv jv(1,4)=jv4(1,1); jv(2,4)=jv4(2,1); jv(3,4)=jv4(3,1);
%column5 %Get the cross product jv5=cross(jwc5,rc5); %fill the jv jv(1,5)=jv5(1,1); jv(2,5)=jv5(2,1); jv(3,5)=jv5(3,1);
%Final Jacobian %The upper part is the Jacobian linear velocity (columns 1~5 , rows 1~3) %The lower part is the Jacobian angular velocity (columns 1~5, rows 4~6)
%column1 %Upper part (from jv) Jacobian(1,1)=jv(1,1); Jacobian(2,1)=jv(2,1); Jacobian(3,1)=jv(3,1); %Lower part (from jw) Jacobian(4,1)=jw(1,1); Jacobian(5,1)=jw(2,1); Jacobian(6,1)=jw(3,1);
%column2 %Upper part (from jv) Jacobian(1,2)=jv(1,2); Jacobian(2,2)=jv(2,2); Jacobian(3,2)=jv(3,2); %Lower part (from jw) Jacobian(4,2)=jw(1,2); Jacobian(5,2)=jw(2,2); Jacobian(6,2)=jw(3,2);
%column3 %Upper part (from jv) Jacobian(1,3)=jv(1,3); Jacobian(2,3)=jv(2,3); Jacobian(3,3)=jv(3,3); %Lower part (from jw) Jacobian(4,3)=jw(1,3); Jacobian(5,3)=jw(2,3); Jacobian(6,3)=jw(3,3);
%column4 %Upper part (from jv) Jacobian(1,4)=jv(1,4); Jacobian(2,4)=jv(2,4); Jacobian(3,4)=jv(3,4); %Lower part (from jw) Jacobian(4,4)=jw(1,4); Jacobian(5,4)=jw(2,4); Jacobian(6,4)=jw(3,4);
%column5 %Upper part (from jv) Jacobian(1,5)=jv(1,5); Jacobian(2,5)=jv(2,5); Jacobian(3,5)=jv(3,5); %Lower part (from jw) Jacobian(4,5)=jw(1,5); Jacobian(5,5)=jw(2,5); Jacobian(6,5)=jw(3,5);
x=A12345(1,4); y=A12345(2,4); z=A12345(3,4);
Jt=transpose(Jacobian);
%Inverse kinematics %Damped Least Squares method
evector=[xt-x;yt-y;zt-z;1;0;0];
lmbd=1;
cof=(Jacobian*Jt + (lmbd^2)*eye(6,6)); invcof=inv(cof);
thetavector=Jt*invcof*evector;
th1=th1+thetavector(1,1); th2=th2+thetavector(2,1); th3=0; %Physical limitations th5=th5+thetavector(5,1);
%Control the orientation %th4=th4+thetavector(4,1); th4=-th2 -th3;
ev1=abs(evector(1)); ev2=abs(evector(2)); ev3=abs(evector(3));
thvec1(i)=th1; thvec2(i)=th2; thvec3(i)=th3; thvec4(i)=th4; thvec5(i)=th5;
i=i+1; n=n+1;
end %end use orientation end
n=0;
again=0; useagain=false;
if (abort)
 fprintf('\n')
  fprintf('No solution for this pointusing orientation control')
  fprintf('\n')
    again=input('Enter 1 to run with no orientation or 2 to exit')
    i=1;
end
if (again==1)
  useagain=true;
end
if (again==2)
 useagain=false;
end
if (useagain) 
    status=true; 
    n=0; 
    while(status)
  n=n+1;
  if(n>1000)
       fprint('More than 1000  iterations')
       status=false;
       abort=true;
 End
if(ev1<1&&ev2<1&&ev3<1)
     status=false;
End  
%Homogeneous transforms from link to link
A1=[cosd(th1) 0 -sind(th1) 0;sind(th1) 0 cosd(th1) 0;0 -1 0 L1;0 0 0 1];
A2=[cosd(th2) -sind(th2) 0 L2*cosd(th2);sind(th2) cosd(th2) 0 L2*sind(th2);0 0 1 0;0 0 0 1];
A3=[cosd(th3) -sind(th3) 0 L3*cosd(th3);sind(th3) cosd(th3) 0 L3*sind(th3);0 0 1 0;0 0 0 1];
A4=[cosd(th4) 0 sind(th4) 0;sind(th4) 0 -cosd(th4) 0;0 1 0 0 ;0 0 0 1];
A5=[cosd(th5) -sind(th5) 0 0;sind(th5) cosd(th5) 0 0;0 0 1 L4+L5;0 0 0 1];
%Auxiliar transforms to calculate the jacobian A12=A1*A2;
A123=A1*A2*A3;
A1234=A1*A2*A3*A4;
A12345=A1*A2*A3*A4*A5;
% Calculating Jw (Jacobian angular velocity ) % Jw is (3xn) where n is the number of joints
%column1 0A0 , default k vector (0,0,1) jw(1,1)=0; jw(2,1)=0; jw(3,1)=1;
%COLUMN2 0A1 get value from A1 (3rd column rows 1,2,3) jw(1,2)=A1(1,3); jw(2,2)=A1(2,3); jw(3,2)=A1(3,3);
%COLUMN3 0A2 get value from A1.A2 (3rd column rows 1,2,3) jw(1,3)=A12(1,3); jw(2,3)=A12(2,3); jw(3,3)=A12(3,3);
%COLUMN4 0A3 get value from A1.A2.A3 (3rd column rows 1,2,3)
jw(1,4)=A123(1,3); jw(2,4)=A123(2,3); jw(3,4)=A123(3,3);
%COLUMN5 0A4 get value from A1.A2.A3.A4 (3rd column rows 1,2,3) jw(1,5)=A1234(1,3); jw(2,5)=A1234(2,3); jw(3,5)=A1234(3,3);
%Calculating Jv ( Jacobian linear velocity )
%Distance from end effector frame to base frame %Get value from 0A5 (4th column rows1,2,3) oeff(1,1)=A12345(1,4); oeff(2,1)=A12345(2,4); oeff(3,1)=A12345(3,4);
%Distance from base frame to base frame (=0) o00(1,1)=0; o00(2,1)=0; o00(3,1)=0;
%Distance from joint 1 frame to base frame %Get value from 0A1 (4th column rows1,2,3) o01(1,1)=A1(1,4); o01(2,1)=A1(2,4); o01(3,1)=A1(3,4);
%Distance from joint 2 frame to base frame %Get value from A1.A2 (4th column rows1,2,3) o02(1,1)=A12(1,4); o02(2,1)=A12(2,4); o02(3,1)=A12(3,4);
%Distance from joint 3 frame to base frame %Get value from A1.A2.A3 (4th column rows1,2,3) o03(1,1)=A123(1,4); o03(2,1)=A123(2,4); o03(3,1)=A123(3,4);
%Distance from joint 4 frame to base frame %Get value from A1.A2.A3.A4 (4th column rows1,2,3) o04(1,1)=A1234(1,4); o04(2,1)=A1234(2,4); o04(3,1)=A1234(3,4);
%distance in each column of Jv ( oeff - o0(i-1)) %calculate for the 5 columns rc1=oeff-o00; rc2=oeff-o01; rc3=oeff-o02; rc4=oeff-o03; rc5=oeff-o04;
%columns from jw to do the cross with the distance jwc1(1,1)=jw(1,1); jwc1(2,1)=jw(2,1); jwc1(3,1)=jw(3,1);
jwc2(1,1)=jw(1,2); jwc2(2,1)=jw(2,2); jwc2(3,1)=jw(3,2);
jwc3(1,1)=jw(1,3); jwc3(2,1)=jw(2,3); jwc3(3,1)=jw(3,3);
jwc4(1,1)=jw(1,4); jwc4(2,1)=jw(2,4); jwc4(3,1)=jw(3,4);
jwc5(1,1)=jw(1,5); jwc5(2,1)=jw(2,5); jwc5(3,1)=jw(3,5);
%Constructing Jv
%column1 %Get the cross product jv1=cross(jwc1,rc1); %fill the jv jv(1,1)=jv1(1,1); jv(2,1)=jv1(2,1); jv(3,1)=jv1(3,1);
%column2 %Get the cross product jv2=cross(jwc2,rc2); %fill the jv jv(1,2)=jv2(1,1); jv(2,2)=jv2(2,1); jv(3,2)=jv2(3,1);
%column3 %Get the cross product jv3=cross(jwc3,rc3); %fill the jv jv(1,3)=jv3(1,1); jv(2,3)=jv3(2,1); jv(3,3)=jv3(3,1);
%column4 %Get the cross product jv4=cross(jwc4,rc4); %fill the jv jv(1,4)=jv4(1,1); jv(2,4)=jv4(2,1); jv(3,4)=jv4(3,1);
%column5 %Get the cross product jv5=cross(jwc5,rc5); %fill the jv jv(1,5)=jv5(1,1); jv(2,5)=jv5(2,1); jv(3,5)=jv5(3,1);
%Final Jacobian %The upper part is the Jacobian linear velocity (columns 1~5 , rows 1~3) %The lower part is the Jacobian angular velocity (columns 1~5, rows 4~6)
%column1 %Upper part (from jv) Jacobian(1,1)=jv(1,1); Jacobian(2,1)=jv(2,1); Jacobian(3,1)=jv(3,1); %Lower part (from jw) Jacobian(4,1)=jw(1,1); Jacobian(5,1)=jw(2,1); Jacobian(6,1)=jw(3,1);
%column2 %Upper part (from jv) Jacobian(1,2)=jv(1,2); Jacobian(2,2)=jv(2,2); Jacobian(3,2)=jv(3,2); %Lower part (from jw) Jacobian(4,2)=jw(1,2); Jacobian(5,2)=jw(2,2); Jacobian(6,2)=jw(3,2);
%column3 %Upper part (from jv) Jacobian(1,3)=jv(1,3); Jacobian(2,3)=jv(2,3); Jacobian(3,3)=jv(3,3); %Lower part (from jw) Jacobian(4,3)=jw(1,3); Jacobian(5,3)=jw(2,3); Jacobian(6,3)=jw(3,3);
%column4 %Upper part (from jv) Jacobian(1,4)=jv(1,4); Jacobian(2,4)=jv(2,4); Jacobian(3,4)=jv(3,4); %Lower part (from jw) Jacobian(4,4)=jw(1,4); Jacobian(5,4)=jw(2,4); Jacobian(6,4)=jw(3,4);
%column5 %Upper part (from jv) Jacobian(1,5)=jv(1,5); Jacobian(2,5)=jv(2,5); Jacobian(3,5)=jv(3,5); %Lower part (from jw) Jacobian(4,5)=jw(1,5); Jacobian(5,5)=jw(2,5); Jacobian(6,5)=jw(3,5);
x=A12345(1,4); y=A12345(2,4); z=A12345(3,4);
Jt=transpose(Jacobian);
%Inverse kinematics %Damped Least Squares method
evector=[xt-x;yt-y;zt-z;1;0;0];
lmbd=1;
cof=(Jacobian*Jt + (lmbd^2)*eye(6,6)); invcof=inv(cof);
thetavector=Jt*invcof*evector;
th1=th1+thetavector(1,1); th2=th2+thetavector(2,1); th3=th3+thetavector(3,1); th4=th4+thetavector(4,1); th5=th5+thetavector(5,1);
ev1=abs(evector(1)); ev2=abs(evector(2)); ev3=abs(evector(3));
thvec1(i)=th1; thvec2(i)=th2; thvec3(i)=th3; thvec4(i)=th4; thvec5(i)=th5;
i=i+1;
end
  end
if (NOuseorientation) 
    status=true; 
    n=0;
    while (status) 
        n=n+1
 if (n>1000)
      fprintf('More than 1000 iterations')
      status=false;
      abort=true;
  end
  
  if(ev1<1 && ev2<1 && ev3<1)
      status=false;
  end 
     
%Homogeneous transforms from link to link
A1=[cosd(th1) 0 -sind(th1) 0;sind(th1) 0 cosd(th1) 0;0 -1 0 L1;0 0 0 1];
A2=[cosd(th2) -sind(th2) 0 L2*cosd(th2);sind(th2) cosd(th2) 0 L2*sind(th2);0 0 1 0;0 0 0 1];
A3=[cosd(th3) -sind(th3) 0 L3*cosd(th3);sind(th3) cosd(th3) 0 L3*sind(th3);0 0 1 0;0 0 0 1];
A4=[cosd(th4) 0 sind(th4) 0;sind(th4) 0 -cosd(th4) 0;0 1 0 0 ;0 0 0 1];
A5=[cosd(th5) -sind(th5) 0 0;sind(th5) cosd(th5) 0 0;0 0 1 L4+L5;0 0 0 1];
%Auxiliar transforms to calculate the jacobian A12=A1*A2;
A123=A1*A2*A3;
A1234=A1*A2*A3*A4;
A12345=A1*A2*A3*A4*A5;
% Calculating Jw (Jacobian angular velocity ) % Jw is (3xn) where n is the number of joints
%column1 0A0 , default k vector (0,0,1) jw(1,1)=0; jw(2,1)=0; jw(3,1)=1;
%COLUMN2 0A1 get value from A1 (3rd column rows 1,2,3) jw(1,2)=A1(1,3); jw(2,2)=A1(2,3); jw(3,2)=A1(3,3);
%COLUMN3 0A2 get value from A1.A2 (3rd column rows 1,2,3) jw(1,3)=A12(1,3); jw(2,3)=A12(2,3); jw(3,3)=A12(3,3);
%COLUMN4 0A3 get value from A1.A2.A3 (3rd column rows 1,2,3)
jw(1,4)=A123(1,3); jw(2,4)=A123(2,3); jw(3,4)=A123(3,3);
%COLUMN5 0A4 get value from A1.A2.A3.A4 (3rd column rows 1,2,3) jw(1,5)=A1234(1,3); jw(2,5)=A1234(2,3); jw(3,5)=A1234(3,3);
%Calculating Jv ( Jacobian linear velocity )
%Distance from end effector frame to base frame %Get value from 0A5 (4th column rows1,2,3) oeff(1,1)=A12345(1,4); oeff(2,1)=A12345(2,4); oeff(3,1)=A12345(3,4);
%Distance from base frame to base frame (=0) o00(1,1)=0; o00(2,1)=0; o00(3,1)=0;
%Distance from joint 1 frame to base frame %Get value from 0A1 (4th column rows1,2,3) o01(1,1)=A1(1,4); o01(2,1)=A1(2,4); o01(3,1)=A1(3,4);
%Distance from joint 2 frame to base frame %Get value from A1.A2 (4th column rows1,2,3) o02(1,1)=A12(1,4); o02(2,1)=A12(2,4); o02(3,1)=A12(3,4);
%Distance from joint 3 frame to base frame %Get value from A1.A2.A3 (4th column rows1,2,3) o03(1,1)=A123(1,4); o03(2,1)=A123(2,4); o03(3,1)=A123(3,4);
%Distance from joint 4 frame to base frame %Get value from A1.A2.A3.A4 (4th column rows1,2,3) o04(1,1)=A1234(1,4); o04(2,1)=A1234(2,4); o04(3,1)=A1234(3,4);
%distance in each column of Jv ( oeff - o0(i-1)) %calculate for the 5 columns rc1=oeff-o00; rc2=oeff-o01; rc3=oeff-o02; rc4=oeff-o03; rc5=oeff-o04;
%columns from jw to do the cross with the distance jwc1(1,1)=jw(1,1); jwc1(2,1)=jw(2,1); jwc1(3,1)=jw(3,1);
jwc2(1,1)=jw(1,2); jwc2(2,1)=jw(2,2); jwc2(3,1)=jw(3,2);
jwc3(1,1)=jw(1,3); jwc3(2,1)=jw(2,3); jwc3(3,1)=jw(3,3);
jwc4(1,1)=jw(1,4); jwc4(2,1)=jw(2,4); jwc4(3,1)=jw(3,4);
jwc5(1,1)=jw(1,5); jwc5(2,1)=jw(2,5); jwc5(3,1)=jw(3,5);
%Constructing Jv
%column1 %Get the cross product jv1=cross(jwc1,rc1); %fill the jv jv(1,1)=jv1(1,1); jv(2,1)=jv1(2,1); jv(3,1)=jv1(3,1);
%column2 %Get the cross product jv2=cross(jwc2,rc2); %fill the jv jv(1,2)=jv2(1,1); jv(2,2)=jv2(2,1); jv(3,2)=jv2(3,1);
%column3 %Get the cross product jv3=cross(jwc3,rc3); %fill the jv jv(1,3)=jv3(1,1); jv(2,3)=jv3(2,1); jv(3,3)=jv3(3,1);
%column4 %Get the cross product jv4=cross(jwc4,rc4); %fill the jv jv(1,4)=jv4(1,1); jv(2,4)=jv4(2,1); jv(3,4)=jv4(3,1);
%column5 %Get the cross product jv5=cross(jwc5,rc5); %fill the jv jv(1,5)=jv5(1,1); jv(2,5)=jv5(2,1); jv(3,5)=jv5(3,1);
%Final Jacobian %The upper part is the Jacobian linear velocity (columns 1~5 , rows 1~3) %The lower part is the Jacobian angular velocity (columns 1~5, rows 4~6)
%column1 %Upper part (from jv) Jacobian(1,1)=jv(1,1); Jacobian(2,1)=jv(2,1); Jacobian(3,1)=jv(3,1); %Lower part (from jw) Jacobian(4,1)=jw(1,1); Jacobian(5,1)=jw(2,1); Jacobian(6,1)=jw(3,1);
%column2 %Upper part (from jv) Jacobian(1,2)=jv(1,2); Jacobian(2,2)=jv(2,2); Jacobian(3,2)=jv(3,2); %Lower part (from jw) Jacobian(4,2)=jw(1,2); Jacobian(5,2)=jw(2,2); Jacobian(6,2)=jw(3,2);
%column3 %Upper part (from jv) Jacobian(1,3)=jv(1,3); Jacobian(2,3)=jv(2,3); Jacobian(3,3)=jv(3,3); %Lower part (from jw) Jacobian(4,3)=jw(1,3); Jacobian(5,3)=jw(2,3); Jacobian(6,3)=jw(3,3);
%column4 %Upper part (from jv) Jacobian(1,4)=jv(1,4); Jacobian(2,4)=jv(2,4); Jacobian(3,4)=jv(3,4); %Lower part (from jw) Jacobian(4,4)=jw(1,4); Jacobian(5,4)=jw(2,4); Jacobian(6,4)=jw(3,4);
%column5 %Upper part (from jv) Jacobian(1,5)=jv(1,5); Jacobian(2,5)=jv(2,5); Jacobian(3,5)=jv(3,5); %Lower part (from jw) Jacobian(4,5)=jw(1,5); Jacobian(5,5)=jw(2,5); Jacobian(6,5)=jw(3,5);
x=A12345(1,4); y=A12345(2,4); z=A12345(3,4);
Jt=transpose(Jacobian);
%Inverse kinematics %Damped Least Squares method
%evector=[xt-x;yt-y;zt-z;tz1-te1;tz2-te2;tz3 - te3]; evector=[xt-x;yt-y;zt-z;1;0;0];
lmbd=1;
cof=(Jacobian*Jt + (lmbd^2)*eye(6,6)); invcof=inv(cof);
thetavector=Jt*invcof*evector;
th1=th1+thetavector(1,1); th2=th2+thetavector(2,1); th3=th3+thetavector(3,1); th4=th4+thetavector(4,1); th5=th5+thetavector(5,1);
ev1=abs(evector(1)); ev2=abs(evector(2)); ev3=abs(evector(3));
thvec1(i)=th1; thvec2(i)=th2; thvec3(i)=th3; thvec4(i)=th4; thvec5(i)=th5;
i=i+1;
%waitkey=input('Enter to iterate') %evector end end
q_ikine=[th1 th2 th3 th4 th5]; q_radians=(3.1416/180)*q_ikine;
clc; fprintf('The IK solver took %3g iterations to find the solution',i) 
fprintf('\n') 
fprintf('Angles calculated by the IK') 
showt=[th1 th2 th3 th4 th5] 
fprintf('\n')
%Creating Serial Link L1=11; L2=15; L3=10; L4=21; L5=8; pi=3.1416;
L(1)=Link([0 -L1 0 -pi/2 0]); L(2)=Link([0 0 L2 0 0]); L(3)=Link([0 0 L3 0 0]); L(4)=Link([0 0 0 pi/2 0]); L(5)=Link([0 -(L4+L5) 0 0 0]);
DaslArm=SerialLink(L,'name','DaslArm');
%Checking final pose xe=A12345(1,4); ye=A12345(2,4); ze=A12345(3,4);
fprintf('The end effector position is: x=%3g,y=%3g,z=%3g',xe,ye,ze) 
fprintf('\n') 
fprintf('The desired position was x=%3g,y=%3g,z=%3g',xt,yt,zt)
%Creating a trajectory from q0 to the IK solution %Time variable t=[0:0.05:4]; %trajectory q0=[0 0 0 0 0];
q_TRAJ=jtraj(q0,q_radians,t);
key=input('Enter 1 for animation and 2 for final pose'); if (key==1)
DaslArm.plot(q_TRAJ) 
end
if (key==2)
  DaslArm.teach(q_radians)


end
%Calculate the goals position to send to the Dynamixels gp1=thvec1*(1/0.088); gp2=thvec2*(1/0.088); gp3=thvec3*(1/0.088); gp4=thvec4*(1/0.29); gp5=thvec5*(1/0.29);
gp1initial=2292; gp2initial=2086; gp3initial=2007; gp4initial=538; gp5initial=509;
k=length(thvec1);
for i=1:1:k
gp1cmd(i) = gp1initial - gp1(i); %nao sei gp2cmd(i)=gp2initial - gp2(i); %ok gp3cmd(i)=gp3initial - gp3(i); %ok gp4cmd(i)=gp4initial - gp4(i); gp5cmd(i)=gp5initial - gp5(i);
end
deltagp=[gp1(k) gp2(k) gp3(k) gp4(k) gp5(k)]; finalgp=[gp1cmd(k) gp2cmd(k) gp3cmd(k) gp4cmd(k) gp5cmd(k)];
format short g fprintf('\n') fprintf('The delta goals are') deltagp fprintf('\n') fprintf('the final goals are') finalgp
