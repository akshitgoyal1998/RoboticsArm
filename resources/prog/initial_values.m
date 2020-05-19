% %%all dimension are like length breadth height
% %% base Dimesions
base.length = 50;
base.breadth= 50;
base.height = 10;

%% link1 deimensions
link1.length = 10;
link1.breadth = 10;
link1.height = 50;

%% link2 dimensions
link2.length = 10;
link2.breadth = 10;
link2.height =50;

%% wristLink dimensions
wristlink.radius = 20;
wristlink.length = 10;

%% gripper dimensions
gripperArray =[-10 0; 0 0; 10 0; 10 20; 5 20; 5 5; -5 5;
    5 20; -10 20];
gripper.length = 2.5;

%% aggregrating
base_dimension = [base.length base.breadth base.height];
link1_dimension = [link1.length link1.breadth link1.height];
link2_dimension = [link2.length link2.breadth link2.height];



%%target point
xtarget = 25;
ytarget = 77;
ztarget =200;
robot =importrobot('oldrobot.slx');
configuration = homeConfiguration(robot);



