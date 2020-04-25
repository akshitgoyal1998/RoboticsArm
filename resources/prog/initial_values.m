%%all dimension are like length breadth height
%% base Dimesions
base.length = 5;
base.breadth= 5;
base.height = 1;

%% link1 deimensions
link1.length = 1;
link1.breadth = 1;
link1.height = 5;

%% link2 dimensions
link2.length = 1;
link2.breadth = 1;
link2.height =5;

%% wristLink dimensions
wristlink.radius = 2;
wristlink.length = 1;

%% gripper dimensions
gripperArray =[-1 0; 0 0; 1 0; 1 2; 0.5 2; 0.5 0.5; -0.5 0.5;
    -0.5 2; -1 2];
gripper.length = 0.25;

%% aggregrating
base_dimension = [base.length base.breadth base.height];
link1_dimension = [link1.length link1.breadth link1.height];
link2_dimension = [link2.length link2.breadth link2.height];



%%target point
xtarget = 20;
ytarget = 30;
ztarget =40;

