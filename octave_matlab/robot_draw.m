%%-----------------------------------------------------------
%% (c) Juan Gonzalez-Gomez. (Obijuan)  juan@iearobotics.com
%% Distrubuted under del GPL license
%%-----------------------------------------------------------
%%-- Draw the robot from its pose (x,y,theta)
%%-- x,y: position in the xy plane
%%-- theta: orientation (in radians!)
%%-- l: Distance from the wheels to the center of mass
%%-- r: Wheel's radius
%%----------------------------------------------------------
function robot_draw(x,y,theta,r,l)

%%-- Transformation matrix for drawing the robot
c=cos(theta);
s=sin(theta);
T=[ c, -s, 0 x;
    s,  c, 0 y;
    0,  0, 1 0;
    0   0  0 1
];

%%-- Robot data. These points define the robot shape in its home state
p1 = [0  l     0  1];
p2 = [-r l     0  1];
p3 = [r  l     0  1];
p4 = [0 -l     0  1];
p5 = [-r -l    0  1];
p6 = [r  -l    0  1];
p7 = [r+20 0    0  1];

%%-- Calculate the robot pose according to the
%%-- the parameters given (x,y,theta)
P1 = T*p1';
P2 = T*p2';
P3 = T*p3';
P4 = T*p4';
P5 = T*p5';
P6 = T*p6';
P7 = T*p7';

%%-- convert the points into two list with the x and y coordinates
%%-- It is necesarry for ploting
x_chasis = [P2(1) P3(1) P1(1) P4(1) P5(1) P6(1) P4(1) P7(1) P1(1)];
y_chasis = [P2(2) P3(2) P1(2) P4(2) P5(2) P6(2) P4(2) P7(2) P1(2)];

%-- Draw the robot
plot(x_chasis,y_chasis,'-k');

%-- Set other atributes
set_axis(500);

