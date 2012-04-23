%---------------------------------------------------------------
%-- (c) Juan Gonzalez-Gomez (Obijuan) juan@iearobotics.com
%-- April, 2012
%-- Distributed under the GPL license
%--------------------------------------------------------------
%-- Simulation of the kinematics of limited wheel robots
%---------------------------------------------------------------

%%------------------ PARAMETERS THAT THE USER CAN CHANGE

%------ Robot parameters
r=30;    %-- Wheel radious (mm)
l=85/2;  %-- Distance between the center of mass and the wheel (mm)

%------- Oscillator parameters
O=deg2rad(0);    %--- Offset (rad)
A=deg2rad(45);   %--- Amplitude (rad)
T=1;             %--- Period (sec)
DF=deg2rad(90);  %--- Phase difference (rad)

%------ Simulation parameters
%-- If both parameters, show_robot and show_tray are false, 
%-- the simulation will be carried out at maximum speed. The resulting
%-- trayectory and robot pose will be drawn

%-- Draw the robot during simulation
show_robot = true;

%--Draw the robot's trayectory during simulation
show_tray = true;

%-- Simulation time (sec)
%%-- Setting total_time to a negative value will play the simulation
%%-- 'forever'
total_time = 10;

%-- Simulation step (in sec)
step = 0.01;

%-- Refresh time (in simulation time (sec))
%-- Adjust this parameter to simulation time to refresh the screen
%-- Refresh >= step. The bigger the faster the simulation
refresh = 0.01; 

%--------------------- END OF THE PARAMETERS THAT THE USER CAN CHANGE -------


%-- Set the Robot's initial pose
x=0;
y=0;
teta=-r/(2*l)*(A*sin(DF)+2*O) + pi/2;    %-- Robot Initial orientation

%-- Wheels initial position
teta1 = O;
teta2 = A*sin(DF) + O;

%-- Variables for storing the robot trayectory
tray_x = [x ];
tray_y = [y ];

%-- Current simulation time
stime =0;

%robot_draw(x,y,teta,r,l);
%set_axis(500);
%pause();

%-- Main Loop
%-- The simulation is run until the total time is reached
%-- If total time is a negative number, it will be run forever
while (stime<=total_time || total_time<0)

  %-- Draw the robot
  hold off;
  if (show_robot)
    robot_draw(x,y,teta,r,l);
    hold on;
  end;

  %- Draw the trayectory
  if (show_tray)
    plot(tray_x,tray_y);
    set_axis(500);
  end;

  if (show_robot || show_tray)
    pause(0.01);
  end;

  %-- update the Robot's state
  for n=0:step:refresh
    %--- Calculations in the local frame
    %-- Store the previous wheel's angles
    teta1_old = teta1;  
    teta2_old = teta2;

    %-- Calculate the new wheel's angle
    teta1 = A*sin(2*pi*stime/T) + O;
    teta2 = A*sin(2*pi*stime/T + DF) + O;
  
    %-- Calculate the actual angular velocity (Rad/s)
    w1 = (teta1 - teta1_old)/step;
    w2 = (teta2 - teta2_old)/step;

    %w1 = (2*pi*A1/T1)*cos(2*pi*t/T1 + PHI0);
    %w2 = (2*pi*A2/T2)*cos(2*pi*t/T2 + DF + PHI0);

    %-- Calculate the linear and angular velocites from wheel's angular speed
    v = r/2*(w1+w2);
    w = r/(2*l) * (w1-w2);
    vl = [v  0  w];

    %-- Calculations in the global frame
    %-- Rotation matrix (z)
    c = cos(teta);
    s = sin(teta);
    Rot = [c -s 0;
           s  c 0;
           0  0 1 ];

    %-- Calculate the velocity in the global frame reference
    vg = Rot*vl';

    %-- The new position vector is calculated from the velocity
    x=x+vg(1)*step;
    y=y+vg(2)*step;
    teta=teta+vg(3)*step;

    %-- Increase the simulation time
    stime = stime + step;
  end;

  %-- Store the current position vector
  %-- It is for drawing the trayectory
  tray_x=[tray_x x];
  tray_y=[tray_y y];

end;

%%-- Draw the trayectory and robot posses, when the user selected not
%%-- to shown them during simulation
if (not(show_robot) && not(show_tray))
  hold off;
  robot_draw(x,y,teta,r,l);
    hold on;
  plot(tray_x,tray_y);
    set_axis(500);
end;



