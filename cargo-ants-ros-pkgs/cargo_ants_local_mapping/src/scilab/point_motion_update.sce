//testing aparent motion of points when base frame moves  

// clear all 
xdel(winsid());
clear;

// --------------- USER INPUTS -------------------

//set of points in vehicle frame 0
p0 = [1  3  4  2  2 -3 -4;
      2  3  1  5 -1  1 -2;
      0  0  0  0  0  0  0];
      
//rotatinal speed
w = [0 0 1]'; 

//delta t
dt = 0.1;

//translational speed
v = [0 0 0]';

// -----------------------------------------------

// get num of points
np = size(p0,2);

//init transformed points
pt = zeros(3,np)

for ii=1:np
    //first half rotation
    q1 = p0(:,ii) - cross(w,p0(:,ii))*dt/2;
    
    //translation
    q2 = q1 - v*dt
    
    //second half rotation
    q3 = q2 - cross(w,q2)*dt/2;
    
    //set to result vector
    pt(:,ii) = q3; 
end

//plot
fig = figure(2);
fig.background = 8;
plot(p0(1,:),p0(2,:),"g.");
plot(pt(1,:),pt(2,:),"r.");

//plot settings
ah = gca();
ah.auto_scale = "on";
ah.grid = [1,1];




