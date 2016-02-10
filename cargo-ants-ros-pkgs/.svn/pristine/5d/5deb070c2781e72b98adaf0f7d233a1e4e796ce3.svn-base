

//user entries
mm = 0.3;
bb = 1;
stdv = 0.3;

//inits
xx = [0:0.1:10];
[rows nn] = size(xx);
yy = mm*xx + bb + stdv*rand(1,nn,"normal");

//compute mean:
mx = sum(xx)/nn;
my = sum(yy)/nn;
centroid = [mx;my];

//compute cov matrix (explicitly and with scilab call)
cxx = (xx-mx)*(xx-mx)'/(nn-1);
cyy = (yy-my)*(yy-my)'/(nn-1);
cxy = (xx-mx)*(yy-my)'/(nn-1);
CC = cov(xx',yy');

//get eigen values and eigen vectors of CC
[RR,diagCC] = spec(CC);
eval1 = diagCC(1,1);
eval2 = diagCC(2,2);
evec1 = [RR(1,1);RR(2,1)];
evec2 = [RR(1,2);RR(2,2)];

//compute BB limits. Naive way
a1 = 5*sqrt(eval1);
a2 = 5*sqrt(eval2);
bb = zeros(2,4);
bb(:,1) = centroid - 0.5*a1*evec1 - 0.5*a2*evec2;
bb(:,2) = centroid + 0.5*a1*evec1 - 0.5*a2*evec2;
bb(:,3) = centroid + 0.5*a1*evec1 + 0.5*a2*evec2;
bb(:,4) = centroid - 0.5*a1*evec1 + 0.5*a2*evec2;

//compute BB limits. Correct way
//1. transform all points to new base defined by CC matrix
points_O = [xx;yy];
points_C = inv(RR)*points_O; 

//2. find bounding corner values:
max_x = max(points_C(1,:));
min_x = min(points_C(1,:));
max_y = max(points_C(2,:));
min_y = min(points_C(2,:));
corner_tl_C = [min_x;max_y]; //top left
corner_tr_C = [max_x;max_y]; //top right
corner_br_C = [max_x;min_y]; //bottom right
corner_bl_C = [min_x;min_y]; //bottom left
corners_C = [corner_tl_C corner_tr_C corner_br_C corner_bl_C];

//3. Move corners from C base to Origin base
corners_O = RR*corners_C; 


plot(xx,yy,".");
ph = gca(); //plot handle
ph.isoview = 'on';

//main directions
xpoly([mx RR(1,1)+mx],[my RR(2,1)+my]);
e=gce(); // get the current entity (the last created: here the polyline)
set(e,"foreground",5);
set(e,"thickness",3);
xpoly([mx RR(1,2)+mx],[my RR(2,2)+my]);
e=gce(); // get the current entity (the last created: here the polyline)
set(e,"foreground",5);
set(e,"thickness",3);

//naive bounidng box
xpoly(bb(1,:),bb(2,:));
e=gce(); // get the current entity (the last created: here the polyline)
set(e,"foreground",2);
set(e,"thickness",3);
e.closed = 'on' // the polyline is now closed

//good bounidng box
xpoly(corners_O(1,:),corners_O(2,:));
e=gce(); // get the current entity (the last created: here the polyline)
set(e,"foreground",3);
set(e,"thickness",3);
e.closed = 'on' // the polyline is now closed

