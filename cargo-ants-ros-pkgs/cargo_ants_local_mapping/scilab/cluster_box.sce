

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
a1 = 5*sqrt(eval1);
a2 = 5*sqrt(eval2);
evec1 = [RR(1,1);RR(2,1)];
evec2 = [RR(1,2);RR(2,2)];

//compute BB limits
bb = zeros(2,4);
bb(:,1) = centroid - 0.5*a1*evec1 - 0.5*a2*evec2;
bb(:,2) = centroid + 0.5*a1*evec1 - 0.5*a2*evec2;
bb(:,3) = centroid + 0.5*a1*evec1 + 0.5*a2*evec2;
bb(:,4) = centroid - 0.5*a1*evec1 + 0.5*a2*evec2;

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

//bounidng box
xpoly(bb(1,:),bb(2,:));
e=gce(); // get the current entity (the last created: here the polyline)
set(e,"foreground",3);
set(e,"thickness",3);
e.closed = 'on' // the polyline is now closed