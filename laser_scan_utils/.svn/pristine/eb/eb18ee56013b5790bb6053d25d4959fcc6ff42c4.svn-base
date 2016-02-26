//info about 2D homogeneous lines and points: http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/BEARDSLEY/node2.html

// clear all 
xdel(winsid());
clear;

//scan params:
Ns = 720; //scan rays
aperture = %pi; //scan aperture [rad]
azimuth_step = aperture/Ns;

//User Tunning params
Nw = 8; //window size
theta_th = %pi/8;
theta_max = 0.3;
K = 3; //How many std_dev are tolerated to count that a point is supporting a line
r_stdev = 0.01; //ranging std dev
max_beam_dist = 5; //max distance in amount of beams to consider concatenation of two lines
max_dist = 0.2; //max distance to a corner from the ends of both lines to take it
range_jump_th = 0.5; //threshold of distance to detect jumps in ranges
max_points_line = 1000; //max amount of beams of a line

//init
points = [];
result_lines = [];
line_indexes = [];
corners = [];
corners_jumps = [];
jumps = [];
apertures=[];//corner apertures

//scan ranges
ranges = read('/home/acoromin/dev/labrobotica/algorithms/laser_scan_utils/trunk/src/scilab/scan.txt',-1,Ns);
//ranges = read(fullpath('scan.txt'),-1,Ns);
//ranges = read(fullpath(TMPDIR + '/scan.txt'),-1,Ns);
//ranges = [];

//invent a set of points + noise
//points = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24  25 26  27 28  29 30  31 32  33  34  35  36  37 38  39  40  41  42  43;
//          7 6 5 4 3 2 1 2 3 4  5  6  7  8  9  10 9  8  7  6  5  4  3  3.5 4  4.5 5  5.5 6  6.5 7  7.5 7.4 7.3 7.2 7.1 7  6.9 6.8 6.7 6.6 6.5 6.4];
for i=1:Ns
   points = [points [ranges(i)*cos(aperture/2 - azimuth_step*i); ranges(i)*sin(aperture/2 - azimuth_step*i)]];
   // Store range jumps
   if i>1 & abs(ranges(i)-ranges(i-1)) > range_jump_th then
       jumps = [jumps i];
   end
end

points = points + rand(points,"normal")*r_stdev;

//Runs over a sliding window of Nw points looking for straight lines
i_jump = 1;
for i = Nw:size(points,2)
    
    //set the window indexes
    i_from = i-Nw+1;
    i_to = i;
    points_w = points(:,i_from:i_to);
    
    //update the jump to be checked
    while i_jump < size(jumps,2) & i_from > jumps(i_jump)
        i_jump = i_jump+1;
    end
    
    //check if there is a jump inside the window (if it is the case, do not fit a line)
    if jumps(i_jump) > i_from & jumps(i_jump) <= i_to then
        continue;
    end

    //Found the best fitting line over the window. Build the system: Ax=0. Matrix A = a_ij
//    a_00 = sum( points_w(1,:).^2 );
//    a_01 = sum( points_w(1,:).*points_w(2,:) );
//    a_02 = sum( points_w(1,:) );
//    a_10 = a_01;
//    a_11 = sum( points_w(2,:).^2 );
//    a_12 = sum( points_w(2,:) );
//    a_20 = a_02;
//    a_21 = a_12;
//    a_22 = Nw;
//    A1 = [a_00 a_01 a_02; a_10 a_11 a_12; a_20 a_21 a_22; 0 0 1];
 
    a_00 = sum( points_w(1,:).^2 );
    a_01 = sum( points_w(1,:).*points_w(2,:) );
    a_02 = sum( points_w(1,:) );
    a_10 = a_01;
    a_11 = sum( points_w(2,:).^2 );
    a_12 = sum( points_w(2,:) );
    A = [a_00 a_01 a_02; a_10 a_11 a_12; 0 0 1];
    
    //solve
//    line1 = pinv(A1)*[zeros(3,1);1];
    line = inv(A)*[0; 0; 1];
    //disp(line1-line);

    //compute error
    err = 0;
    for j=1:Nw
        err = err + abs(line'*[points_w(:,j);1])/sqrt(line(1)^2+line(2)^2);
    end
    err = err/Nw;
    //disp("error: "); disp(err);
    
    //if error below stdev, add line to result set
    if err < K*r_stdev then
        result_lines = [result_lines [line;points_w(:,1);points_w(:,$)]];
        line_indexes = [line_indexes [i_from; i_to]]; //ray index where the segment ends
    end    
end

//line concatenation
j = 1;
while (j < size(result_lines,2))
    
    // num of rays between last of current line and first of next line
    if (line_indexes(1,j+1)-line_indexes(2,j)) > max_beam_dist then
        j=j+1;
    else    
        //compute angle diff between consecutive segments
        cos_theta = result_lines(1:2,j)'*result_lines(1:2,j+1) / ( norm(result_lines(1:2,j)) * norm(result_lines(1:2,j+1)) );
        theta = abs(acos(cos_theta));
        
        //if angle diff lower than threshold, concatenate
        if theta < theta_max then
            
            //set the new window
            i_from = line_indexes(1,j);//first point index of the first line
            i_to = line_indexes(2,j+1);//last point index of the second line
            points_w = points(:,i_from:i_to); //that's the new window !
        
            //Found the best fitting line over the window. Build the system: Ax=0. Matrix A = a_ij           
            a_00 = sum( points_w(1,:).^2 );
            a_01 = sum( points_w(1,:).*points_w(2,:) );
            a_02 = sum( points_w(1,:) );
            a_10 = a_01;
            a_11 = sum( points_w(2,:).^2 );
            a_12 = sum( points_w(2,:) );
            A = [a_00 a_01 a_02; a_10 a_11 a_12; 0 0 1];
            
            //solve
            line = inv(A)*[0; 0; 1];
        
            //compute error
            err = 0;
            for k=1:Nw
                err = err + abs(line'*[points_w(:,k);1])/sqrt(line(1)^2+line(2)^2);
            end
            err = err/Nw;
            
            //if error below stdev, change line to concatenation and erase the next line
            if err < K*r_stdev then
                result_lines(:,j) = [line;points_w(:,1);points_w(:,$)];
                line_indexes(:,j) = [i_from; i_to];
                result_lines = [result_lines(:,1:j) result_lines(:,j+2:$)];
                line_indexes = [line_indexes(:,1:j) line_indexes(:,j+2:$)];
                if (i_to-i_from)>max_points_line then
                    j=j+1;
                end
            else
                j=j+1;
            end    
        else
            j=j+1;
        end
    end
end

//corner detection
for i = 1:(size(result_lines,2)-1)
    for j = i+1:size(result_lines,2)
        
        // num of rays between last of current line and first of next line
        if (line_indexes(1,j)-line_indexes(2,i)) > max_beam_dist then
            break;
        end
        //compute angle diff between consecutive segments
        cos_theta = result_lines(1:2,i)'*result_lines(1:2,j) / ( norm(result_lines(1:2,i)) * norm(result_lines(1:2,j)) );
        theta = abs(acos(cos_theta));
        
        //if angle diff greater than threshold && indexes are less than Nw, we decide corner
        if theta > theta_th then
            
            //Corner found! Compute "sub-pixel" corner location as the intersection of two lines
            corner = cross(result_lines(1:3,i),result_lines(1:3,j));
            corner = corner./corner(3);//norlamlize homogeneous point
            
            // Check if the corner is close to both lines ends
            if ( norm(corner(1:2)-points(:,line_indexes(2,i))) < max_dist & norm(corner(1:2)-points(:,line_indexes(1,j))) < max_dist ) then

                //add corner to the resulting list
                corners = [corners corner];

                //compute aperture
                p1 = [points(:,line_indexes(1,i));1]//first point of the i line (lower azimuth)
                p2 = [points(:,line_indexes(2,j));1]//last point of the j line (greater azimuth)
                det([p1';corner;p2])
                aperture = 
                apertures = [apertures aperture];
            end
            //display
            //disp("theta: "); disp(theta);
            //disp("index:" ); disp(line_indexes(i)-Nw+1);//line_indexes(i) indicates the end point of the segment
        end
        
    end
end

// corner detection from jumps
for i=1:size(jumps,2)
    if ranges(jumps(i)) < ranges(jumps(i)-1) then
        corners_jumps = [corners_jumps points(:,jumps(i))];
    else
        corners_jumps = [corners_jumps points(:,jumps(i)-1)];
    end
end
    
//Set figure
fig1 = figure(0);
fig1.background = 8;

//plot points
plot(points(1,:),points(2,:),"g.");

//axis settings
ah = gca();
ah.isoview = "on";
ah.x_label.text = "$x [m]$";
ah.x_label.font_size = 4;
ah.y_label.text = "$y [m]$";
ah.grid = [0.1,0.1,0.1];
ah.grid_position = "background";


//plot lines
for i=1:size(result_lines,2)
    m = -result_lines(1,i)/result_lines(2,i);
    xc = -result_lines(3,i)/result_lines(2,i);
    point1 = [result_lines(4,i) m*result_lines(4,i)+xc];
    point2 = [result_lines(6,i) m*result_lines(6,i)+xc];
    xpoly([point1(1) point2(1)],[point1(2) point2(2)]);
end

//plot corners
plot(corners(1,:),corners(2,:),"ro");
plot(corners_jumps(1,:),corners_jumps(2,:),"bo");

disp(corners_jumps');
disp(corners');

//plot jumps
//for i=1:size(jumps,2)
//    plot(ranges(jumps(i))*cos(aperture/2 - azimuth_step*jumps(i)), ranges(jumps(i))*sin(aperture/2 - azimuth_step*jumps(i)),"bx");
//end


