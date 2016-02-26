
//Computes corner aperture in 2D, given two extreme points and the corner location
// REQUIRES: three points in homogeneous coordinates (last component set to 1): 
//    p1: first point of the first line supporting a corner
//    c: corner point
//    p2: last point of the second line supporting a corner
// ENSURES:
//    A return angle in [0,2pi]
// DEGENERATE CASE: if p1=c, and/or p2=c, the result is 0; 

function [ret_alpha] = corner_aperture(p1,c,p2);

    //check degenerate case (c=p1, or c=p2)
    deg_case = %F;//set degenerate case flag to false
    n_comp_equal = size(find(p1 == c),2); //number of components equal
    if n_comp_equal == 3
        deg_case = %T;
    end
    n_comp_equal = size(find(p2 == c),2); //number of components equal
    if n_comp_equal == 3
        deg_case = %T;
    end

    if deg_case == %F then
        //buid lines from c to p1 and p2
        l1 = cross(c,p1);
        l2 = cross(c,p2);
        
        //normalize lines
        l1 = l1 / ( sqrt( l1(1)*l1(1)+l1(2)*l1(2) ) );
        l2 = l2 / ( sqrt( l2(1)*l2(1)+l2(2)*l2(2) ) );
        
        //compute angle
        alpha = abs(acos( l1(1)*l2(1) + l1(2)*l2(2) ) ) ; // abs to avoid complex numbers due to numerical precision
        
        //compute det
        detM = det([p1';c';p2']);
        
        //if det<0, return the complementary angle
        if(detM<0) then
            ret_alpha = 2*%pi-alpha;
        else
            ret_alpha = alpha;    
        end
    else
        ret_alpha = 0;
    end
    
endfunction
