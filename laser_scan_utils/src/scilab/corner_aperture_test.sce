
//includes
exec('/home/vvaquero/iri-lab/labrobotica/algorithms/laser_scan_utils/trunk/src/scilab/corner_aperture.sci');

//points
p1 = [2;5;1];
p2 = [5;2;1];

//plot steps
dx = 0.1;
dy = 0.1
xmin = 1;
xmax = 10;
ymin = 1;
ymax = 10;

//move a corner between these two points to build the 3D plot 
ii = 0; 
ap_mat = [];
for cx = xmin:dx:xmax
    ii = ii + 1; 
    jj = 0; 
    ap_mat_col = [];
    for cy = ymin:dy:ymax
        jj = jj + 1;
        aperture = corner_aperture(p1,[cx;cy;1],p2);
        ap_mat_col = [ap_mat_col; aperture]; 
    end
    ap_mat = [ap_mat ap_mat_col];
end

//plot results
fh = scf();
xset("colormap",jetcolormap(128));
colorbar(min(ap_mat),max(ap_mat));
surf(ap_mat,'thickness',0);
//fh.color_map = jetcolormap(128);
fh.children.rotation_angles = [0,-90];
title('Corner aperture between two points');
