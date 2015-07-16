
#include "grid_2d.h"

Grid2D::Grid2D(const double & _sz_x, const double & _sz_y, 
               const double & _dx, const double & _dy, 
               const double & _xmax, const double & _ymax, 
               const unsigned int & _hold) : 
    size_x_(_sz_x),
    size_y_(_sz_y),
    step_x_(_dx),
    step_y_(_dy),
    xmax_(_xmax),
    ymax_(_ymax),
    holding_iters_(_hold)
{
    //compute the number of cells in each axis
    nx_ = (unsigned int)ceil(size_x_ / step_x_); 
    ny_ = (unsigned int)ceil(size_y_ / step_y_); 
    
    //update cell sizes
    size_x_ = nx_*step_x_;
    size_y_ = ny_*step_y_;
                
    //Resizes and resets the grid, with nx_ rows and ny_ columns
    grid_[0].setZero(nx_, ny_);
    grid_[1].setZero(nx_, ny_); 
    
    //init grid array indexes
    current_ = 0; 
    next_ = 1; 
}

Grid2D::~Grid2D()
{
    
}

double Grid2D::getGridStep() const
{
    return step_x_; 
}

double Grid2D::getSizeX() const
{
    return size_x_;
}

double Grid2D::getSizeY() const
{
    return size_y_;
}

unsigned int Grid2D::getNx() const
{
    return nx_;
}

unsigned int Grid2D::getNy() const
{
    return ny_;
}

double Grid2D::getXmax() const
{
    return xmax_;
}

double Grid2D::getYmax() const
{
    return ymax_;
}

bool Grid2D::xy2ij(const double & _x, const double & _y, unsigned int & _i, unsigned int & _j) const
{
    //check xy inside the grid limits. If no, return false
    if ( ( _x > xmax_ ) || (_x < xmax_-size_x_) || ( _y > ymax_ ) || (_y < ymax_-size_y_) )
    {
        return false;
    }
    else //if within limits, compute ij and return true
    {
        _i = (unsigned int)floor( (xmax_ - _x) / step_x_ ); //floor ensures xmax,ymax is the upper-left corner of the cell 0,0
        _j = (unsigned int)floor( (ymax_ - _y) / step_y_ ); //floor ensures xmax,ymax is the upper-left corner of the cell 0,0
        return true;
    }
}

void Grid2D::ij2xy(const unsigned int & _i, const unsigned int & _j, double & _x, double & _y) const
{
    _x = xmax_ - _i*step_x_ - step_x_/2.;//returns the center of the cell (-> -step_x_/2.)
    _y = ymax_ - _j*step_y_ - step_y_/2.;//returns the center of the cell (-> -step_y_/2.)
}

void Grid2D::odometryUpdate(const double & _vx, const double & _vy, const double & _w, const double & _dt)
{
    //variables
    double xx, yy, cell_value;
    unsigned gi,gj; //gridi indexes
    Eigen::Vector3d pt; 

    //reset the next grid
    grid_[next_].setZero(nx_, ny_);
    
    //mount the rotational speed vector, heading rate is around z axis
    Eigen::Vector3d ww;
    ww << 0,0,_w;
    
    //mount the translational speed vector in the xy plane
    Eigen::Vector3d vv;
    vv << _vx,_vy,0;
    
    for (unsigned int ii = 0; ii < nx_; ii++)
    {
        for (unsigned int jj = 0; jj < ny_; jj++ )
        {
            //get cell value
            cell_value = grid_[current_](ii,jj); 
            
            //get x,y point corresponding to i,j cell
            ij2xy(ii,jj,xx,yy); 
            pt << xx,yy,0;
            
            //update x,y with the odometry twist and integration time
            pt = pt - ww.cross(pt)*_dt/2.; //half twist
            pt = pt - vv*_dt; //translation part
            pt = pt - ww.cross(pt)*_dt/2.; //half twist
            
            //get i,j cell given the updated x,y point
            if ( xy2ij( pt(0), pt(1), gi, gj ) ) //if updated point within the grid limits
            {
                //copy the value to the next grid
                grid_[next_](gi,gj) = cell_value; 
            }
        }
    }
    
    //update grid array indexes
    updateGridArrayIndexes(); 
    
}

void Grid2D::lidarUpdate(const Eigen::MatrixXd & _scan_points)
{
    unsigned int ii,jj;

    //reset the next grid
    grid_[next_].setZero(nx_, ny_);
    
    for ( unsigned int kk = 0; kk < _scan_points.cols(); kk++ )
    {
        // Checks if point is inside the grid and get ij indexes from xy point
        if ( xy2ij(_scan_points(0,kk), _scan_points(1,kk), ii, jj) )
        {
            //Update current grid with this ij hit
            //grid_[current_](ii,jj) = grid_[current_](ii,jj)*0.8 + 0.2; //TODO: could be improved with something more sophisticated ...
            grid_[next_](ii,jj) = grid_[current_](ii,jj)*0.8 + 0.2;
        }
    }

    //update grid array indexes
    updateGridArrayIndexes();     
    
}

void Grid2D::radarUpdate(const Eigen::MatrixXd & _radar_clusters)
{
    
}

void Grid2D::getGrid(std::vector<int8_t> & _data) const
{
    //an index
    unsigned int kk = 0; 
    
    //resize result vector according grid size
    _data.resize(nx_*ny_);
    
    for ( int jj = ny_-1; jj >= 0; jj-- )
    {
        for ( int ii = nx_-1; ii >= 0 ; ii-- )
        {
            _data.at(kk) = (int8_t)round(grid_[current_](ii,jj)*100);
            kk ++;
        }
    }   
}

void Grid2D::display() const
{
    for ( unsigned int ii = 0; ii < nx_; ii++)
    {
        for ( unsigned int jj = 0; jj < ny_; jj++ )
        {
            if ( grid_[current_](ii,jj) > 0.8 ) std::cout << grid_[current_](ii,jj);
            else std::cout << "--";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void Grid2D::print() const
{
    std::cout << "Grid2D: " << std::endl 
            << "   size_x_: " << size_x_ << std::endl
            << "   size_y_: " << size_y_ << std::endl
            << "   step_x_: " << step_x_ << std::endl
            << "   step_y_: " << step_y_ << std::endl
            << "   nx_: " << nx_ << std::endl
            << "   ny_: " << ny_ << std::endl
            << "   xmax_: " << xmax_ << std::endl
            << "   ymax_: " << ymax_ << std::endl
            << "   holding_iters_: " << holding_iters_ << std::endl; 
}

void Grid2D::debugFill()
{
    double val = 11; 
    for (unsigned int ii = 0; ii < nx_; ii++)
    {
        for (unsigned int jj = 0; jj < ny_; jj++ )
        {
            //apply a pattern 
            if ( ( (ii+jj)%5 == 0 ) && ( jj%3 == 0 ) )
            {
                grid_[current_](ii,jj) = val; 
                val ++; 
            }
        }
    }
}

void Grid2D::updateGridArrayIndexes()
{
    next_ = current_;
    current_ = (current_ + 1) % 2; 
}
