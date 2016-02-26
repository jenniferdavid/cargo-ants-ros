#include "scan_basics.h"

void laserscanutils::ScanParams::print() const
{
    std::cout << "Laser 2D Scan Parameters : " << std::endl 
                << "   Angle min: " << angle_min_ << std::endl
                << "   Angle max: " << angle_max_ << std::endl
                << "   Angle step: " << angle_step_ << std::endl
                << "   Scan time: " << scan_time_ << std::endl
                << "   Range min: " << range_min_ << std::endl
                << "   Range max: " << range_max_ << std::endl
                << "   Range std dev: " << range_std_dev_ << std::endl;
}

void laserscanutils::ranges2xy(const ScanParams & _params, std::vector<float> & _ranges, const ScalarT& _jump_threshold, Eigen::MatrixXs & _points, std::list<unsigned int> & _jumps)
{
    ScalarT azimuth = _params.angle_min_;
    ScalarT prev_range;
    unsigned int ii = 0;
    
    //resize to all points case
    _points.resize(3,_ranges.size());
    
    _jumps.clear();
    
    //for each range, check correctness of value and translate from polar to xy coordinates 
    while (ii <_ranges.size())
    {
        if ( _ranges[ii] >_params.range_min_ && _ranges[ii] < _params.range_max_ && !std::isnan(_ranges[ii]) && !std::isinf(_ranges[ii]) )
        {
            //transform from polar to euclidean
            _points(0,ii) = _ranges[ii] * cos(azimuth);
            _points(1,ii) = _ranges[ii] * sin(azimuth);
            _points(2,ii) = 1;
            
            //check jump
            if ( (ii != 0) && (fabs(_ranges[ii]-prev_range) > _jump_threshold) )
                _jumps.push_back(ii);

            //keep current range and update counter
            prev_range = _ranges[ii];
            ii++;
        }
        else
            _ranges.erase(_ranges.begin()+ii);

        azimuth += _params.angle_step_;
    }
    
    //resize the output matrix to the number of correct points, while keeping values
    _points.conservativeResize(3, _ranges.size());
    //_points.conservativeResize(Eigen::NoChange_t, ii_ok);  //does not compile ... why ?
}

void laserscanutils::ranges2xy(const ScanParams & _params, std::vector<float> & _ranges, const ScalarT& _jump_threshold, const ScalarT& _jump_angle_threshold, Eigen::MatrixXs & _points, std::list<unsigned int> & _jumps)
{
    ScalarT azimuth = _params.angle_min_;
    ScalarT prev_range;
    unsigned int ii = 0;

    //resize to all points case
    _points.resize(3,_ranges.size());

    _jumps.clear();

    //for each range, check correctness of value and translate from polar to xy coordinates
    while (ii <_ranges.size())
    {
        if ( _ranges[ii] >_params.range_min_ && _ranges[ii] < _params.range_max_ && !std::isnan(_ranges[ii]) && !std::isinf(_ranges[ii]) )
        {
            //transform from polar to euclidean
            _points(0,ii) = _ranges[ii] * cos(azimuth);
            _points(1,ii) = _ranges[ii] * sin(azimuth);
            _points(2,ii) = 1;

            //check jump
            if ( (ii != 0) && (fabs(_ranges[ii]-prev_range) > _jump_threshold) && (fabs(_ranges[ii]-prev_range) > tan(_jump_angle_threshold) * prev_range * _params.angle_step_) )
                _jumps.push_back(ii);

            //keep current range and update counter
            prev_range = _ranges[ii];
            ii++;
        }
        else
            _ranges.erase(_ranges.begin()+ii);

        azimuth += _params.angle_step_;
    }

    //resize the output matrix to the number of correct points, while keeping values
    _points.conservativeResize(3, _ranges.size());
    //_points.conservativeResize(Eigen::NoChange_t, ii_ok);  //does not compile ... why ?
}

void laserscanutils::ranges2xy(const ScanParams & _params, std::vector<float> & _ranges, Eigen::MatrixXs & _points)
{
    ScalarT azimuth = _params.angle_min_;
    unsigned int ii = 0;

    //resize to all points case
    _points.resize(3,_ranges.size());

    //for each range, check correctness of value and translate from polar to xy coordinates
    while (ii <_ranges.size())
    {
        if ( _ranges[ii] >_params.range_min_ && _ranges[ii] < _params.range_max_ && !std::isnan(_ranges[ii]) && !std::isinf(_ranges[ii]) )
        {
            //transform from polar to euclidean
            _points(0,ii) = _ranges[ii] * cos(azimuth);
            _points(1,ii) = _ranges[ii] * sin(azimuth);
            _points(2,ii) = 1;

            ii++;
        }
        else 
        {
            //erase the current range 
            _ranges.erase(_ranges.begin()+ii);
        }

        //increment azimuth
        azimuth += _params.angle_step_;
    }

    //resize the output matrix to the number of correct points, while keeping values
    _points.conservativeResize(3, _ranges.size());
}

laserscanutils::ScalarT laserscanutils::pi2pi(const ScalarT& angle)
{
    return (angle > 0 ? fmod(angle + M_PI, 2 * M_PI) - M_PI : fmod(angle - M_PI, 2 * M_PI) + M_PI);
}
