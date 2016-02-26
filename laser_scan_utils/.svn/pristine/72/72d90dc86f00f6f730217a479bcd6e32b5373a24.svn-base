//std includes
#include <iostream>
#include <list>
#include <random>

//faramotics includes
#include "faramotics/dynamicSceneRender.h"
#include "faramotics/rangeScan2D.h"
#include "btr-headers/pose3d.h"

//laserscanutils
#include "entities.h"
#include "corner_detector.h"

//function to travel around each model
void motionCampus(unsigned int ii, Cpose3d & pose)
{
    if (ii<=40)
    {
        //pose.rt.setEuler( pose.rt.head()-2*M_PI/180., pose.rt.pitch(), pose.rt.roll() );
        pose.pt(0) = pose.pt(0) + 0.1;
    }
    if ( (ii>40) && (ii<=80) )
    {
        pose.pt(0) = pose.pt(0) + 0.1;
        pose.rt.setEuler(pose.rt.head(), pose.rt.pitch(), pose.rt.roll() + 0.01);
    }
    if ( (ii>80) && (ii<=120) )
    {
        pose.pt(0) = pose.pt(0) + 0.1;
        pose.rt.setEuler(pose.rt.head(), pose.rt.pitch(), pose.rt.roll() - 0.015);
    }
    if ( (ii>120) && (ii<=170) )
    {
        pose.rt.setEuler( pose.rt.head()+1.8*M_PI/180., pose.rt.pitch(), pose.rt.roll() );
        pose.moveForward(0.2);
    }
    if ( (ii>170) && (ii<=220) )
    {
        pose.rt.setEuler( pose.rt.head()-1.8*M_PI/180., pose.rt.pitch(), pose.rt.roll()-0.05*M_PI/180. );
    }
    if ( (ii>220) && (ii<=310) ) 
    {
        pose.pt(0) = pose.pt(0) + 0.1;
    }
    if ( (ii>310) && (ii<=487) ) 
    {
        pose.rt.setEuler( pose.rt.head()-1.*M_PI/180., pose.rt.pitch(), pose.rt.roll()+0.1*M_PI/180. );
        pose.moveForward(0.1);
    }
    if ( (ii>487) && (ii<=582) ) 
    {
        pose.moveForward(0.2);
    }
    if ( (ii>582) && (ii<=700) ) 
    {
        pose.pt(2) = pose.pt(2) + 0.001;
        pose.rt.setEuler( pose.rt.head()-1.*M_PI/180., pose.rt.pitch(), pose.rt.roll());
        pose.moveForward(0.1);
    }    
}

int main(int argc, char** argv)
{
	std::cout << "\n ========= Corner extraction test ===========\n";

	// INITIALIZATION ============================================================================================
    
    if (argc != 2)
    {
        std::cout << "Invalid number of arguments!" << std::endl;
        std::cout << "Call test as: test_extract_corners [full_path_file_name]" << std::endl;
        std::cout << "EXIT PROGRAM" << std::endl;
        return -1;
    }

	// Faramotics stuff
	CdynamicSceneRender *myRender;
	CrangeScan2D *myScanner;
	Cpose3d viewPoint;
	Cpose3d devicePose(2,8,0.2,0,0,0);
	vector<float> myScan;
	string modelFileName(argv[1]);
    
    //laserscanutils
    laserscanutils::ScanParams scan_params;
    scan_params.angle_min_ = -M_PI/2;
    scan_params.angle_max_ = M_PI/2;
    scan_params.angle_step_ = M_PI/720;
    scan_params.scan_time_ = 0.01;//not relevant
    scan_params.range_min_ = 0.1;
    scan_params.range_max_ = 30;
    scan_params.range_std_dev_ = 0.01;
    scan_params.print(); 

    laserscanutils::ExtractCornerParams alg_params;
    alg_params.line_params_.jump_dist_ut_ = 1.0; 
    alg_params.line_params_.jump_angle_ut_ = 3;
    alg_params.line_params_.window_length_ = 0.5;
    alg_params.line_params_.min_window_points_ = 5;
    alg_params.line_params_.k_sigmas_ut_ = 3;
    alg_params.line_params_.concatenate_ii_ut_ = 5; 
    alg_params.line_params_.concatenate_angle_ut_ = 0.1; 
    alg_params.theta_min_ = 0.4;
    alg_params.max_distance_ = 1.0;
    alg_params.print(); 
    
    //init random generators
    std::default_random_engine generator(1);
    std::normal_distribution<laserscanutils::ScalarT> laser_range_noise(0.001, scan_params.range_std_dev_); 

	//glut initialization
	faramotics::initGLUT(argc, argv);

    //create a viewer for the 3D model and scan points
	myRender = new CdynamicSceneRender(1200,700,90*M_PI/180,90*700.0*M_PI/(1200.0*180.0),0.2,100);
	myRender->loadAssimpModel(modelFileName,true); //with wireframe
    
	//create scanner and load 3D model
	myScanner = new CrangeScan2D(HOKUYO_UTM30LX_180DEG);//HOKUYO_UTM30LX_180DEG
	myScanner->loadAssimpModel(modelFileName);

	// START TRAJECTORY ============================================================================================
	for (uint step=1; step < 1000; step++)
	{
        std::cout << std::endl << "New iter: " << step << std::endl;
        
		// moves the device position
		motionCampus(step, devicePose);

		//compute scan
		myScan.clear();
		myScanner->computeScan(devicePose,myScan);

		//draws the device frame, scan hits and depth image
		myRender->drawPoseAxis(devicePose);
		myRender->drawScan(devicePose,myScan,180.*M_PI/180.,90.*M_PI/180.); //draw scan

		// extract corners
		std::list<laserscanutils::Corner> corner_list;
		laserscanutils::extractCorners(scan_params, alg_params, myScan, corner_list);
        std::cout << "corner_list.size(): " << corner_list.size() << std::endl;
        
        //draw corners
        std::vector<double> corner_vector(corner_list.size()*2);
        unsigned int ii = 0;
		for (std::list<laserscanutils::Corner>::iterator corner_it = corner_list.begin(); corner_it != corner_list.end(); corner_it++, ii=ii+2)
		{
            corner_vector[ii] = corner_it->pt_(0);
            corner_vector[ii+1] = corner_it->pt_(1);
		}
		myRender->drawCorners(devicePose,corner_vector);

		//Set view point and render the scene. Locate visualization view point, somewhere behind the device
		viewPoint.setPose(devicePose);
		viewPoint.rt.setEuler( viewPoint.rt.head(), viewPoint.rt.pitch()+20.*M_PI/180., viewPoint.rt.roll() );
		viewPoint.moveForward(-5);
		myRender->setViewPoint(viewPoint);
		myRender->render();
	}

	//delete things
	delete myRender;
    delete myScanner;
    
	//exit
	return 0;
}
