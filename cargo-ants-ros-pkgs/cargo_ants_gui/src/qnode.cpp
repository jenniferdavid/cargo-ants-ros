/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/cargo_ants_gui/qnode.hpp"
#include "cargo_ants_msgs/ContainerToSchedule.h"
//#include "cargo_ants_msgs/MockupMap.h"
#include <algorithm>
#include <QDebug>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cargo_ants_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"cargo_ants_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"cargo_ants_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	start();
	return true;
}

void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to read just the scrollbar
}

  void QNode::run (QList<QListWidgetItem *> const &items)
  {
     //  ROS_INFO ("oiuytrew");
     QString s[items.size()];
     cargo_ants_msgs::ContainerToSchedule containers;//first layer

     ros::NodeHandle n;
     container_to_schedule = n.advertise<cargo_ants_msgs::ContainerToSchedule>("container_to_schedule_topic",10);
     ros::Rate loop_rate(1);
          
     while ( ros::ok() ) {
       for (int i =0; i<items.size(); i++)
	 {
	   s[i] = items[i]->text(); 
	   cargo_ants_msgs::ContainerToSchedule container; //inner layer
	   container.name = s[i].toUtf8().constData();
	   container.container.push_back(container.name);
	 }
       
       container_to_schedule.publish(containers);
       //   ROS_INFO ("oiuytrew");
       // ROS_INFO (" LIST OF containers",containers);

       ros::spinOnce();
       loop_rate.sleep();
      }
      std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
      Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)*/
   }

  /* QNode::list ()
  {
    typedef map <string, ContainerInfo> container_info_t;//renaming map type as a new type container_info_t for future needs
    static container_info_t container_info;//this is the map with container info
    typedef map <string, VehicleInfo> vehicle_info_t; // same for vehicle map type
    static vehicle_info_t vehicle_info; // ... and the map itself
    static dispatched_t dispatched;

    static void container_info_cb (ContainerInfo::ConstPtr const & msg)
    {
      container_info[msg->container] = *msg; // adds msg into container map, using 'container' field as the key.
    }

    static void vehicle_info_cb (VehicleInfo::ConstPtr const & msg)
    {
      vehicle_info[msg->vehicle] = *msg; // the same as above, using 'vehicle'
      if (VehicleInfo::PLACING == msg->mode) { // if the vehicle is in a placing mode
	dispatched.erase (msg->container); // remove the vehicle from the "dispatched" set
      }
    }

    ros::Subscriber container_info_sub (node.subscribe ("/container_info", 100, container_info_cb));
    ros::Subscriber vehicle_info_sub (node.subscribe ("/vehicle_info", 100, vehicle_info_cb));
    set <string> available_vehicles;

    // checks the list of vehicles and filters only the names of available vehicles
    for (vehicle_info_t::const_iterator iv (vehicle_info.begin()); vehicle_info.end() != iv; ++iv) {
      if (VehicleInfo::IDLE == iv->second.mode) {
	available_vehicles.insert (iv->first);
      }
    }
    set <string> waiting_containers;

    // checks the list of containers and filters only the names that are lifted and ... not dispatched?
    for (container_info_t::const_iterator ic (container_info.begin()); container_info.end() != ic; ++ic) {
      if ((ContainerInfo::LIFTED == ic->second.state)  && (0 == dispatched.count (ic->first))) {
	waiting_containers.insert (ic->first);
      }
    }
    }*/
}
  // namespace cargo_ants_gui
