// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author Sergi Hernandez & Joan Perez
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef _IRI_BASE_DRIVER_NODE_H
#define _IRI_BASE_DRIVER_NODE_H

// ros base driver node, diagnostics, and iri_ros driver
#include <driver_base/driver_node.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include "iri_base_driver.h"

namespace iri_base_driver
{

/**
 * \brief IRI ROS Base Node Class
 *
 * This class inherits from the ROS class driver_base::DriverNode<TemplateDriver> 
 * to provide an execution thread to the driver object. The TemplateDriver object
 * must be an implementation of the IriBaseDriver class. The inherit template
 * design form allows complete access to the driver object while manteining 
 * flexibility to instantiate any object inherit from IriBaseDriver.
 *
 * A shared library is generated together with IriBaseDriver class to force 
 * implementation of virtual methods to final user. Functions from ROS driver 
 * base class to perform tests, add diagnostics or hooks, are mainteined to 
 * implement common purpose behaviours and forwarded to the final user node 
 * class for specification.
 *
 * Threads are implemented using iri_utils software utilities. The mainThread() 
 * function loops in a defined loop_rate_. In each iteration, the abstract 
 * mainNodeThread() function defined in the inherit node class is called.
 *
 * Similarly, functions such as addDiagnostics() or addRunningTests() are 
 * prepared for detailing common features for all driver nodes while executing  
 * specific commands for each specification by calling the respective functions 
 * addNodeDiagnostics() and addNodeRunningTests() from the inherit node
 * implementation.
 *
 * Instances of both IriBaseDriver and IriBaseNodeDriver can be easly generated 
 * with the  iri_ros_scripts package. Similarly, data can be sent and read 
 * through ROS  topics by using those scripts. The scripts can be downloaded 
 * from the iri_stack SVN.
 */
template <class Driver>
class IriBaseNodeDriver : public driver_base::DriverNode<Driver>
{
  protected:
    /**
     * \brief Thread information structure
     *
     * This structure hold system level information of the thread and it is 
     * initialized when the thread is first created. This information can not 
     * be modified at any time and it is not accessible from outside the class.
     *
     */
    pthread_t thread;

   /**
    * \brief main node post open hook
    *
    * This function sets the common parameters for all drivers which might be
    * tuned for the ROS dynamic reconfigure application.
    */
    void postOpenHook(void);

   /**
    * \brief node implementation post open hook
    * 
    * Abstrac function called by postOpenHook() that adds the specific driver
    * node parameters to the dynamic reconfigure application.
    */
    virtual void postNodeOpenHook(void) = 0;

   /**
    * \brief main node pre close hook
    *
    */
    void preCloseHook(void);

   /**
    * \brief node implementation pre close hook
    * 
    */
    virtual void preNodeCloseHook(void);

   /**
    * \brief public node handle communication object
    *
    * This node handle is going to be used to create topics and services within
    * the node namespace. Additional node handles can be instantatied if 
    * additional namespaces are needed.
    */
    ros::NodeHandle public_node_handle_;

   /**
    * \brief main thread loop rate
    * 
    * This value determines the loop frequency of the node main thread function
    * mainThread() in HZ. It is initialized at construction time. This variable 
    * may be modified in the node implementation constructor if a desired
    * frequency is required.
    */
    ros::Rate loop_rate_;

   /**
    * \brief default main thread frequency
    * 
    * This constant determines the default frequency of the mainThread() in HZ.
    * All nodes will loop at this rate if loop_rate_ variable is not modified.
    */
    static const unsigned int DEFAULT_RATE = 10; //[Hz]

  public:
   /**
    * \brief constructor
    *
    * This constructor creates and initializes the main thread of the class. It 
    * also sets the driver_base::postOpenHook thread. NodeHandle and loop_rate_
    * variables are also initialized.
    *
    * \param nh a reference to the node handle object to manage all ROS topics.
    */
    IriBaseNodeDriver(ros::NodeHandle& nh);

   /**
    * \brief destructor
    *
    * This destructor closes the driver object and kills the main thread.
    */
    ~IriBaseNodeDriver();

   /**
    * \brief main node add diagnostics
    *
    * In this function ROS diagnostics applied to all node may be added.
    *
    */
    void addDiagnostics(void);

   /**
    * \brief open status driver main node tests
    *
    * In this function tests checking driver's functionallity in driver_base
    * status=open can be added.
    */
    void addOpenedTests(void);

   /**
    * \brief stop status driver main node tests
    *
    * In this function tests checking driver's functionallity in driver_base
    * status=stop can be added.
    */
    void addStoppedTests(void);

   /**
    * \brief run status driver main node tests
    *
    * In this function tests checking driver's functionallity in driver_base
    * status=run can be added.
    */
    void addRunningTests(void);

   /**
    * \brief main node dynamic reconfigure
    *
    * 
    * \param level integer
    */
    void reconfigureHook(int level);

  protected:
   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the 
    * specific node may be added.
    */
    virtual void addNodeDiagnostics(void) = 0;

   /**
    * \brief open status driver specific node tests
    *
    * In this abstract function tests checking driver's functionallity in 
    * driver_base status=open can be added.
    */
    virtual void addNodeOpenedTests(void) = 0;

   /**
    * \brief stop status driver specific node tests
    *
    * In this abstract function tests checking driver's functionallity in 
    * driver_base status=stop can be added.
    */
    virtual void addNodeStoppedTests(void) = 0;

   /**
    * \brief run status driver specific node tests
    *
    * In this abstract function tests checking driver's functionallity in
    * driver_base status=run can be added.
    */
    virtual void addNodeRunningTests(void) = 0;

   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every inherit driver node object. The loop won't exit until driver node
    * finish its execution. The commands implemented in the abstract function
    * mainNodeThread() will be executed in every iteration.
    * 
    * Loop frequency can be tuned my modifying loop_rate_ attribute.
    * 
    * \param param is a pointer to a IriBaseDriver object class. It is used to 
    *              access to the object attributes and methods.
    */
    static void *mainThread(void *param);

   /**
    * \brief specific node thread
    *
    * In this abstract function specific commands for the driver base node
    * have to be detailed.
    */
    virtual void mainNodeThread(void) = 0;

   /**
    * \brief specific node dynamic reconfigure
    *
    * This function is called reconfigureHook()
    * 
    * \param level integer
    */
    virtual void reconfigureNodeHook(int level) = 0;
};

template <class Driver>
IriBaseNodeDriver<Driver>::IriBaseNodeDriver(ros::NodeHandle &nh) : 
  driver_base::DriverNode<Driver>(nh), 
  public_node_handle_(ros::this_node::getName()),
  loop_rate_(DEFAULT_RATE)
{
  ROS_DEBUG("IriBaseNodeDriver::Constructor");
  this->driver_.setPostOpenHook(boost::bind(&IriBaseNodeDriver::postOpenHook, this));
  this->driver_.setPreCloseHook(boost::bind(&IriBaseNodeDriver::preNodeCloseHook, this));

  // create the status thread
  pthread_create(&this->thread,NULL,this->mainThread,this);
  
  // initialize class attributes
}

template <class Driver>
void *IriBaseNodeDriver<Driver>::mainThread(void *param)
{
  ROS_DEBUG("IriBaseNodeDriver::mainThread");

  IriBaseNodeDriver<Driver> *iriNode = (IriBaseNodeDriver<Driver> *)param;

  while(ros::ok())
  {
//     std::cout << __LINE__ << ": driver state=" << iriNode->driver_.getStateName() << std::endl;
    if(iriNode->driver_.isRunning())
    {
      iriNode->mainNodeThread();

//       ros::spinOnce();
    }
    iriNode->loop_rate_.sleep();

  }

  pthread_exit(NULL);
}

template <class Driver>
void IriBaseNodeDriver<Driver>::postOpenHook(void)
{
  ROS_DEBUG("IriBaseNodeDriver::postOpenHook");

  //set hardware id with driver id
  this->diagnostic_.setHardwareID(this->driver_.getID());

  postNodeOpenHook();
}

template <class Driver>
void IriBaseNodeDriver<Driver>::preCloseHook(void)
{
  ROS_INFO("IriBaseNodeDriver::preCloseHook");
  preNodeCloseHook();
}

template <class Driver>
void IriBaseNodeDriver<Driver>::preNodeCloseHook(void)
{
}
    
template <class Driver>
void IriBaseNodeDriver<Driver>::addDiagnostics(void)
{
  ROS_DEBUG("IriBaseNodeDriver::addDiagnostics");
  addNodeDiagnostics();
}

template <class Driver>
void IriBaseNodeDriver<Driver>::addOpenedTests(void)
{
  ROS_DEBUG("IriBaseNodeDriver::addOpenedTests");
  addNodeOpenedTests();
}

template <class Driver>
void IriBaseNodeDriver<Driver>::addStoppedTests(void)
{
  ROS_DEBUG("IriBaseNodeDriver::addStoppedTests");
  addNodeStoppedTests();
}

template <class Driver>
void IriBaseNodeDriver<Driver>::addRunningTests(void)
{
  ROS_DEBUG("IriBaseNodeDriver::addRunningTests");
  addNodeRunningTests();
}

template <class Driver>
void IriBaseNodeDriver<Driver>::reconfigureHook(int level)
{
  ROS_DEBUG("IriBaseNodeDriver::reconfigureHook");
  reconfigureNodeHook(level);
}

template <class Driver>
IriBaseNodeDriver<Driver>::~IriBaseNodeDriver()
{
  ROS_DEBUG("IriBaseNodeDriver::Destrcutor");
  //try access to driver to assert it is
  //unlock before its destruction
  this->driver_.try_enter();
  this->driver_.unlock();
  
  this->driver_.close();
  pthread_cancel(this->thread);
  pthread_join(this->thread,NULL);
}

}

#endif
