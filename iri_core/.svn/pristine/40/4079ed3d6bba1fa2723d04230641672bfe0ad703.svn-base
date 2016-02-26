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

#ifndef _IRI_BASE_DRIVER_H
#define _IRI_BASE_DRIVER_H

// ros base driver class
#include "driver_base/driver.h"
#include "ros/ros.h"

namespace iri_base_driver
{

/**
 * \brief IRI ROS Base Driver Class
 *
 * This class inherits from the ROS class driver_base::Driver, which provides 
 * the guidelines for any driver kind. The IriBaseDriver class offers an easy 
 * framework to integrate functional drivers implemented in C++ with the ROS 
 * driver structure. A shared library is generated together with IriBaseNodeDriver 
 * class to force implementation of virtual methods to final user. Common use 
 * functions to open, close, start  and stop a driver need to be filled up when 
 * instantiating this class. ROS  provides state transition methods to change 
 * from one state to another.
 *
 * This class is used as an intermediate level between ROS basic classes and the
 * user final driver implementation. Methods to manage drivers are implemented 
 * to provide common features to all driver-type objects. New methods are 
 * provided to the user for the specific driver needs.
 *
 * Instances of both IriBaseDriver and IriBaseNodeDriver can be easly generated 
 * with the  iri_ros_scripts package. Similarly, data can be sent and read 
 * through ROS topics by using those scripts. The scripts can be downloaded 
 * from the iri_stack SVN.
 */
class IriBaseDriver : public driver_base::Driver
{
  protected:
   /**
    * \brief driver unique identifier
    *
    * This value is used to identify each driver. Must be set before changing
    * the driver_base::state=run. Use the abstract function setDriverId().
    */
    std::string driver_id_;

    /**
     * \brief  variable to handle the mutual exclusion mechanism
     *
     * this variable is automatically initialized when an object of this class
     * or any inherited class is created and holds all the necessary information 
     * to handle any shared resource.
     *
     */
    pthread_mutex_t access_;

   /**
    * \brief Set Driver Id
    *
    * A private method to set the unique driver identifier. Must be set before
    * changing the driver_base::state=run in either the specific node constructor
    * or in the doOpen() function.
    *
    * \param id string driver id
    */
    void setDriverId(const std::string & id);

//     typedef boost::function< void() > hookFunction;
    hookFunction preCloseHook;

  public:
   /**
    * \brief Constructor
    *
    * The constructor calls the base class constructor from driver_base::Driver. 
    * Main driver's parameters can be initialized here or in the doOpen() method.
    */
    IriBaseDriver();

   /**
    * \brief Lock Driver
    *
    * Locks access to the driver class
    */
    void lock(void);

   /**
    * \brief Unlock Driver
    *
    * Unlocks access to the driver class
    */
    void unlock(void);

   /**
    * \brief Tries Access to Driver
    *
    * Tries access to driver
    * 
    * \return true if the lock was adquired, false otherwise
    */
    bool try_enter(void);

   /**
    * \brief Generic Open Driver
    *
    * Currently only changes the ROS driver state to open and calls the abstract 
    * function openDriver(). Common actions required for opening any kind of 
    * drivers should be added here.
    */
    void doOpen(void);

   /**
    * \brief Generic Close Driver
    *
    * Currently only changes the ROS driver state to close and calls the abstract
    * function closeDriver(). Common actions required for opening any kind of 
    * drivers should be added here.
    */
    void doClose(void);

   /**
    * \brief Generic Start Driver
    *
    * Currently only changes the ROS driver state to start and calls the abstract
    * function startDriver().Common actions required for opening any kind of 
    * drivers should be added here.
    */
    void doStart(void);

   /**
    * \brief Generic Stop Driver
    *
    * Currently only changes the ROS driver state to stop and calls the abstract 
    * function stopDriver(). Common actions required for opening any kind of 
    * drivers should be added here.
    */
    void doStop(void);

   /**
    * \brief Get Driver Id
    *
    * A method to get the unique driver identifier, such as serial number or 
    * driver name + id. If no id is provided in the inherit class, then string 
    * "none" is assigned.
    *
    * \return string driver id
    */
    std::string getID(void);

   /**
    * \brief User Open Driver
    *
    * An abstract method to be implemented in the inherit classes. Is called by
    * doOpen() method. Parameters necessary to be initialized when openning the
    * driver need to be filled up here.
    *
    * \return bool successful
    */
    virtual bool openDriver(void) = 0;

   /**
    * \brief User Close Driver
    *
    * An abstract method to be implemented in the inherit classes. Is called by
    * doClose() method. Updates necessary driver variables according to the state.
    *
    * \return bool successful
    */
    virtual bool closeDriver(void) = 0;

   /**
    * \brief User Start Driver
    *
    * An abstract method to be implemented in the inherit classes. Is called by
    * doStart() method. Parameters and comprovations before running the driver 
    * may be initialized and checked.
    *
    * \return bool successful
    */
    virtual bool startDriver(void) = 0;

   /**
    * \brief User Stop Driver
    *
    * An abstract method to be implemented in the inherit classes. Is called by 
    * doStop() method. Updates necessary driver variables according to the state.
    *
    * \return bool successful
    */
    virtual bool stopDriver(void) = 0;
    
    
    void setPreCloseHook(hookFunction f);

   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~IriBaseDriver();
};

}

#endif
