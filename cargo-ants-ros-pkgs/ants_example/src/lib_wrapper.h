#ifndef lib_wrapper_H
#define lib_wrapper_H

//add here your library includes
// #include "..."

// std includes
#include <iostream>

//ROS includes
#include "ros/ros.h" //required
#include <std_msgs/String.h> //out message
//#include <std_msgs/Float64.h>

//ROS dynamic configure. 
// #include <ants_example/ants_exampleConfig.h>

/** \brief Wrapper class to Wolf library objects
 *
 * Wrapper class to Wolf library objects
 * 
 */
class LibWrapper
{
    protected: 
        //ros node handle
        ros::NodeHandle nh_;
        
        // subscriber
        //ros::Subscriber subscriber_;

        // publisher
        ros::Publisher publisher_;
        
        //output message
        std_msgs::String out_msg_;    
        
        //data 
        unsigned int count_;
        
    public:    
        //wished process rate, [hz]
        double loop_rate_;
        
    protected: 
        // subscriber callback
        //void subscriber_callback(const std_msgs::Float64::ConstPtr& msg);        
                   
    public:
        /** \brief Default constructor
        * 
        * This constructor initializes the node. 
        * 
        */
        LibWrapper();

        /** \brief Destructor
        * 
        * This destructor frees all necessary dynamic memory allocated within this class.
        */
        ~LibWrapper();
        
        /** \brief Returns true if new data has been received
         * 
         * Returns true if new data has been received
         * 
         **/
        bool newData();

        /** \brief Main process 
        * 
        * Main process flow
        * 
        **/
        void process();
                    
        /** \brief Publish output messages
        * 
        * Publish output messages
        */
        void publish();                              
        
        /** \brief Sleep to adjust to desired rate
        * 
        * Sleep to adjust to desired rate
        */
        void sleep();
                
};
#endif
