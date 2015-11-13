/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://asl.epfl.ch/
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 */


#ifndef SUNFLOWER_ROBOTMODEL_HPP
#define SUNFLOWER_ROBOTMODEL_HPP


#include <sfl/util/numeric.hpp>
#include <sfl/util/Hull.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>


namespace sfl {


  /**
     Encapsulates geometric, kinematic, and dynamic models of the
     robot. The geometry is represented as a hull (set of polygons),
     kinematics are hard coded for differential drive robots, and
     dynamics are simple bounds on speeds and accelerations.
  */
  class RobotModel
  {
  public:
    /**
       Abstract class for providing actual numerical values to
       RobotModel.
     
       If you play with model parameters, good ballpark figures for
       the max accelerations are:

       - qddMax approx. 1.0 * qdMax: the robot will take one second to
         go from zero to full speed

       - sddMax approx. 0.75 * wheelRadius * qddMax: the DWA will cut
         a small corner off the actuator velocity space, which is
         important to allow maneuvers at high speeds (otherwise the
         speed objective function can trump the alignment objective,
         and the robot will swerve towards the goal only at the last
         moment).

       - thetaddMax approx. 1.5 * wheelRadius * qddMax / wheelBase:
         similarly to sddMax, this cuts of a corner for pure rotations
         (this is much less critical, because pure rotations tend to
         not compete with the speed objective)
    */
    class Parameters{
    public:
      /** safetyDistance [m]   (the robot outline is virtually expanded by this) */
      double safetyDistance;
      
      /** wheelBase [m]        (distance between wheel center points) */
      double wheelBase;
      
      /** wheelRadius [m]      (distance from floor to wheel center points) */
      double wheelRadius;
      
      /** qdMax [rad/s]        (max wheel rotation speed) */
      double qdMax;
      
      /** qddMax [rad/s/s]     (max wheel rotation acceleration [rad/s/s]) */
      double qddMax;
      
      /** sdMax [m/s]          (max robot translational speed) */
      double sdMax;
      
      /** thetadMax [rad/s]    (max robot rotational speed) */
      double thetadMax;
      
      /** sddMax [m/s/s]       (max robot translational acceleration) */
      double sddMax;
      
      /** thetaddMax [rad/s/s] (max robot rotational acceleration) */
      double thetaddMax;
      
      Parameters(double safetyDistance,
		 double wheelBase,
		 double wheelRadius,
		 double qdMax,
		 double qddMax,
		 double sdMax,
		 double thetadMax,
		 /** suggest 0.75*wheelRadius*qddMax */
		 double sddMax,
		 /** suggest 1.5*wheelRadius*qddMax/wheelBase */
		 double thetaddMax);
    };
    
    
    /** \todo Good to pass parameters as instance? */
    RobotModel(Parameters parameters,
	       boost::shared_ptr<const Hull> hull);
    
    /**
       \return The robot's hull (without safety buffer zone).
    */
    boost::shared_ptr<const Hull> GetHull() const;
    
    /**
       \return The robot's hull with safety buffer zone
       (Parameters::safetyDistance).
    */
    boost::shared_ptr<const Hull> GetSafetyHull() const;    
    
    /**
       \return The radius of the robot's (non-grown) hull (max
       distance from robot center to any of the points on the hull).
    */
    double RobotRadius() const;
    
    /**
       \return The safety distance.
       \todo should be renamed SafetyDistance().
    */
    double SecurityDistance() const;
    
    /**
       \return The wheel base (distance between the two drive wheels
       on a differetially driven robot).
    */
    double WheelBase() const;
    
    /**
       \return The radius of the drive wheels.
    */
    double WheelRadius() const;
    
    /**
       \return The maximum actuator speed [rad/s].
    */
    double QdMax() const;
    
    /**
       \return The maximum actuator acceleration [rad/s/s].
    */
    double QddMax() const;
    
    /**
       \return The maximum (global) forward speed [m/s].
    */
    double SdMax() const;
    
    /**
       \return The maximum (global) rotational speed [rad/s].
    */
    double ThetadMax() const;
    
    /**
       \return The maximum (global) acceleration [m/s/s].
    */
    double SddMax() const;
    
    /**
       \return The maximum (global) rotational acceleration [rad/s/s].
    */
    double ThetaddMax() const;
    
    /**
       Direct kinematic model for differential drive robots. Given
       left and right actuator speeds, returns the corresponding
       global translational and rotational speeds. Takes into account
       the special cases of pure translation and pure rotation.
    */
    void Actuator2Global(double qdl, double qdr,
			 double & sd, double & thetad) const;
    
    /** convenient if you don't want to instantiate RobotModel */
    static void Actuator2Global(double qdl, double qdr,
				double wheelBase, double wheelRadius,
				double & sd, double & thetad);
    
    /**
       Indirect kinematic model for differential drive robots. Given
       the global translational and rotational speeds, calculates the
       corresponding left and right actuator speeds.
    */
    void Global2Actuator(double sd, double thetad,
			 double & qdl, double & qdr) const;
    
    /** convenient if you don't want to instantiate RobotModel */
    static void Global2Actuator(double sd, double thetad,
				double wheelBase, double wheelRadius,
				double & qdl, double & qdr);

    /**
       Given current actuator wheel speeds, predicts the robot's
       position if it brakes along a constant-curvature path until
       standstill. An additional safety delay (before the braking
       maneuver starts) can be added as well. The prediction is
       expressed in the local frame of reference, that is to say the
       current robot pose.
       
       \note See PredictStandstillGlob() for a version that takes
       global speeds as input.
    */    
    void PredictStandstillAct(double qdl, double qdr, double safety_delay,
			      double & dx, double & dy, double & dtheta) const;
    
    /**
       Like PredictStandstillAct() but takes global wheel speeds.
    */    
    void PredictStandstillGlob(double sd, double thetad, double safety_delay,
			       double & dx, double & dy, double & dtheta)
      const;
    
    /**
       Given (global) translational and rotational speeds, predicts
       the robot's position at a given timestep in the future. The
       prediction is expressed in the local frame of reference, that
       is to say the current robot pose.
    */
    static void LocalKinematics(double sd, double thetad,
				double timestep,
				double & deltax,
				double & deltay,
				double & deltatheta);
    
    /**
       Given the current and a desired (qdl, qdr) as well as a
       timestep within which to reach that speed, this method applies
       actuator acceleration limits, actuator speed limits, and global
       speed limits, and returns the possibly corrected command
       speeds.
       
       Notes:
       - global acceleration limits are not taken into account,
         just the actuator acceleration limits.
       - when speed or acceleration limits are hit, the resulting
         motion can be on a different radius of curvature than the
         originally desired (qdl, qdr) would have produced.
     */
    void ApplyActuatorLimits(double timestep,
			     double qdl_cur, double qdr_cur,
			     double qdl_des, double qdr_des,
			     double & qdl_cmd, double & qdr_cmd,
			     std::ostream * verbose_os);
    
    
  protected:
    const Parameters m_params;
    boost::shared_ptr<const Hull> m_hull;
    boost::shared_ptr<const Hull> m_safety_hull;
    mutable double m_robot_radius;
  };

}

#endif // SUNFLOWER_ROBOTMODEL_HPP
