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


#ifndef SUNFLOWER_FRAME_HPP
#define SUNFLOWER_FRAME_HPP


#include <iosfwd>


namespace sfl {


  class Pose;
  class Goal;


  /**
     Coordinate frame without covariance information. If you need
     probabilistic coordinate frames, use a sfl::Pose object.
  */
  class Frame
  {
  public:
    /** Default Frame at (0, 0, 0). */
    Frame();

    /** Create a Frame at (x, y, theta). */
    Frame(double x, double y, double theta);

    /** Make this instance equal to f. */
    void Set(const Frame & f);

    /** Redefine the Frame's position. */
    void Set(double x, double y, double theta);

    /** Retrieve the Frame's position. */
    void Get(double & x, double & y, double & theta) const;

    /** \return The Frame's x-coordinate. */
    double X() const;

    /** \return The Frame's y-coordinate. */
    double Y() const;

    /** \return The Frame's orientation (angle measured from the
	containing frame's x-axis to this frame's x-axis). */
    double Theta() const;

    /** \return The sine of Theta(). */
    double Sintheta() const;

    /** \return The cosine of Theta(). */
    double Costheta() const;

    /** Transform a point (x, y) given in this frame to the
	corresponding coordinates of the enclosing frame. */
    void To(double & x, double & y) const;

    /** Transform a Frame instance given in this frame to the
	corresponding coordinates of the enclosing frame. */
    void To(Frame & frame) const;

    /** Like To(Frame &) but takes three references to double. */
    void To(double & x, double & y, double & theta) const;

    /** Transform a Goal instance given in this frame to the
	corresponding coordinates of the enclosing frame. */
    void To(Goal & goal) const;

    /** Performs only the rotational part of To() for points (x,
	y). This corresponds to rotating the provided point around the
	frame's origin by Theta(). */
    void RotateTo(double & x, double & y) const;

    /** Performs only the rotational part of To() for Frame
	instances. This corresponds to rotating the provided instance
	around this frame's origin by Theta(). */
    void RotateTo(Frame & frame) const;

    /** The inverse of To(): Given a point (x, y) defined in the
       enclosing frame, this method calculates the point's coordinates
       with respect to this frame. */
    void From(double & x, double & y) const;

    /** The inverse of To() for Frame instances. */
    void From(Frame & frame) const;

    /** Like From(Frame &) but takes three references to double. */
    void From(double & x, double & y, double & theta) const;

    /** The inverse of To() for Goal instances. */
    void From(Goal & goal) const;

    /** Performs only the rotational part of From() for points (x,
       y). This corresponds to rotating the provided point around the
       frame's origin by the <em>negated</em> Theta(). */
    void RotateFrom(double & x, double & y) const;

    /** Performs only the rotational part of From() for Frame
       instances. This corresponds to rotating the provided instance
       around this frame's origin by the <em>negated</em> Theta(). */
    void RotateFrom(Frame & frame) const;

    /** Moves a Frame instance to a new position expressed in its own
       frame of reference. Very useful for integrating pose
       predictions, which are expressed in the local frame (see also
       RobotModel::LocalKinematics()). */
    void Add(double dx, double dy, double dtheta);

    /** Prints the frame in human-readable format. */
    friend std::ostream & operator << (std::ostream & os, const Frame & f);

    /** Convenience assignment operator. */
    Frame & operator = (const Frame & orig);

    /** Convenience conversion operator. */
    Frame & operator = (const Pose & orig);

    /** compute Euclidean distance between two frames */
    double DistanceDiff(const Frame&) const;

    /** compute absolute angular difference between two frames */
    double AngleDiff(const Frame&) const;

  protected:
    double m_x, m_y, m_theta, m_sintheta, m_costheta;
  };

}

#endif // SUNFLOWER_FRAME_HPP
