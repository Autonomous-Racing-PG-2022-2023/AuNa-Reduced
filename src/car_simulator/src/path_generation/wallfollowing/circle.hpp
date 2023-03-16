#pragma once

#include <vector>

#include "car_simulator_msgs/msg/circle.hpp"

#include "geometric_math.hpp"

template<typename PointT>
class Circle
{
    PointT m_center;
    double m_radius;

    public:
    Circle(PointT center, double radius)
	:m_center(center), m_radius(radius)
    {}

    Circle()
	:m_center(), m_radius(0.0)
    {}

    inline double getRadius() const
    {
        return m_radius;
    }
	
    inline PointT& getCenter()
    {
        return m_center;
    }
	
	inline const PointT& getCenter() const
    {
        return m_center;
    }

    inline PointT getClosestPoint(const PointT& point) const
	{
		double x = point.x - m_center.x;
		double y = point.y - m_center.y;
		double distance = std::sqrt(x * x + y * y);
		
		PointT ret;
		ret.x = m_center.x + x * m_radius / distance;
		ret.y = m_center.y + y * m_radius / distance;
		
		return ret;
	}

    inline std::vector<PointT> calcIntersections(const Circle<PointT>& circle) const
	{
		std::vector<PointT> intersections(2);
		// calc vector between the two circles
		PointT ab;
		ab.x = circle.getCenter().x - getCenter().x;
		ab.y = circle.getCenter().y - getCenter().y;
		
		const double norm = GeometricFunctions::distance(ab, PointT());
		if (norm == 0)
		{
			// no distance between centers
			return intersections;
		}
		const double x = (getRadius() * getRadius() + norm * norm - circle.getRadius() * circle.getRadius()) / (2 * norm);
		double y = getRadius() * getRadius() - x * x;
		if (y < 0)
		{
			// no intersection
			return intersections;
		}
		else if (y > 0)
		{
			y = std::sqrt(y);
		}
		// compute unit vectors
		PointT ex;
		ex.x = ab.x / norm;
		ex.y = ab.y / norm;
		
		PointT ey;
		ey.x = -ex.y;
		ey.y = ex.x;
		
		// compute intersection
		PointT Q1;
		Q1.x = getCenter().x + x * ex.x;
		Q1.y = getCenter().y + x * ex.y;
		
		if (y == 0)
		{
			// only one intersection
			intersections.push_back(Q1);
			return intersections;
		}
		PointT Q2;
		Q2.x = Q1.x - y * ey.x;
		Q2.y = Q1.y - y * ey.y;
		
		Q1.x = Q1.x + y * ey.x;
		Q1.y = Q1.y + y * ey.y;
		intersections.push_back(Q1);
		intersections.push_back(Q2);
		return intersections;
	}
	
    inline std::vector<PointT> calcTangents(const PointT& outside_point) const
	{
		const PointT center = getCenter();
		const double radius = GeometricFunctions::distance(center, outside_point) / 2.0;
		
		PointT circleCenter;
		circleCenter.x = (center.x - outside_point.x) / 2.0;
		circleCenter.y = (center.y - outside_point.y) / 2.0;
		const Circle<PointT> circle(circleCenter, radius);
		std::vector<PointT> intersections = calcIntersections(circle);
		std::sort(intersections.begin(), intersections.end(), [](const PointT& a, const PointT& b) {
			return a.y > b.y; 
		});
		return intersections;
	}
	
    inline double getDistance(PointT& outside_point) const
	{
		return std::fabs(
					GeometricFunctions::distance(outside_point, m_center)
					- m_radius
				);
	}
	
	static inline void fromROSMsg(const car_simulator_msgs::msg::Circle& circle_in, Circle<PointT>& circle_out){
		circle_out.m_center.x = circle_in.center.x;
		circle_out.m_center.y = circle_in.center.y;
		circle_out.m_center.z = circle_in.center.z;
		circle_out.m_radius = circle_in.radius;
	}
	
	static inline void toROSMsg(const Circle<PointT>& circle_in, car_simulator_msgs::msg::Circle& circle_out){
		circle_out.center.x = circle_in.m_center.x;
		circle_out.center.y = circle_in.m_center.y;
		circle_out.center.z = circle_in.m_center.z;
		circle_out.radius = circle_in.m_radius;
	}
};