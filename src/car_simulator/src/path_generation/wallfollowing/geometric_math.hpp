#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace GeometricFunctions
{
	template<typename PointT>
    inline double distance(const PointT& a, const PointT& b)
	{
		return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
	}
} // namespace GeometricFunctions

template<typename PointT>
struct Line
{
    PointT start;
    PointT end;
	
	Line(PointT start, PointT end)
	: start(start), end(end)
	{}

    inline double length() const
    {
        return GeometricFunctions::distance(start, end);
    }
};

namespace GeometricFunctions
{
	template<typename PointT>
    inline double calcShortestDistanceToLine(const PointT& point, const Line<PointT>& line)
	{
		return (
				std::fabs(
						(line.end.y - line.start.y) * point.x
						- (line.end.x - line.start.x) * point.y
						+ line.end.x * line.start.y
						- line.end.y * line.start.x
						)
						/ line.length()
			);
	}
	
	template<typename PointT>
    inline double calcSignedShortestDistanceToLine(const PointT& point, const Line<PointT>& line)
	{
		return (
				(line.end.y - line.start.y) * point.x
				- (line.end.x - line.start.x) * point.y
				+ line.end.x * line.start.y
				- line.end.y * line.start.x
				)
				/ line.length()
			;
	}
} // namespace GeometricFunctions