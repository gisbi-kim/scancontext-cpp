#ifndef _POINTCLOUD_H_
#define _POINTCLOUD_H_

#include <iostream>
#include <vector>
#include <string>
using namespace std;

namespace pointcloud
{

struct Point 
{
    float x;
    float y;
    float z;
}; // Class Point

struct Pointcloud
{
    std::vector<Point> points;
}; // Class Pointcloud



} // namespace pointcloud 
#endif // _POINTCLOUD_H_

