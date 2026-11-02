//Bangash Owais //June 8th, 2022
#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <pcl\io\pcd_io.h>
#include <pcl\io\ply_io.h>
#include <pcl\point_types.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PLYReader Reader;
  //Read .PLY file
    Reader.read("C:\\Users\\admin\\Desktop\\try - 001\\Bangash.ply", *cloud);
  
  //Load .PCD file
    pcl::io::loadPCDFile("C:\\Users\\admin\\Desktop\\try-001\\Bangash.pcd", *cloud);
  
  //Save .PLY file
    pcl::io::savePLYFile("C:\\Users\\admin\\Desktop\\try-001\\Bangash.ply", *cloud);
    system("pause");
    return 0;
}
