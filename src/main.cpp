// those next two should be always called first!....  
#include <iostream> 
#include <cstring>
#include <sys/time.h>  
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/tracking/tracking.h>
#include <pcl/common/distances.h>
#include <pcl/search/kdtree.h>
#include <pcl/ros/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>

// ROS
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h> 

#include <sstream>  

#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>
#include <sys/time.h> 
 
#include <pcl/visualization/pcl_visualizer.h>  
#include <../../opt/ros/hydro/include/ros/init.h>
#include <sstream> 
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/point_cloud.h>   
#include <stdio.h>
#include <stdlib.h>  
#include <fstream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <time.h> 

#include "threadpool.h"

//SocketComm was written to communicate with the Morse simulator, V-REP does not need it
//#include "SocketComm.h"

#define NIL (0) 
#define PI 3.14152

//cspaceconverter * CSP;


//---------------------------------------------------------------------------------------------------------------------//
int main(int argc, char** argv)
{ 
	//CSP = new cspaceconverter();
	//KDL::Frame k(KDL::Rotation::Quaternion(-0.444, 0.231, 0.40, 0.768), KDL::Vector(-0.4, 0.15, 0.35)  ); //listen to the rotation with "rosrun tf tf_echo KUKA_base world" from console
	threadpool thrdpl(101, 10, 27);
	thrdpl.run();
	/*KDL::JntArray q(7);
	q(0) = 1.57;
	q(1) = 0;
	q(2) = 0;
	q(3) = 0;
	q(4) = 0;
	q(5) = 1.57;
	q(6) = 0;
	Chain KukaChain = KukaLWR_DHnew();
	ChainFkSolverPos_recursive kinematic_solver(KukaChain);
	KDL::Frame cartpos;
	kinematic_solver.JntToCart(q, cartpos);
	KDL::Vector handOffset;
	handOffset(0) = 0;
	handOffset(1) = 0;
	handOffset(2) = 0.2;

	KDL::Vector position = k*cartpos.p;
	KDL::Vector position2 = k*cartpos*handOffset;
	printf("Pos1 = %f, %f, %f: Pos2 = %f, %f, %f \n", position(0), position(1), position(2), position2(0), position2(1), position2(2));
	
	q(0) = 0;
	q(2) = 1.57;
	kinematic_solver.JntToCart(q, cartpos);
	position = k*cartpos.p;
	position2 = k*cartpos*handOffset;
	printf("Pos1 = %f, %f, %f: Pos2 = %f, %f, %f \n", position(0), position(1), position(2), position2(0), position2(1), position2(2));
	q(2) = 0;
	q(4) = 1.57;
	kinematic_solver.JntToCart(q, cartpos);
	position = k*cartpos.p;
	position2 = k*cartpos*handOffset;
	printf("Pos1 = %f, %f, %f: Pos2 = %f, %f, %f \n", position(0), position(1), position(2), position2(0), position2(1), position2(2));
	*/

	//CSP->generate_points_data(k);
	return 1;
}
//---------------------------------------------------------------------------------------------------------------------//
 
