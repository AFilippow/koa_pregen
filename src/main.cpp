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
   
#include "segmenter.h"
#include "xdmp.h"
#include "vrepComm.h"
#include <time.h> 

#include "cspaceconverter.h"

//SocketComm was written to communicate with the Morse simulator, V-REP does not need it
//#include "SocketComm.h"

#define NIL (0) 
#define PI 3.14152
typedef pcl::PointCloud<pcl::PointXYZ> mPointCloudType;
typedef pcl::PointCloud<pcl::PointXYZRGB> mPointCloudTypeColor;
typedef pcl::PointXYZ mPointType; 

mPointCloudType::Ptr currCloud;  
mPointCloudTypeColor::Ptr newCloud;
mPointCloudTypeColor::Ptr obstacleCloud;
int dmpDimensions;
boost::mutex m_keycloud;
int currKeyFrameID;
vector<float> y;
bool mode;
pclsegmenter segm;
float angle;  
bool perfectObstacle;  
bool fixedcloud;
int dmp_dim;
float T; //seconds
float tau;
float dt; //seconds
int n; 
int flipTrajectory;
float sigma; 
float mass_center_distance;
float closest_distance;
vector<float> obst(0);
vector<float> dist(0);  
std::vector<mPointTypeColor> previousObstacles; 
std::vector<float> obstaclePersistenceWeight;
vector<float> s;
vector<float> e;
tf::TransformListener* listener;
tf::TransformListener* kukabaseListener;
tf::StampedTransform * frameTransform;
tf::StampedTransform * kukaBaseTransform;
xDMP dmp;

cspaceconverter * CSP;

FILE * cspobst;
FILE * cspobst2;
FILE * cspobst3;


/*void deepCopy(mPointCloudTypeColor::Ptr givenCloud, mPointCloudTypeColor::Ptr copyCloud ){
	copyCloud.reset(new mPointCloudTypeColor);
	strcpy(copyCloud->header.frame_id, givenCloud->header.frame_id);
	copyCloud->height = givenCloud->height;
	copyCloud->width = givenCloud->width;
	for (unsigned int i = 0; i < givenCloud->points.size(); i++)
		copyCloud->points[i] = givenCloud->points[i];
	return;	
}*/

float randomNumber() //between 0 and 1
{
	return ((float) (rand()%10000))/10000;
}





vector<float> partialKinematic(vector<float> jointValues, float linkLength){
	
	float a,b,c,d;
	a = jointValues[0];
	b = jointValues[1];
	c = jointValues[2];
	d = linkLength;
	vector<float> position(3);
	position[0] = (cos(a)*cos(b)*cos(c) - sin(a)*sin(c))*d + cos(a)*cos(b);
	position[1] = (cos(b)*cos(c)*sin(a) + cos(a)*sin(c))*d + cos(b)*sin(a);
	position[2] = d*cos(c)*sin(b) + sin(b);
	//printf("FK value: %f, %f \n", position[0], position[1]);
	

	return position;
}

float weighted_euclidean_distance(vector<float> a, vector<float> b){
	float distance = 0;	
	distance += (a[0]-b[0])*(a[0]-b[0])*0.2; 
	distance += (a[1]-b[1])*(a[1]-b[1])*0.5;
	distance += (a[2]-b[2])*(a[2]-b[2]);
	
return distance;
}

void pregenerateObstacles(){
	vector<float> o(3);
	o[0] = 0;
	o[1] = 0;
	o[2] = 0;
	vector<float> spaceobst = partialKinematic(o,1);
	std::cout << "obstacle x:" << spaceobst[0] << ", y:" << spaceobst[1] <<", z:" << spaceobst[3] << "\n";
	obst.resize(0);
	dist.resize(0);
	for(float i = -1.7; i < 1; i += 0.05)
		for(float j = -1.525; j < 1.5; j += 0.05)
		for(float k = -1.525; k < 1.5; k += 0.05)
		{
			vector<float> a(3);
			a[0] = i;
			a[1] = j;
			a[2] = k;
			

			for (float g = 0; g < 1; g+= 0.1){
				vector<float> fkobstacle = partialKinematic(a,g);
				if (vector_length(vector_difference( fkobstacle, spaceobst))<0.15 /*|| partialKinematic(a,g)[2] < 0*/)	
				{
					obst.resize(obst.size()+3);
					dist.resize(dist.size()+1);	
					obst[obst.size()-3] = a[0];
					obst[obst.size()-2] = a[1];
					obst[obst.size()-1] = a[2];
					//fprintf(cspobst,"%f \t %f \t %f  \t %f  \t %f  \t %f \n", a[0], a[1], a[2] ,fkobstacle[0],fkobstacle[1],fkobstacle[2]);
					break;
				}
			}

			
		}
	printf("Found %i c-space obstacles. %i!\n", obst.size()/3, dist.size());
}	
int selectObstacleBasedOnDist(){
	int index = 0;
	float mindist = 1000;	
	for (int i = 0; i < dist.size(); i++){
		if (dist[i] < mindist){
			mindist = dist[i];
			index = i;
		}
	}
	return index*3;
}
void setPerfectObstacle(vector<float> position){
	//printf("currently at %s \n", __func__);
	/*obst.resize(7);    //7 dimensions
	dist.resize(1);	
	obst[0] =  0;
	obst[1] =  0;
	obst[2] =  0;
	obst[3] =  0;
	obst[4] =  0;
	obst[5] =  0;
	obst[6] =  0;*/
	///Use this only if not using pregeneration
	/*obst.resize(2); //2 dimensions
	dist.resize(1);	
	obst[0] = -1.49;
	obst[1] = 0;
	dist[0] = 0;
	vector<float> spacepos = forwardsKinematic(position);
	vector<float> spaceobst = forwardsKinematic(obst);
		*/
			
	for (int j = 0; j < obst.size(); j +=3){
		vector<float> a(3);
		a[0] = obst[j];
		a[1] = obst[j+1]; 
		a[2] = obst[j+2]; 
		dist[j/3] = 0;
		for (int i = 0; i < 3; i ++)
			dist[j/3] += (a[i]-position[i])*(a[i]-position[i]);
		dist[j/3] = sqrt(dist[j/3]);
		if (dist[j/3] < 0.001){
			printf("Error at obstacle index %i! no distance calculated.\n",j/3);
			printf("y: %f, %f, %f;  obstacle: %f, %f, %f;\n",position[0], position[1],position[2],a[0],a[1],a[2]);
		}
	}
	

	///[TODO] fix distances
	//dist[0] = sqrt((obst[0] - segm.position.x)*(obst[0] - segm.position.x) + (obst[1] - segm.position.y) *(obst[1] - segm.position.y) + (obst[2] - segm.position.z)*(obst[2]- segm.position.z));
	//dist[1] = sqrt((obst[3] - segm.position.x)*(obst[3] - segm.position.x) + (obst[4] - segm.position.y) *(obst[4] - segm.position.y) + (obst[5] - segm.position.z)*(obst[5]- segm.position.z));
	//dist[2] = sqrt((obst[6] - segm.position.x)*(obst[6] - segm.position.x) + (obst[7] - segm.position.y) *(obst[7] - segm.position.y) + (obst[8] - segm.position.z)*(obst[8]- segm.position.z));
}

void paintObstacles(){

	//std::cout << "Number of obstacles:" <<segm.obstacles.size() << "\n";
	obstacleCloud->height = 1;
	obstacleCloud->width = segm.obstacles.size()+2;
	obstacleCloud->points.resize(obstacleCloud->width);
	if (obstacleCloud->points.size() > 0)
	{
		obstacleCloud->points[0].x = y[0];
		obstacleCloud->points[0].y = y[1];
		obstacleCloud->points[0].z = y[2];
		obstacleCloud->points[0].r = 0;
		obstacleCloud->points[0].g = 0;
		obstacleCloud->points[0].b = 250;
		obstacleCloud->points[1].x = 0;
		obstacleCloud->points[1].y = 0;
		obstacleCloud->points[1].z = 0;
		obstacleCloud->points[1].r = 0;
		obstacleCloud->points[1].g = 0;
		obstacleCloud->points[1].b = 0;
	
	}
	for (unsigned int i = 0; i < segm.obstacles.size(); i++)
		{
		obstacleCloud->points[i+2].x = segm.obstacles[i].x;
		obstacleCloud->points[i+2].y = segm.obstacles[i].y;
		obstacleCloud->points[i+2].z = segm.obstacles[i].z;
		obstacleCloud->points[i+2].r = segm.obstacles[i].r*2;
		obstacleCloud->points[i+2].g = segm.obstacles[i].g*2;
		obstacleCloud->points[i+2].b = segm.obstacles[i].b*2;			
		
		}	

}

void segmentOnceAndGetClouds(){
	//printf("currently at %s \n", __func__);
	newCloud.reset(new mPointCloudTypeColor);
	obstacleCloud.reset(new mPointCloudTypeColor);
	segm.segment();
	//std::cout <<"Cloud segments found: "<< segm.segment() << std::endl;
	newCloud =  segm.getColoredCloud()->makeShared();
	//segm.excludeObstacle(segm.position);
	paintObstacles();
}


void callback(const sensor_msgs::PointCloud2 inputROSMsg_tracker){
	//printf("currently at %s \n", __func__);
	boost::mutex::scoped_lock lock (m_keycloud);   
	currCloud.reset(new mPointCloudType);

	
	pcl::fromROSMsg ( inputROSMsg_tracker,*currCloud); //convert the cloud 
	
	pcl_ros::transformPointCloud(*currCloud, *currCloud, ((tf::Transform*)(frameTransform))->inverse());
	//pcl_ros::transformPointCloud(*currCloud, *currCloud, *frameTransform);
	
	currCloud->header.seq++; 
	pcl::VoxelGrid< pcl::PointXYZ > sor;
	sor.setInputCloud (currCloud);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*currCloud);
	currKeyFrameID=currCloud->header.seq;
	
	
	//   std::cout << "point at at: "<< currCloud->points[0].x << " " << currCloud->points[0].y << " " << currCloud->points[0].z << "\n ";
	
	
	segm.setRawCloud(currCloud);
	

	/*->header.frame_id = "some_tf_frame";
	newCloud->height = newCloud->width = 1;
	newCloud->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));
	*/
	//color_depth();
}

void load_trajectory(char* filein, int n, int dim, xDMP *dmp)
{   
	//printf("currently at %s \n", __func__);
	string tempStr;
	fstream fileIn(filein);
	float** w = new float*[dim];
	for (int i = 0; i < dim; i++)
		w[i] = new float[n];



	for (int i = 0; i < n; i ++)
	{
		for (int j = 0; j < dim; j++)
			w[j][i] = 0;
	}
	int i = 0;
	while (!fileIn.eof() && i < n)
	{
		int parsedCharacters = 0;
		getline(fileIn,tempStr);
		for (int j = 0; j < dim; j++)
		{
			int a;
			if (sscanf(tempStr.c_str()+parsedCharacters,"%f %n", &w[j][i], &a) > 0)
			{
				parsedCharacters += a;
				printf("parsed w[%i][%i] to %f \n", j,i,w[j][i]);
			}
			else 
			{
				w[j][i] = 0;
				printf("parse failed");
			}
		}
		i++;
   }
   	std::cout << ".\n";
   fileIn.close();
	
	for(int i = 0; i < n; i++)
	{
		printf("W: ");
		for(int j = 0; j < dim; j++){
			printf(" %f,", w[j][i]);
		}
		printf("\n");
	}
		
	
	vector<float> wtemp;
	wtemp.resize(dim);
   for (int i=0; i<n; i++)
	{
		for(int j = 0; j < dim; j++)
			wtemp[j] = w[j][i];
	   dmp->set_w(i, wtemp);
	}
		std::cout << "done\n";
   	for (int i = 0; i < dim; i++)
		delete [] w[i];
}


void calculate_obstacle_persistence()
{
	//printf("currently at %s \n", __func__);
	//if (!segm) return;
	std::vector<float> newObstaclePersistenceWeight(segm.obstacles.size());
	for (unsigned int i = 0; i < newObstaclePersistenceWeight.size(); i++)
		newObstaclePersistenceWeight[i] = 0;
	std::vector<bool> obstacleWasFoundAgain(obstaclePersistenceWeight.size());
	for (unsigned int i = 0; i < obstacleWasFoundAgain.size(); i++)
		obstacleWasFoundAgain[i] = false;
	for (unsigned int oldObstacleNumber = 0; oldObstacleNumber < obstaclePersistenceWeight.size(); oldObstacleNumber++)
	{
		for (unsigned int newObstacleNumber = 0; newObstacleNumber < segm.obstacles.size(); newObstacleNumber++)
		{
			if (pcl::euclideanDistance( previousObstacles[oldObstacleNumber], segm.obstacles[newObstacleNumber]) < 0.03)
			{
				obstacleWasFoundAgain[oldObstacleNumber] = true;
				newObstaclePersistenceWeight[newObstacleNumber] = 1;
				//add here if we want obstacles to stick to their previous positions
			}
			
		}
	}
	for (unsigned int oldObstacleNumber = 0; oldObstacleNumber < obstacleWasFoundAgain.size(); oldObstacleNumber++)
	{
		if (!obstacleWasFoundAgain[oldObstacleNumber] && obstaclePersistenceWeight[oldObstacleNumber] > 0.1)
			{
				segm.obstacles.push_back(mPointTypeColor(previousObstacles[oldObstacleNumber]));
				newObstaclePersistenceWeight.push_back(obstaclePersistenceWeight[oldObstacleNumber]-0.1);
			}	
	}
	for (unsigned int newObstacleNumber = 0; newObstacleNumber < segm.obstacles.size(); newObstacleNumber++)
	{
		if (newObstaclePersistenceWeight[newObstacleNumber] == 0)
			newObstaclePersistenceWeight[newObstacleNumber] = 1;
	}
	obstaclePersistenceWeight = newObstaclePersistenceWeight;
	previousObstacles = segm.obstacles;
}





///This function retrieves the obstacles from the segmenter in real time
void getLiveObstacleData(float t){
	//printf("currently at %s \n", __func__);
	segmentOnceAndGetClouds();
	if((segm.obstacles.size()>0 && t > 0.05*tau*T) && !perfectObstacle)
		{
			obst.resize(segm.obstacles.size()*3);
			dist.resize(obst.size()/3);
			//printf("%u obstacles found: \n",segm.obstacles.size());
			for(unsigned int i = 0; i < obst.size(); i+=3)
			{		
				//std::cout << "obstacle" << i/3 << " \n";
				//printf("nr. %i: %f, %f, %f \n",i,segm.obstacles[i/3].x,segm.obstacles[i/3].y,segm.obstacles[i/3].z);
				obst[i]=segm.obstacles[i/3].x;
				obst[i+1]=segm.obstacles[i/3].y;
				obst[i+2]=segm.obstacles[i/3].z;
				//dist[i/3] = pcl::euclideanDistance(segm.obstacles[i], mPointType(y[0], y[1], y[2]));
				//dist[i/3] =sqrt((segm.obstacles[i].x - segm.position.x)*(segm.obstacles[i].x - segm.position.x) + (segm.obstacles[i].y - segm.position.y) *(segm.obstacles[i].y - segm.position.y) + (segm.obstacles[i].z - segm.position.z)*(segm.obstacles[i].z - segm.position.z));
				dist[i/3] = pcl::euclideanDistance(mPointType(obst[i], obst[i+1], obst[i+2]), mPointType(y[0], y[1], y[2]));
				if (dist[i/3] == -1) std::cout <<"Warning: distance estimation failure";
				dist[i/3] -= 0.05;
				//dist[i/3]=segm.distanceOfObstacleToPosition(segm.obstacles[i], mPointType(y[0], y[1], y[2]))-0.1; 
			}
			closest_distance = pcl::euclideanDistance(mPointType(obst[0], obst[1], obst[2]), mPointType(y[0], y[1], y[2]));
		}


}

void getSavedObstacleData(float t){
	//printf("currently at %s \n", __func__);
	segm.findClosestPoints();	
	segm.excludeObstacle(mPointType(-0.143, 0.571, 0.4));
	if(segm.obstacles.size()>0 )
	{
		//just in case, keep the resizing
		obst.resize(segm.obstacles.size()*3);
		dist.resize(segm.obstacles.size());
		for (unsigned int i = 0; i < dist.size(); i++)
			dist[i] = -2;
		for (unsigned int i = 0; i < obst.size(); i++)
			obst[i] = -2;

		for(unsigned int i = 0; i < obst.size(); i+=3)
		{
			obst[i]=segm.obstacles[i/3].x;
			obst[i+1]=segm.obstacles[i/3].y;
			obst[i+2]=segm.obstacles[i/3].z;
			//dist[i/3] = pcl::euclideanDistance(segm.obstacles[i], mPointType(y[0], y[1], y[2]));
			//dist[i/3] =sqrt((segm.obstacles[i].x - segm.position.x)*(segm.obstacles[i].x - segm.position.x) + (segm.obstacles[i].y - segm.position.y) *(segm.obstacles[i].y - segm.position.y) + (segm.obstacles[i].z - segm.position.z)*(segm.obstacles[i].z - segm.position.z));
			dist[i/3] = pcl::euclideanDistance(mPointType(obst[i], obst[i+1], obst[i+2]), mPointType(y[0], y[1], y[2]));
			
			//printf("obst %i is %f,%f,%f  \n",i/3,obst[i],obst[i+1],obst[i+2]);
			
			//printf("dist %i is %f \n",i/3,dist[i/3]);
			if (dist[i/3] == -1) std::cout <<"Warning: distance estimation failure\n";
			if (dist[i/3] == -2) std::cout <<"Warning: distance initialisation failed\n";
			//dist[i/3]=segm.distanceOfObstacleToPosition(segm.obstacles[i], mPointType(y[0], y[1], y[2]))-0.1; 
		}
		closest_distance = pcl::euclideanDistance(mPointType(obst[0], obst[1], obst[2]), mPointType(y[0], y[1], y[2]));
	}
	paintObstacles();
}

void initGlobalParams(int argc, char** argv){
	//printf("currently at %s \n", __func__);
	srand(time(NULL));
	mass_center_distance = 0;
	closest_distance = 0;
	mode = false;
	perfectObstacle = false;
	fixedcloud = false;
	flipTrajectory = -1;
	/*if (argc > 1)
		for(int i = 1; i < argc; i++)
		{
		if (strcmp(argv[i], "-noobstacle"))
		mode = true;
		
		//if (strcmp(argv[i], "-m"))
		//	mode = true;
		//if (strcmp(argv[i], "-perfect"))
		//	perfectObstacle = true;
		}*/
	std::cout << "Initialising point cloud...\n";
	// Point Cloud initialisation
	currKeyFrameID = -1;
	newCloud.reset(new mPointCloudTypeColor);
	newCloud->header.frame_id = "kuka";
	newCloud->height = newCloud->width = 1;
	newCloud->points.push_back (pcl::PointXYZRGB());
	obstacleCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	obstacleCloud->header.frame_id = "kuka";
	obstacleCloud->height = obstacleCloud->width = 1;
	obstacleCloud->points.push_back (pcl::PointXYZRGB());
	frameTransform = new tf::StampedTransform( tf::Transform::getIdentity() , ros::Time::now(), "world", "kuka");
	kukaBaseTransform = new tf::StampedTransform( tf::Transform::getIdentity() , ros::Time::now(), "world", "kuka_base");
}

void initDMP(){
	dmpDimensions = 3;
	//printf("currently at %s \n", __func__);
	y.resize(dmpDimensions);
	std::cout << "Initialising dmp\n";
	dmp_dim=dmpDimensions;
	T=3; //seconds
	tau=1;
	dt=0.005; ///change here important ----- seconds
	n=3; //number of cores
	sigma=1; 
	//Start and endpoint
	/*s.resize(7);
	s[0] =  1.0212134122849; ///7 dimensions
	s[1] = -1.6560771465302;
	s[2] = -1.5361843109131;
	s[3] =-0.90436959266663;
	s[4] = 0.50615078210831;
	s[5] =-0.69563746452332;
	s[6] = 0.59889233112335;
	e.resize(7);
	e[0] = 0.40823304653168;
	e[1] = -1.3737616539001;
	e[2] = -1.9576900005341;
	e[3] = -1.4553508758545;
	e[4] = 0.26177817583084;
	e[5] =-0.64877796173096;
	e[6] = 0.59570634365082;*/
	//important!
	s.resize(dmpDimensions);
	e.resize(dmpDimensions);
	s[0] = -1;
	s[1] = 0.01;
	s[2] = 0;
	e[0] = 1;
	e[1] = 0.01;
	e[2] = 0;
	
	dmp.init_dmp(dmp_dim, s, e, T, dt, tau, n, sigma);

}
void moveObstacles(vrepComm vcom){
	
	float radius = 0.1+randomNumber()/4;
	float alpha = randomNumber()*3.141572*2;
	for (int i = 1; i <= 3; i++){
		vcom.moveObstacleNr(i-1, radius*sin(alpha+i*3.141572*2/3), radius*cos(alpha+i*3.141572*2/3), i == 3? 0.15:0.075);
	}
	sleep(3);
}

//---------------------------------------------------------------------------------------------------------------------//
int main(int argc, char** argv)
{ 
	CSP = new cspaceconverter();
	KDL::Frame k(KDL::Rotation::Quaternion(-0.444, 0.231, 0.40, 0.768), KDL::Vector(-0.35, 0.05, 0.35)  ); //listen to the rotation with "rosrun tf tf_echo KUKA_base world" from console

	CSP->generate_points_data(k);
	return 1;
	
	
	//cspobst = fopen("/home/andrej/Workspace/cspaceobstacles.txt","w");
	//cspobst2 = fopen("/home/andrej/Workspace/cspaceobstacles2.txt","w");
	//cspobst3 = fopen("/home/andrej/Workspace/cspaceobstacles3.txt","w");
	ros::init(argc, argv, "koa");
	ros::NodeHandle nh("~");  
	initGlobalParams(argc, argv);
	std::string source_, sim_topic_;
	
	nh.getParam ("source", source_);
	nh.getParam ("sim_topic", sim_topic_);   
	nh.getParam ("perfect_Obstacle", perfectObstacle);
	nh.getParam ("no_Obstacle", mode);	
	nh.getParam ("fixed_cloud", fixedcloud);
	ros::Subscriber sub_sim = nh.subscribe<sensor_msgs::PointCloud2>("/"+source_+"/"+ sim_topic_, 1, callback);
	ros::Publisher output = nh.advertise<sensor_msgs::PointCloud2>("segmentedOutput", 10);
	ros::Publisher output2 = nh.advertise<sensor_msgs::PointCloud2>("obstacles", 10);
	
	ros::Rate r(30); 
	// DMP initialisation
	initDMP();
	std::cout << ".\n";
	//[TODO] fix this absolute link
	load_trajectory("/home/andrej/Workspace/koa/t2.txt", n, dmpDimensions,  &dmp);
	float t=0;
	//Comm initialisation
		std::cout << "Initialising VREP Comm...\n";
	//SocketComm scom;
	vrepComm vcom(&nh);

		std::cout << "spinning...\n";
	//segmenter init
	segm.position.x = 0;
	segm.position.y = 0;
	segm.position.z = 0;	
		
	tf::TransformBroadcaster br;
	listener = new   tf::TransformListener;
	kukabaseListener = new   tf::TransformListener;
	ofstream fileOutput("/home/andrej/Workspace/trajectiveCollisions.txt");	
	

	//for testing purposes:
	angle = 0.5;
	double angular_increment = 1/16 * 3.14152;



	angular_increment = (3.14152/8);
	std::cout << "angular_increment: " << angular_increment << "\n";

	pregenerateObstacles();

	while (!currCloud)
	{	
		std::cout << "no cloud yet \n";
		ros::spinOnce();
		sleep(1);
	}
	try{
			listener->lookupTransform( "kinect_visionSensor", "world", ros::Time(0), *frameTransform);
		}
		catch (tf::TransformException ex){
		  //ROS_ERROR("%s",ex.what());;
		}
	try{
			kukabaseListener->lookupTransform( "KUKA_base", "world", ros::Time(0), *kukaBaseTransform);
		}
		catch (tf::TransformException ex){
		  //ROS_ERROR("%s",ex.what());;
		}
		
	moveObstacles(vcom);
	if (fixedcloud) 
	{
		segmentOnceAndGetClouds();
		getSavedObstacleData(t);
	}
	


	int trialrun = 1;
	
	
	while (ros::ok()){
		if (currKeyFrameID >= 0)
			boost::mutex::scoped_lock lock (m_keycloud); 
		
		try{
			listener->lookupTransform( "kinect_visionSensor", "world", ros::Time(0), *frameTransform);
		}
		catch (tf::TransformException ex){
		  //ROS_ERROR("%s",ex.what());;
		}

		
		///change here
		//br.sendTransform(tf::StampedTransform(frameTransform->inverse(), ros::Time::now(), "world", "kuka"));
		
		tf::StampedTransform nullTransform(frameTransform->inverse(), ros::Time::now(), "world", "kuka");
		nullTransform.setIdentity ();
		br.sendTransform(nullTransform);
		
		/*if (!fixedcloud)
			if (segm.getColoredCloud()) 
				segm.excludeObstacle(segm.getColoredCloud(),  segm.position);  */
				
				
		try{
			kukabaseListener->lookupTransform( "KUKA_base", "world", ros::Time(0), *kukaBaseTransform);
		}
		catch (tf::TransformException ex){
		  //ROS_ERROR("%s",ex.what());;
		}
		
		
		
		vector<float> jointvalues(7, 0);
		jointvalues[1] = 1;
		vector<float> spacecoords;
		for (int i = 0; i < 7; i++)
		{
			spacecoords = CSP->joint_to_cartesian(jointvalues, i);
			printf("Joint %i at %f, %f, %f \n",i,spacecoords[0],spacecoords[1],spacecoords[2]);
		}
		 
		spacecoords = CSP->joint_to_cartesian(jointvalues);
		//THIS is how you transform points into the KUKA frame of reference
		//	to be used with forward kinematics		
		geometry_msgs::PointStamped kuka_end;
        kuka_end.header.frame_id = "KUKA_base";
        kuka_end.header.stamp = ros::Time();
        kuka_end.point.x = spacecoords[0];
        kuka_end.point.y = spacecoords[1];
        kuka_end.point.z = spacecoords[2];
		geometry_msgs::PointStamped base_point;
		try{
			
			kukabaseListener->transformPoint("world", kuka_end, base_point);

			ROS_INFO("point in kuka coords: (%.2f, %.2f. %.2f) -----> point in global coords: (%.2f, %.2f, %.2f) at time %.2f",
			kuka_end.point.x, kuka_end.point.y, kuka_end.point.z,
			base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
		}
		catch(tf::TransformException& ex){
			ROS_ERROR("Received an exception trying to transform a point from \"KUKA_base\" to \"world\": %s", ex.what());
		}
		vcom.sendJointAngles(jointvalues);
		printf("Transformed Coordinates: %f, %f, %f \n",base_point.point.x, base_point.point.y, base_point.point.z);
		
		
		break;
		
		
		
		
		
		
		
		
		
		
		if(t<=1.0*tau*T) //run cycle
		{

			y=dmp.get_y();
			closest_distance = -1;

			
			
			//Automatic obstacles
			if (fixedcloud)
				getSavedObstacleData(t);
			else
			{
				getLiveObstacleData(t);
			}
			
			if (perfectObstacle) //Offset is -1.9, -0.1, -0.1 
				setPerfectObstacle(y);
			
			//printf("Y: %f, %f;  dist: %f \n", y[0], y[1], dist[0]);
			//mass_center_distance = sqrt((-0.4 - segm.position.x)*(-0.4 - segm.position.x) + (-0.4 - segm.position.y) *(-0.4 - segm.position.y) + (0.3 - segm.position.z)*(0.3 - segm.position.z));
			
			///[TODO] here we need to transform our obstacle into joint space;
			//dmp.set_obstacle(obst, dist);
			if (obst.size() > 1 && !mode){
				vector<float> o(3);
				vector<float> d(1);
				int i = selectObstacleBasedOnDist();
				o[0] = obst[i];
				o[1] = obst[i+1];
				o[3] = obst[i+2];
				d[0] = dist[i/3];
				printf("i: %i;  d: %f \n",i/3,d[0]);
				dmp.set_obstacle(o, d);
			} else{
			printf("no obstacles found %i \n",obst.size());
			}
			vector<float> x = partialKinematic(y,1);
			printf("X: %f, %f, %f \n",x[0],x[1],x[2]);
			dmp.calculate_one_step_dmp(t);
			y=dmp.get_y();


			
				vcom.sendJointAngles(y);
			/*segm.position.x = y[0];
			segm.position.y = y[1];
			segm.position.z = y[2];*/
			
			if ( t < 0.05*tau*T && angle < 0.1 ) 
			{
			//std::cout << "Attempting grasp.\n";
			//scom.sendGrasp(grasp);
		
			}
					
			t=t+dt;
		}
		if (t>1.0*tau*T){
			t = 0;
			load_trajectory("/home/andrej/Workspace/koa/t2.txt", n, dmpDimensions, &dmp);
			angle += angular_increment;
			flipTrajectory = -flipTrajectory;
			//std::cout << "added "<<   angular_increment << "to angle\n";
			/*s[0] = 0.4*sin(angle)*flipTrajectory;
			s[1] = 0.5*cos(angle)*flipTrajectory;
			s[2] = 0.15;*/
			
			//e[0] = 0.3 + 0.2*randomNumber();
			//e[1] = 0.1 + 0.2*randomNumber();
			//e[2] = 0.1;			
			//s[0] = -0.5 + 0.2*randomNumber();
			//s[1] = -0.3 + 0.2*randomNumber();
			//s[2] = 0.15;			
			
			/*s[0] = 0.4;
			s[1] = 0.05;
			s[2] = 0.14;
			e[0] = -0.4;
			e[1] = -0.05;
			e[2] = 0.14;*/
			dmp.set_s(s);
			dmp.set_g(e);

			
			/*fclose(dmp.positionAndMotion);
			
			char result[100];
			sprintf(result,"/home/andrej/Workspace/xdmp_positionmotion%i.txt",trialrun);
		
			dmp.positionAndMotion = fopen(result,"w");*/
			
			/*switch (trialrun){ //0.1, 0.05
			case 1: dmp.motion_persistence_weight = 0.90; break;
			case 2: dmp.motion_persistence_weight = 0.95;break;			
			case 3: dmp.motion_persistence_weight = 0.97; break;
			case 4: dmp.motion_persistence_weight = 0.99; break;						
			}*/
			/*switch (trialrun){ //0.1, 0.05
			case 1: dmp.LAMBDA = 0.05; break;
			case 2: dmp.LAMBDA = 0.025; break;			
			case 3: dmp.LAMBDA = 0.01; break;
			case 4: dmp.LAMBDA = 0.005; break;						
			}*/
			trialrun++;
			//e[0] = -0.4*sin(angle)*flipTrajectory;
			//e[1] = -0.4*cos(angle)*flipTrajectory;
			//e[2] = 0.15;
			
			/*if (1 == flipTrajectory){
				dmp.set_s(y);
				dmp.set_g(s);
			}else{
				dmp.set_s(y);
				dmp.set_g(e);
			}*/
			
			moveObstacles(vcom);
			previousObstacles.clear(); 
			obstaclePersistenceWeight.clear();
			//segm.obstacles.clear();
			if (angle > 2.1)
			{
				fileOutput.close();
				std::cout << "Test finished! exiting...\n";
				return 1;
			}	
			
			std::cout<< "Collision status: " << vcom.getAnyCollisions() << endl;
			y = s;
			/*segm.position.x = y[0];
			segm.position.y = y[1];
			segm.position.z = y[2];
			dmp.reset();
			if (fixedcloud){
				 segmentOnceAndGetClouds();
				 //segm.excludeObstacle(mPointType(-0.143, 0.571, 0.4));
				 getSavedObstacleData(t);
			}*/
			}
	
	
		bool collided = false;
		//std::cout<< "Collision status: " << vcom.getAnyCollisions() << endl;
		
		
		
		//cout<< "obstacles memorized: " << previousObstacles.size() << "\n";
		//fileOutput << y[0] << "\t" << y[2] << "\t" << collided << "\t";
		//fileOutput << mass_center_distance << "\t" << closest_distance << "\t";
		//fileOutput << angle+ (angular_increment)*(t/(tau*T)) << "\t";
		//std::cout <<"current angle:" << angle+ (angular_increment)*(t/(tau*T)) << "\n";
		//fileOutput <<y[0] << "\t" <<  y[1] << "\t" <<  y[2] << "\t";
		if (dmp.deviation.size() > 0)
			fileOutput << dmp.deviation[0] << "\t" <<  dmp.deviation[1] << "\t" <<  dmp.deviation[2] << "\t";
		else
			fileOutput << "0 \t 0 \t 0 \t";
		//fileOutput << pcl::euclideanDistance(mPointType(additional_track_points[0], additional_track_points[1], additional_track_points[2]), mPointType(-0.5,-0.4,0.15)) << "\t";
		//fileOutput << pcl::euclideanDistance(mPointType(additional_track_points[3], additional_track_points[4], additional_track_points[5]), mPointType(-0.5,-0.4,0.15)) << "\t";
		fileOutput << std::endl;
		newCloud->header.frame_id = "kuka";
		obstacleCloud->header.frame_id = "kuka";
		output.publish(newCloud); 
		output2.publish(obstacleCloud); 
		ros::spinOnce();
		r.sleep();

	}
	
			//fclose(cspobst);
			//fclose(cspobst2);
			//fclose(cspobst3);
	return 0;
}
//---------------------------------------------------------------------------------------------------------------------//
 
