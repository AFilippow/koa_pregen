#include "cspaceconverter.h"


using namespace std;
using namespace KDL;
cspaceconverter::cspaceconverter(){
	KukaChain = KukaLWR_DHnew();
	kinematic_solver = new ChainFkSolverPos_recursive(KukaChain);
	jointNumber = KukaChain.getNrOfJoints();

}
int cspaceconverter::examineDifference(vector<float> point1, vector<float> point2){
	//printf(" %f \n",vector_length(vector_difference(point1, point2)));
	if(vector_length(vector_difference(point1, point2))<0.03){
		//printf("Point found at %f,%f,%f \n",point1[0],point1[1],point1[2]);
		return 1;
	}
	else
		return 0;
}



/*
 *float vectordistance(KDL::JntArray q1, KDL::JntArray q2){
	float result = 0;
	for (int i = 0; i < 7; i++)
		result += (q1(i)-q2(i))*(q1(i)-q2(i));
	return sqrt(result);
}*/
float specialdistance(KDL::JntArray q1, vector<float> q2){
	float result = 0;
	for (int i = 0; i < 4; i++)
		result += (q1(i)-q2[i])*(q1(i)-q2[i]);
	return sqrt(result);
}
KDL::JntArray toJointArray(vector< float > i){
	KDL::JntArray output(i.size());
	for (int j = 0; j < i.size(); j++)
		output(j) = i[j];
	return output;
}
vector< float > toVector(KDL::JntArray i){
	vector< float > output(i.rows());
	for (int j = 0; j < i.rows(); j++)
		output[j] = i(j);
	return output;
}
void* coarsenYZData(void* slicenumber){
	int i = *(int*)slicenumber;

	FILE * yzslice;
	std::string line;
	int x;
	KDL::JntArray point(4);
	time_t start = time(0);
	//for (int i = 0; i < 62; i++){
		for(int j = 0; j < 22; j++){
			vector< vector< vector< float > > >obstacles(62);
			vector< vector< float > > counts(62);
			std::ifstream  slicefile(("/home/andrej/Workspace/cspoutput/yzslices/slice_"+boost::to_string(i)+"_"+boost::to_string(j)+".dat").c_str(), ios::in);		
			while (std::getline(slicefile, line)){
				std::istringstream iss(line);
				if (!(iss >> x  >> point(0) >> point(1) >> point(2) >> point(3))) { printf("error: cannot read line!\n");break; }
					
				int count;
				bool found = false;
				for (int i = 0; i < obstacles[x].size(); i++){
					if (specialdistance(point, obstacles[x][i]) < 0.2 && !found)
						{
							count = counts[x][i];
							for (int j = 0; j < 4; j++)
								obstacles[x][i][j] = (obstacles[x][i][j]+point(j)/((float)count))*((float)count)/((float)count+1);
							counts[x][i]++;
							found = true;
						}
				}
				if(!found){
					int size = obstacles[x].size();
					obstacles[x].resize(size +1);
					obstacles[x][size].resize(4);
					counts[x].resize(size+1);
					for (int j = 0; j < 4; j++)
						obstacles[x][size][j] = point(j);
					counts[x][size] = 1;
				}	
					
			}
		FILE * yzslice;
			for (int x1 = 0; x1 < 62; x1++){
				yzslice = fopen(("/home/andrej/Workspace/cspoutput/yzreduced/slice_"+boost::to_string(x1)+"_"+boost::to_string(i)+"_"+boost::to_string(j)+".dat").c_str(),"w");
				for (int i1 = 0; i1 < obstacles[x1].size(); i1++)
					fprintf(yzslice,"%i \t %f \t %f \t %f \t %f \t %i \n",x1, obstacles[x1][i1][0], obstacles[x1][i1][1], obstacles[x1][i1][2], obstacles[x1][i1][3], counts[x1][i1]);
			fclose(yzslice);
			
			}
			printf("slice %i, %i of 22 done after %f seconds \n",i ,j,difftime( time(0), start));

		}

	//}
}


void rearrangeSlicesByYZ(){
	printf("rearranging slices\n");
	/*vector<FILE *> yzslices(61*21); 
	for (int i = 0; i < 61; i++)
		for(int j = 0; j < 21; j++)
			yzslices[i*21+j] = fopen(("/home/andrej/Workspace/cspoutput/yzslices/slice_"+boost::to_string(i)+"_"+boost::to_string(j)+".dat").c_str(),"w");*/
	
	FILE * yzslice;
	
	std::string line;
	int x, y, z;
	float a, b;
	KDL::JntArray point(4);
	for (int i = 0; i < THREADNUMBER; i ++){
		printf("at file %i currently  \n",i);
		std::ifstream  slicefile(("/home/andrej/Workspace/cspoutput/sliceraster/slice_"+boost::to_string(i)+".dat").c_str(), ios::in);
		//std::cout << "Error: " << strerror(errno);
		
		while (std::getline(slicefile, line))
		{	
			//printf(line.c_str());
			std::istringstream iss(line);
			if (!(iss >> x >> y >> z >> a >> point(0) >> point(1) >> point(2) >> point(3) >> b)) { printf("error: cannot read line!\n");break; }
			//fprintf(yzslices[y*21+z],"%i \t %f \t %f \t %f \t %f \n", x, point(0), point(1), point(2), point(3));
			yzslice = fopen(("/home/andrej/Workspace/cspoutput/yzslices/slice_"+boost::to_string(y)+"_"+boost::to_string(z)+".dat").c_str(),"a");
			fprintf(yzslice,"%i \t %f \t %f \t %f \t %f \n", x, point(0), point(1), point(2), point(3));
			fclose(yzslice);
		}
	}
	/*for (volatile int i = 0; i < 61*21; i++)
		fclose(yzslices[i]);*/
}

void* calculateObstacleCenters(void *slicenumber){
	int i1 = *(int*)slicenumber;
	
	vector< vector< vector< float > > > obstacles(62*62*22);
	for (int i = 0; i < 62*62*22; i++){
		obstacles[i].resize(0);
		//obstacles[i][0].resize(5); //downsampled Obstacle points are to be 4 coords + number of points already folded into this one
	}
	std::ifstream  slicefile(("/home/andrej/Workspace/cspoutput/slices/slice_"+boost::to_string(i1)+".dat").c_str());
	std::string line;
	std::getline(slicefile,line); // get rid of the slice file header
	float x, y, z;
	KDL::JntArray point(4);
	while (std::getline(slicefile, line))
	{
		std::istringstream iss(line);
		
		if (!(iss >> x >> y >> z >> point(0) >> point(1) >> point(2) >> point(3) )) { break; }
		
		int x_index = floor((x+0.3)*61/0.6);
		if (x_index < 0 || x_index >= 62) printf("ERROR: x_index is %i \n", x_index);
		
		int y_index = floor((y+0.15)*61/0.6);
		if (y_index < 0 || y_index >= 62) printf("ERROR: y_index is %i \n", y_index);
		
		int z_index = floor((z)*21/0.2);
		if (z_index < 0 || z_index >= 22) printf("ERROR: z_index is %i \n", z_index);
		
		int index = x_index*61+21 + y_index*21 + z_index;
		int count;
		bool found = false;
		for (int i = 0; i < obstacles[index].size(); i++){
			if (specialdistance(point, obstacles[index][i]) < 0.1 && !found)
				{
					count = obstacles[index][i][4];
					for (int j = 0; j < 4; j++)
						obstacles[index][i][j] = (obstacles[index][i][j]+point(j)/((float)count))*((float)count)/((float)count+1);
					obstacles[index][i][4]++;
					found = true;
				}
		}
		if(!found){
			int size = obstacles[index].size();
			obstacles[index].resize(size +1);
			obstacles[index][size].resize(5);
			for (int j = 0; j < 4; j++)
				obstacles[index][size][j] = point(j);
			obstacles[index][size][4] = 1;
		}	
	}
	FILE * raster = fopen(("/home/andrej/Workspace/cspoutput/sliceraster/slice_"+boost::to_string(i1)+".dat").c_str(),"w");
	for (int i = 0; i < 62; i++)
		for (int j = 0; j < 62; j++)
			for (int k = 0; k < 22; k++) {
				int index = i*62+22 + j*22 + k;
				int pointnumber = obstacles[index].size();
				for (int l = 0; l < pointnumber; l++){
					fprintf(raster,"%i \t %i \t %i \t %i \t",i, j, k, pointnumber);
					fprintf(raster,"%f \t %f \t %f \t %f \t %f \n", obstacles[index][l][0] , obstacles[index][l][1] , obstacles[index][l][2] , obstacles[index][l][3] , obstacles[index][l][4] );
					fflush(raster);
				}
			}
	fclose(raster);
	pthread_exit(NULL);
	}
void* runSlice(void *q3val){
	// I cannot make runSlice() a member function of cspaceconverter under c++03 and ROS hydro does not support c++2011
	// I need to initialise every part of cspaceconverter again =(
	Chain KukaChain = KukaLWR_DHnew();
	ChainFkSolverPos_recursive* kinematic_solver = new ChainFkSolverPos_recursive(KukaChain);
	int jointNumber = KukaChain.getNrOfJoints();
	KDL::Frame baseframe(KDL::Rotation::Quaternion(-0.444, 0.231, 0.40, 0.768), KDL::Vector(-0.4, 0.15, 0.35) ) ;
	int i1 = *(int*)q3val;
	KDL::JntArray q(jointNumber);
			q(2) = (-25.0f+(float)i1*0.5)*PI/30;//k*PI/30.0f;

			q(4) = 0;//m*PI/30.0f;
			q(5) = 0;//n*PI/30.0f;
			q(6) = 0;//o*PI/30.0f;


	vector< vector < float > > position;
	vector<float> a;

	FILE * slicefile = fopen(("/home/andrej/Workspace/cspoutput/slices/slice_"+boost::to_string(i1)+".dat").c_str(),"w");
	fprintf(slicefile,"SLICEFILE %i, joint position: %f \n",i1,q(2));

	for (float i = -29; i <= 29; i+=0.1)	//51
	{
		for (float j = -18; j <= 18; j+=0.1)	//37
		//for (float k = -25; k <= 25; k+=0.5) //51
		for (float l = -18; l <= 18; l+=0.1)	// 37
		{
			q(0) = i*PI/30.0f;
			q(1) = j*PI/30.0f;

			q(3) = l*PI/30.0f;

			KDL::Frame cartpos;
			kinematic_solver->JntToCart(q, cartpos);
			KDL::Vector position = baseframe*cartpos.p;

			/*
			if (position.x() < -0.25 || position.x() > 0.2)
				fprintf(slicefile,"X out of bounds: \t");
			if (position.y() < -0.1 || position.y() > 0.4)
				fprintf(slicefile,"Y out of bounds: \t");
			if (position.z() < 0 || position.z() > 0.2)
				fprintf(slicefile,"Z out of bounds: \t");
			*/
			if (!(position.x() < -0.3 || position.y() < -0.15 || position.z() < 0 || position.x() > 0.3 || position.y() > 0.45 || position.z() > 0.2))
			fprintf(slicefile,"%f \t %f \t %f \t %f \t %f \t %f \t %f \n",position.x(), position.y(), position.z(),q(0),q(1),q(2),q(3));	
			
			
		}
	printf("slice %i at %f percent \n", i1, ((float)i+29)*100.0f/58.0f);
	}
	pthread_exit(NULL);
}
void cspaceconverter::generate_points_data(KDL::Frame k) {
	baseframe = k;




	pthread_t threads[THREADNUMBER];
	int **args = new int*[THREADNUMBER];
	
	for(int i = 0; i < THREADNUMBER; i++){
		args[i]  = new int[1];
		args[i][0] = i;
		int rc = pthread_create(&threads[i], NULL, runSlice, (void*)args[i]);
		//int rc = pthread_create(&threads[i], NULL, calculateObstacleCenters, (void*)args[i]);
	}
	for(int i = 0; i < THREADNUMBER; i++){
		void **status;
		int rc = pthread_join(threads[i],status);
	}
	for(int i = 0; i < THREADNUMBER; i++){
		args[i]  = new int[1];
		args[i][0] = i;
		//int rc = pthread_create(&threads[i], NULL, runSlice, (void*)args[i]);
		int rc = pthread_create(&threads[i], NULL, calculateObstacleCenters, (void*)args[i]);
	}
	for(int i = 0; i < THREADNUMBER; i++){
		void **status;
		int rc = pthread_join(threads[i],status);
	}
	
	
	rearrangeSlicesByYZ();
	
	for(int i = 0; i < THREADNUMBER; i++){
		args[i]  = new int[1];
		args[i][0] = i;
		//int rc = pthread_create(&threads[i], NULL, runSlice, (void*)args[i]);
		//int rc = pthread_create(&threads[i], NULL, calculateObstacleCenters, (void*)args[i]);
		int rc = pthread_create(&threads[i], NULL, coarsenYZData, (void*)args[i]);
		
	}
	pthread_t extrathreads[1]; //just for coarsenYZData
	int * arg = new int;
	arg[0] = THREADNUMBER;
	pthread_create(&extrathreads[0], NULL, coarsenYZData, (void*)arg);
	
	for(int i = 0; i < THREADNUMBER; i++){
		void **status;
		int rc = pthread_join(threads[i],status);
	}
	
	
	void **status1;
	int rc = pthread_join(extrathreads[0],status1);
	
	}
vector<float> cspaceconverter::joint_to_cartesian(vector<float> jointvalues, int segmentnumber){
	
	//printf("parsing %i values for %i joints \n", jointvalues.size(), jointNumber);
	vector<float> cartesianvalues(3, 0);
	if (jointvalues.size() == 0) 
		return cartesianvalues;

	KDL::Frame cartpos;
	KDL::JntArray q(jointNumber);
	for (int i = 0; i < jointNumber; i++)
		q(i) = jointvalues[i];
		
		
	bool kinematics_status = kinematic_solver->JntToCart(q, cartpos, segmentnumber);
	

	cartesianvalues[0] = cartpos.p.x();
	cartesianvalues[1] = cartpos.p.y();
	cartesianvalues[2] = cartpos.p.z();
	return cartesianvalues;
}


/*
 * 
0.39*cos(a)*cos(d)*sin(b) + (0.39*cos(a)*cos(b)*cos(c) - 0.39*sin(a)*sin(c))*sin(e) + 0.4*cos(a)*sin(b)
0.39*cos(d)*sin(a)*sin(b) + (0.39*cos(b)*cos(c)*sin(a) + 0.39*cos(a)*sin(c))*sin(e) + 0.4*sin(a)*sin(b)
                                           -0.39*cos(c)*sin(e)*sin(b) + 0.39*cos(b)*cos(d) + 0.4*cos(b)
 * 
 * 
 */
