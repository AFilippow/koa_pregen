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

void *cspaceconverter::runSlice(void *q3val){
	
	clock_t start, now;
	vector<float> q(jointNumber);
	vector< vector < float > > position;

	int i = *(int*)q3val;
	FILE * slicefile = fopen(("/home/andrej/Workspace/cspoutput/slices/slice_"+boost::to_string(i)+".dat").c_str(),"w");
	start = clock();
	float count = 0;
	for (float i = -25; i <= 25; i+=0.5)	//51
	{
	for (float j = -18; j <= 18; j+=0.5)	//37

	//for (float k = -25; k <= 25; k+=0.5) //51
	for (float l = -18; l <= 18; l+=0.5)	// 37

	//for (float m = -25; m <= 25; m+=4.25)	// 12
	//for (float n = -18; n <= 18; n+=4.5)	// 8
	//for (float o = -25; o <= 25; o+=5) // 11
	{
		q[0] = i*PI/30.0f;
		q[1] = j*PI/30.0f;
		q[2] = -25.0f+(float)i*0.5;//k*PI/30.0f;
		q[3] = l*PI/30.0f;
		q[4] = 0;//m*PI/30.0f;
		q[5] = 0;//n*PI/30.0f;
		q[6] = 0;//o*PI/30.0f;
		
	}
	}
	
}
void cspaceconverter::generate_points_data(KDL::Frame baseframe) {
	
	
	vector< vector < FILE * > > pointfiles;
	pointfiles.resize(4);
	for (int i = 0; i < 4; i++){
		pointfiles[i].resize(4);
		for(int j = 0; j < 4; j++){
			pointfiles[i][j] = fopen(("/home/andrej/Workspace/cspoutput/csobstacle_"+boost::to_string(i)+"_pnt_"+boost::to_string(j)+".txt").c_str(),"w");
		}		
	}
	
	/*FILE * point0;
	FILE * point1;
	FILE * point2;
	FILE * point3;*/
	vector< vector < float > > pnt;
	pnt.resize(4);
	pnt[0].resize(3);
	pnt[0][0] = 0.63;
	pnt[0][1] = -0.07;
	pnt[0][2] = 0.08;
	pnt[1].resize(3);
	pnt[1][0] =  0.46;
	pnt[1][1] = 0.21;
	pnt[1][2] = 0.23;
	pnt[2].resize(3);
	pnt[2][0] = 0.51;
	pnt[2][1] = -0.05;
	pnt[2][2] = +0.31;
	pnt[3].resize(3);
	pnt[3][0] = 0.74;
	pnt[3][1] = 0.06;
	pnt[3][2] = +0.39;
	
	clock_t start, now;

	vector<float> q(jointNumber);
	vector< vector < float > > position;
	position.resize(4);
	start = clock();
	float count = 0;
	//for (float i = -25; i <= 25; i+=2)
	for (float i = -25; i <= 25; i+=0.5)	//51
	for (float j = -18; j <= 18; j+=0.5)	//37
	{
	for (float k = -25; k <= 25; k+=0.5) //51
	for (float l = -18; l <= 18; l+=0.5)	// 37

	//for (float m = -25; m <= 25; m+=4.25)	// 12
	//for (float n = -18; n <= 18; n+=4.5)	// 8
	//for (float o = -25; o <= 25; o+=5) // 11
	{
		q[0] = i*PI/30.0f;
		//q[1] = q[0]-PI/4;//j*PI/30.0f;
		q[1] = j*PI/30.0f;
		q[2] = k*PI/30.0f;
		q[3] = l*PI/30.0f;
		q[4] = 0;//m*PI/30.0f;
		q[5] = 0;//n*PI/30.0f;
		q[6] = 0;//o*PI/30.0f;
		
		position[0] = joint_to_cartesian(q);
		position[1] = joint_to_cartesian(q,4);
		position[2] = position[1];
		position[2][0] = (position[0][0]+2*position[1][0])/3;
		position[2][1] = (position[0][1]+2*position[1][1])/3;
		position[2][2] = (position[0][2]+2*position[1][2])/3;
		position[3] = position[2];
		position[3][0] = (2*position[0][0]+position[1][0])/3;
		position[3][1] = (2*position[0][1]+position[1][1])/3;
		position[3][2] = (2*position[0][2]+position[1][2])/3;
		
		
		for(int i1 = 0; i1 < 4; i1++)
			for(int j1 = 0; j1 < 4; j1++){
			if(examineDifference(position[i1], pnt[j1])){
					fprintf((FILE*)(pointfiles[j1][i1]), "%f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \n",q[0], q[1], q[2], q[3], q[4], q[5], q[6],position[i1][0],position[i1][1],position[i1][2]);
					//continue;
				}
			}
		
		/*if(examineDifference(position, pnt0) || examineDifference(position2, pnt0) || examineDifference(position3, pnt0) || examineDifference(position4, pnt0) )
			fprintf(point0, "%f \t %f \t %f \t %f \t %f \t %f \t %f \n",q[0], q[1], q[2], q[3], q[4], q[5], q[6]);
		if(examineDifference(position, pnt1) || examineDifference(position2, pnt1) || examineDifference(position3, pnt1) || examineDifference(position4, pnt1) )                                                   
			fprintf(point1, "%f \t %f \t %f \t %f \t %f \t %f \t %f \n",q[0], q[1], q[2], q[3], q[4], q[5], q[6]);
		if(examineDifference(position, pnt2) || examineDifference(position2, pnt2) || examineDifference(position3, pnt2) || examineDifference(position4, pnt2) )                                                 
			fprintf(point2, "%f \t %f \t %f \t %f \t %f \t %f \t %f \n",q[0], q[1], q[2], q[3], q[4], q[5], q[6]);
		if(examineDifference(position, pnt3) || examineDifference(position2, pnt3) || examineDifference(position3, pnt3) || examineDifference(position4, pnt3) )                                                   
			fprintf(point3, "%f \t %f \t %f \t %f \t %f \t %f \t %f \n",q[0], q[1], q[2], q[3], q[4], q[5], q[6]);*/
		

	
	}	
		count++;
		printf("\33[2K\r");
		now = clock();
		float count1 = 100.00*count/((float)(7548));
		printf("At %f seconds, %f percent complete. Position; %f, %f, %f", ((float)now-(float)start)/1000000, count1, position[0][0], position[0][1], position[0][2]);
		fflush(stdout); 
	}
	
		for(int i1 = 0; i1 < 4; i1++)
			for(int j1 = 0; j1 < 4; j1++){
				fclose(pointfiles[i1][j1]);
	
			}
	/*fclose(point0);
	fclose(point1);
	fclose(point2);
	fclose(point3);*/
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
