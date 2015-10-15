#include "threadpool.h"

float specialdistance(vector<float> q1, vector<float> q2){
	float result = 0;
	for (int i = 0; i < 6; i++)
		result += (q1[i]-q2[i])*(q1[i]-q2[i]);
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
vector< float > toVector(KDL::Vector i){
	vector< float > output(3);
	output[0] = i.x();
	output[1] = i.y();
	output[2] = i.z();
	return output;
}

threadpool::threadpool(int param_totalgenthreads, int param_maxsortthreads, int param_maxconcthreads){
	
	total_generator_threads = param_totalgenthreads;
	max_concurrent_sorter_threads = param_maxsortthreads;
	max_concurrent_generator_threads = param_maxconcthreads - max_concurrent_sorter_threads;
	threadstatus_generators = new int[max_concurrent_generator_threads];
	threadstatus_sorters = new int[max_concurrent_sorter_threads];
	threadstatus_binners = new int[param_maxconcthreads];
	for (int i = 0; i < max_concurrent_generator_threads; i++){
		threadstatus_generators[i] = 0;
	}
	for (int i = 0; i < max_concurrent_sorter_threads; i++){

		threadstatus_sorters[i] = 0;
	}
	for (int i = 0; i < param_maxconcthreads; i++){

		threadstatus_binners[i] = 0;
	}
	generators = new pthread_t[max_concurrent_generator_threads];
	sorters = new pthread_t[max_concurrent_sorter_threads];
	binners = new pthread_t[param_maxconcthreads];
	finished_generator_threads = 0;
	current_generator_threads = 0;
	generators_finished = false;
	x_low = -0.55; 
	y_low = -0.05;
	z_low = 0;
	x_high = 0.35;
	y_high = 0.85;
	z_high = 0.6;
	xycoarseness = 61;
	zcoarseness = 41;
	
	
}
vector<float> get_multiplet_position(multiplet par_multiplet){
	vector<float> output(3);
	output[0] = par_multiplet.rspaceposition[0];
	output[1] = par_multiplet.rspaceposition[1];
	output[2] = par_multiplet.rspaceposition[2];
	return output;
}

void enqueue_multiplet(multiplet par_multiplet, threadpool* par_thrdpl, int par_debug = 0){
	if (par_multiplet.rspaceposition[0] == 0.0 || par_multiplet.rspaceposition[1] == 0.0 || par_multiplet.rspaceposition[2] == 0.0){
	 printf("Warning: passed %f, %f, %f to queue from part %i\n", par_multiplet.rspaceposition[0], par_multiplet.rspaceposition[1], par_multiplet.rspaceposition[2], par_debug); 
	}
  
	vector<float> position = get_multiplet_position(par_multiplet);
	int x_index = floor((position[0]-par_thrdpl->x_low)*par_thrdpl->xycoarseness/(par_thrdpl->x_high - par_thrdpl->x_low));
	if (x_index < 0 || x_index > par_thrdpl->xycoarseness) printf("ERROR: x_index is %i \n", x_index);
	int y_index = floor((position[1]-par_thrdpl->y_low)*par_thrdpl->xycoarseness/(par_thrdpl->y_high - par_thrdpl->y_low));
	if (y_index < 0 || y_index > par_thrdpl->xycoarseness) printf("ERROR: y_index is %i \n", y_index);
	int z_index = floor((position[2]-par_thrdpl->z_low)*par_thrdpl->zcoarseness/(par_thrdpl->z_high - par_thrdpl->z_low));
	if (z_index < 0 || z_index > par_thrdpl->zcoarseness) printf("ERROR: z_index is %i \n", z_index);
	
	koa_wqueue<multiplet> *queue = (par_thrdpl->qpool.get_queue(x_index, y_index, z_index));
	if (!queue){
		fprintf(par_thrdpl->mainlog, "ERROR retrieving queue %i, %i, %i, aborted!\n", x_index, y_index, z_index);
		return;
	}
	while (queue->size() > 1000){
		fprintf(par_thrdpl->mainlog, "Queue %i, %i, %i full. Sleeping...\n", x_index, y_index, z_index);
		sleep(10);
	}
	queue->add(par_multiplet);
}
vector<float> bin(vector<float> input){
	vector<float> output = input;
	for (int i = 0; i < output.size(); i++)
		output[i] = 0.2*floor(output[i]*5);
	return output;
}



void* coarsenData(void* par_void){
	coarsening_params params = *(coarsening_params*) par_void;
	float rx, ry, rz;
	FILE * yzslice;
	std::string line;
	vector<float> point(6);
	time_t start = time(0);
	float should_be_zero = 0;
	vector< vector< float > >obstacles(0); ///continue here //TODO need a better coarsening algorithm, use one kernel per 6-cube of side length 0.2 maybe?
	vector< float > counts(0);
	std::ifstream  slicefile(("/media/Raid/afilippow/cspoutput/6d/slice_"+boost::to_string(params.x)+"_"+boost::to_string(params.y)+"_"+boost::to_string(params.z)+".dat").c_str(), ios::in);		
	while (std::getline(slicefile, line)){
		std::istringstream iss(line);
		if (!(iss >> rx >> ry >> rz >> point[0] >> point[1] >> point[2] >> point[3] >> point[4] >> point[5] >> should_be_zero)) { printf("error: cannot read line!:"); printf(line.c_str());printf("\n");continue; }
		int count;
		bool found = false;
		for (int i = 0; i < obstacles.size(); i++){
			if (specialdistance(bin(point), bin(obstacles[i])) < 0.01 && !found)
				{
					count = counts[i];
					for (int j = 0; j < 6; j++)
						obstacles[i][j] = (obstacles[i][j]+point[j]/((float)count))*((float)count)/((float)count+1);
					counts[i]++;
					found = true;
					break;
				}
		}
		if(!found){
			int size = obstacles.size();
			obstacles.resize(size +1);
			obstacles[size].resize(6);
			counts.resize(size+1);
			for (int j = 0; j < 6; j++)
				obstacles[size][j] = point[j];
			counts[size] = 1;
		}	
			
	}
	yzslice = fopen(("/media/Raid/afilippow/cspoutput/6dreduced/slice_"+boost::to_string(params.x)+"_"+boost::to_string(params.y)+"_"+boost::to_string(params.z)+".dat").c_str(),"w");
	for (int i1 = 0; i1 < obstacles.size(); i1++)
		fprintf(yzslice,"%f \t %f \t %f \t %f \t %f \t %f \t %i \n", obstacles[i1][0], obstacles[i1][1], obstacles[i1][2], obstacles[i1][3], obstacles[i1][4], obstacles[i1][5], counts[i1]);
	fclose(yzslice);
	printf("slice %i, %i, %i done after %f seconds \n", params.x, params.y, params.z, difftime( time(0), start));
	params.parentThreadpool->threadstatus_binners[params.threadnumber] = FINISHED;
}
void* save_queues_to_disk(void* par1void){
	queue_saving_params params = *(queue_saving_params*)par1void;
	while (!params.parentThreadpool->generators_finished){
	for (int i = params.start_slice; i < params.end_slice; i++)
		for (int j = 0; j < params.parentThreadpool->xycoarseness; j++)
			for (int k = 0; k < params.parentThreadpool->zcoarseness; k++){
				koa_wqueue<multiplet> *queue =  params.parentThreadpool->qpool.get_queue(i,j,k);
				FILE * slicefile = fopen(("/media/Raid/afilippow/cspoutput/6d/slice_"+boost::to_string(i)+"_"+boost::to_string(j)+"_"+boost::to_string(k)+".dat").c_str(),"a");  //TODO fix path here
				while (queue->size() > 0){
					multiplet curr_multiplet = queue->remove();
					fprintf(slicefile,"%f \t %f \t %f \t",curr_multiplet.rspaceposition[0],curr_multiplet.rspaceposition[1],curr_multiplet.rspaceposition[2]);	
					fprintf(slicefile,"%f \t %f \t %f \t %f \t",curr_multiplet.cspaceposition[0],curr_multiplet.cspaceposition[1],curr_multiplet.cspaceposition[2],curr_multiplet.cspaceposition[3]);	
					fprintf(slicefile,"%f \t %f \t %f \t \n",curr_multiplet.cspaceposition[4],curr_multiplet.cspaceposition[5],curr_multiplet.cspaceposition[6]);	
				}
				fclose(slicefile);
			}
	}
	for (int i = params.start_slice; i < params.end_slice; i++)
		for (int j = 0; j < params.parentThreadpool->xycoarseness; j++)
			for (int k = 0; k < params.parentThreadpool->zcoarseness; k++){
				koa_wqueue<multiplet> *queue =  params.parentThreadpool->qpool.get_queue(i,j,k);
				FILE * slicefile = fopen(("/media/Raid/afilippow/cspoutput/6d/slice_"+boost::to_string(i)+"_"+boost::to_string(j)+"_"+boost::to_string(k)+".dat").c_str(),"a");  //TODO fix path here
				while (queue->size() > 0){
					multiplet curr_multiplet = queue->remove();
					fprintf(slicefile,"%f \t %f \t %f \t",curr_multiplet.rspaceposition[0],curr_multiplet.rspaceposition[1],curr_multiplet.rspaceposition[2]);	
					fprintf(slicefile,"%f \t %f \t %f \t %f \t",curr_multiplet.cspaceposition[0],curr_multiplet.cspaceposition[1],curr_multiplet.cspaceposition[2],curr_multiplet.cspaceposition[3]);	
					fprintf(slicefile,"%f \t %f \t %f \t \n",curr_multiplet.cspaceposition[4],curr_multiplet.cspaceposition[5],curr_multiplet.cspaceposition[6]);	
				}
				fclose(slicefile);
			}
}



bool within_range(KDL::Vector par_vector, threadpool* par_thrdpl){
	if (par_vector.x() < par_thrdpl->x_low || par_vector.y() < par_thrdpl->y_low || par_vector.z() < par_thrdpl->z_low || par_vector.x() > par_thrdpl->x_high || par_vector.y() > par_thrdpl->y_high || par_vector.z() > par_thrdpl->z_high)
		return false;
	else 
		return true;
	
	
}

void* generate_poses(void* par1void){ //NOT part of threadpool
	// I cannot make generate_poses() a member function of cspaceconverter under c++03 and ROS hydro does not support c++2011
	// I need to initialise every part of cspaceconverter again =(
	Chain KukaChain = KukaLWR_DHnew();
	ChainFkSolverPos_recursive* kinematic_solver = new ChainFkSolverPos_recursive(KukaChain);
	int jointNumber = KukaChain.getNrOfJoints();
	KDL::Frame baseframe(KDL::Rotation::Quaternion(-0.446, -0.12, 0.857, 0.230), KDL::Vector(-0.05, 0.0, 0.33) ) ;
	threadParams params = *(threadParams*)par1void;
	KDL::JntArray q(jointNumber);
	
			//q(2) = (-25.0f+(float)params.slicenumber/2)*PI/30.0f;//k*PI/30.0f;
			q(2) = -0.349+(float)params.slicenumber*(0.349+1.7453)/100;//k*PI/30.0f;
			q(6) = 0;
			


	
	KDL::Frame cartpos;
	KDL::Vector position;
	KDL::Vector extraposition;
	KDL::Vector handposition;
	fprintf(params.parentThreadpool->mainlog, "slice %i started!\n", params.slicenumber);

	//for (float i = -29; i <= 29; i+=0.1)	//51 +1
	for (float i = 0; i <= 28; i+=1)  //51 +1
	{

		for (float j = 0; j <= 20; j+=1)	//37 +1
		//for (float k = -25; k <= 25; k+=0.2) //51
		for (float l = 0; l <= 28; l+=1)	// 37 +1
		for (float m = 0; m <= 20; m+=1)	// 37 +1
		for (float n = 0; n <= 20; n+=1)	// 37 +1
		{
			//float m = 0;
			//float n = 0;
			q(0) = -1.7453+i*(1.7453-0.349)/28.0;
			q(1) = -2.0944+j*1.7543/20.0;

			q(3) = -0.6981+l*2.7925/28.0;
			q(4) = -PI+m/10.0f*PI;
			q(5) = -PI+n/10.0f*PI;


			kinematic_solver->JntToCart(q, cartpos);
			position = baseframe*cartpos.p;
			handposition.x(0);
			handposition.y(0);
			handposition.z(0.30);
			handposition = baseframe*cartpos*handposition;
			
			multiplet position_multiplet(toVector(q), toVector(position)); 
			multiplet hand_position_multiplet(toVector(q), toVector(handposition)); 
			
			
			//if (within_range(position, params.parentThreadpool))
			//	enqueue_multiplet(position_multiplet, params.parentThreadpool, 1);
			
			//if (within_range(handposition, params.parentThreadpool))
			//	enqueue_multiplet(hand_position_multiplet, params.parentThreadpool, 2);

			/*for (int i = 1; i < 4; i++){
			  float angle = 2*3.14152*((float)i)/6.0;
			  int odd = !!(i % 2);  // using !! to ensure 0 or 1 value.

			  float ifloat = (float)i;
			  extraposition.x(position.x()*ifloat/4.0+handposition.x()*(4.0-ifloat)/4.0);
			  extraposition.y(position.y()*ifloat/4.0+handposition.y()*(4.0-ifloat)/4.0);
			  extraposition.z(position.z()*ifloat/4.0+handposition.z()*(4.0-ifloat)/4.0);
			  if (within_range(extraposition, params.parentThreadpool)){
			    multiplet position_multiplet_extra(toVector(q), toVector(extraposition)); 
				enqueue_multiplet(position_multiplet_extra, params.parentThreadpool, 10+i);
			  }
			}*/


			KDL::Vector xtrapos;
			for (float i = 0; i < 5; i++){

			  float angle = 2*3.14152*((float)i)/11.1;
			  //int odd = !!(i % 2);  // using !! to ensure 0 or 1 value.
			  xtrapos.x(0);
			  xtrapos.y(0);
			  xtrapos.z(i/4*0.15);
			  float ifloat = (float)i;
			  extraposition = baseframe*cartpos*xtrapos;
			  if (within_range(extraposition, params.parentThreadpool)){
			    multiplet position_multiplet_extra(toVector(q), toVector(extraposition)); 
				enqueue_multiplet(position_multiplet_extra, params.parentThreadpool, 10+i);
			  }
			}			
			/*

			
			
			kinematic_solver->JntToCart(q, cartpos, 4);
			KDL::Vector position2 = baseframe*cartpos.p;
			if (within_range(position2, params.parentThreadpool)){
				multiplet position_multiplet_2(toVector(q), toVector(position2)); 
				enqueue_multiplet(position_multiplet_2, params.parentThreadpool, 3);
			}
			KDL::Vector position3;
			position3.x(position.x()/3.00+position2.x()*2.00/3.00);
			position3.y(position.y()/3.00+position2.y()*2.00/3.00);
			position3.z(position.z()/3.00+position2.z()*2.00/3.00);
			if (within_range(position3, params.parentThreadpool)){
				multiplet position_multiplet_3(toVector(q), toVector(position3)); 
				enqueue_multiplet(position_multiplet_3, params.parentThreadpool, 4);
			}
			KDL::Vector position4;
			position4.x(position.x()*2.00/3.00+position2.x()/3.00);
			position4.y(position.y()*2.00/3.00+position2.y()/3.00);
			position4.z(position.z()*2.00/3.00+position2.z()/3.00);
			if (within_range(position4, params.parentThreadpool)){
				multiplet position_multiplet_4(toVector(q), toVector(position4)); 
				enqueue_multiplet(position_multiplet_4, params.parentThreadpool, 5);
			}*/
			
			
		}
	fprintf(params.parentThreadpool->mainlog,"slice %i at %f percent \n", params.slicenumber, ((float)i+29)*100.0f/58.0f);
	}
	fprintf(params.parentThreadpool->mainlog, "slice %i finished succesfully!\n", params.slicenumber);
	params.parentThreadpool->threadstatus_generators[params.threadnumber] = FINISHED;
	pthread_exit(NULL);
}



int threadpool::run(){
	printf("Starting up\n");
	void **status;	
	mainlog = fopen("/home/afilippow/workspace/pregen.log", "w");
		int current_examined_thread_index = 0;

		
	for (int i = 0; i < max_concurrent_generator_threads; i++){
		std::cout << launch_next_generator_thread(i, current_generator_threads) << std::endl;
		threadstatus_generators[i] = BUSY;
		current_generator_threads ++;
	}
	int lbound = 0;
	int ubound = 0;
	for (int i = 0; i < max_concurrent_sorter_threads; i++){
		lbound = ubound;
		ubound = floor(((float)(i+1)*((float)xycoarseness))/((float)max_concurrent_sorter_threads));
		launch_next_sorter_thread(i, lbound, ubound);
		printf("created sorter from %i to %i \n", lbound, ubound);
	}

	while (finished_generator_threads < total_generator_threads){
	///see if generator thread i is finished already	
		if (threadstatus_generators[current_examined_thread_index] == FINISHED){
			pthread_join(generators[current_examined_thread_index], NULL);
			finished_generator_threads++;
			current_generator_threads--;
			threadstatus_generators[current_examined_thread_index] = UNCREATED;
			fprintf(mainlog, "%i generator threads finished successfully of %i total. %i running\n",finished_generator_threads, total_generator_threads, current_generator_threads);
			if (finished_generator_threads + current_generator_threads < total_generator_threads){
				threadstatus_generators[current_examined_thread_index] = BUSY;
				launch_next_generator_thread(current_examined_thread_index, current_generator_threads+finished_generator_threads);
				current_generator_threads++;
				fprintf(mainlog, "Generator thread launched\n");
			}
		}
		current_examined_thread_index++;
		if (current_examined_thread_index == max_concurrent_generator_threads)
 			current_examined_thread_index = 0;
	}
	generators_finished = true;
	
	for(int i = 0; i <max_concurrent_sorter_threads; i++)
	   pthread_join(sorters[i], NULL);
	      
	sleep(10);
	int finished_binning_threads = 0;
	int running_binning_threads = 0;
	int total_binning_threads = xycoarseness*xycoarseness*zcoarseness;
	current_examined_thread_index = 0;
	int i = 0;
	int j = 0;
	int k = 0;
	while (finished_binning_threads < total_binning_threads){
		if (threadstatus_binners[current_examined_thread_index] == FINISHED){
			pthread_join(binners[current_examined_thread_index], NULL);
			finished_binning_threads++;
			running_binning_threads--;
			threadstatus_binners[current_examined_thread_index] = UNCREATED;
		}
		if ((finished_binning_threads + running_binning_threads < total_binning_threads) && (threadstatus_binners[current_examined_thread_index] == UNCREATED)){
			running_binning_threads++;
			threadstatus_binners[current_examined_thread_index] = BUSY;
			launch_next_binning_thread(i, j, k, current_examined_thread_index);
			fprintf(mainlog, "Binning thread %i, %i, %i launched\n", i, j, k);
			i++;
			if (i == xycoarseness){
				i = 0;
				j++;
				if ( j == xycoarseness){
					j = 0;
					k++;
				}
			}

		}
		current_examined_thread_index++;
		if (current_examined_thread_index == max_concurrent_generator_threads+max_concurrent_sorter_threads)
 			current_examined_thread_index = 0;
	}
	
	
	///TODO continue here
	
	fclose(mainlog);
}
int threadpool::launch_next_sorter_thread(int thread_index, int lowerbound, int upperbound){
	queue_saving_params* qparams = new queue_saving_params(lowerbound, upperbound, this);
	return pthread_create(&sorters[thread_index], NULL, save_queues_to_disk, (void*)qparams);
}
	
int threadpool::launch_next_generator_thread(int thread_index, int slicenumber){
	threadParams* thrdprm = new threadParams(this, slicenumber, thread_index);
	return pthread_create(&generators[thread_index], NULL, generate_poses, (void*)thrdprm);
}
int threadpool::launch_next_binning_thread(int par_i, int par_j, int par_k, int par_thrdnmbr){
	coarsening_params* crsprms = new coarsening_params(par_i, par_j, par_k, this, par_thrdnmbr);
	return pthread_create(&binners[par_thrdnmbr], NULL, coarsenData, (void*)crsprms);
}
