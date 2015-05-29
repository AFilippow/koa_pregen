#include "threadpool.h"

float specialdistance(KDL::JntArray q1, vector<float> q2){
	float result = 0;
	for (int i = 0; i < 6; i++)
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
	max_concurrent_generator_threads = total_generator_threads - max_concurrent_sorter_threads;
	threadstatus_generators = new int[max_concurrent_generator_threads];
	threadstatus_sorters = new int[max_concurrent_sorter_threads];
	for (int i = 0; i < max_concurrent_generator_threads; i++){
		threadstatus_generators[i] = 0;
	}
	for (int i = 0; i < max_concurrent_sorter_threads; i++){

		threadstatus_sorters[i] = 0;
	}
	generators = new pthread_t[max_concurrent_generator_threads];
	sorters = new pthread_t[max_concurrent_sorter_threads];
	finished_generator_threads = 0;
	current_generator_threads = 0;
	generators_finished = false;
}
vector<float> get_multiplet_position(multiplet par_multiplet){
	vector<float> output(3);
	output[0] = par_multiplet.rspaceposition[0];
	output[1] = par_multiplet.rspaceposition[1];
	output[2] = par_multiplet.rspaceposition[2];
	return output;
}

void enqueue_multiplet(multiplet par_multiplet, threadpool* par_thrdpl){
	vector<float> position = get_multiplet_position(par_multiplet);
	int x_index = floor((position[0]+0.3)*61/0.6);
	if (x_index < 0 || x_index >= 62) printf("ERROR: x_index is %i \n", x_index);
	int y_index = floor((position[1]+0.15)*61/0.6);
	if (y_index < 0 || y_index >= 62) printf("ERROR: y_index is %i \n", y_index);
	int z_index = floor((position[2])*21/0.2);
	if (z_index < 0 || z_index >= 22) printf("ERROR: z_index is %i \n", z_index);
	
	koa_wqueue<multiplet> *queue = (par_thrdpl->qpool.get_queue(x_index, y_index, z_index));
	if (!queue){
		printf("ERROR retrieving queue %i, %i, %i \n", x_index, y_index, z_index);
		return;
	}
	while (queue->size() > 1000){
		printf("Queue %i, %i, %i full. Sleeping...\n", x_index, y_index, z_index);
		sleep(10);
	}
	queue->add(par_multiplet);
}



void* save_queues_to_disk(void* par1void){
	queue_saving_params params = *(queue_saving_params*)par1void;
	while (!params.parentThreadpool->generators_finished){
	for (int i = params.start_slice; i < params.end_slice; i++)
		for (int j = 0; j < 61; j++)
			for (int k = 0; k < 21; k++){
				koa_wqueue<multiplet> *queue =  params.parentThreadpool->qpool.get_queue(i,j,k);
				FILE * slicefile = fopen(("/media/Raid/afilippow/cspoutput/slices/slice_"+boost::to_string(i)+"_"+boost::to_string(j)+"_"+boost::to_string(k)+".dat").c_str(),"a");  //TODO fix path here
				while (queue->size() > 0){
					multiplet curr_multiplet = queue->remove();
					fprintf(slicefile,"%f \t %f \t %f \t",curr_multiplet.rspaceposition[0],curr_multiplet.rspaceposition[1],curr_multiplet.rspaceposition[2]);	
					fprintf(slicefile,"%f \t %f \t %f \t %f \t",curr_multiplet.cspaceposition[0],curr_multiplet.cspaceposition[1],curr_multiplet.cspaceposition[2],curr_multiplet.cspaceposition[3]);	
					fprintf(slicefile,"%f \t %f \t %f \t \n",curr_multiplet.cspaceposition[4],curr_multiplet.cspaceposition[5],curr_multiplet.cspaceposition[6]);	
				}
				fclose(slicefile);
			}
	}
}





void* generate_poses(void* par1void){ //NOT part of threadpool
	// I cannot make generate_poses() a member function of cspaceconverter under c++03 and ROS hydro does not support c++2011
	// I need to initialise every part of cspaceconverter again =(
	Chain KukaChain = KukaLWR_DHnew();
	ChainFkSolverPos_recursive* kinematic_solver = new ChainFkSolverPos_recursive(KukaChain);
	int jointNumber = KukaChain.getNrOfJoints();
	KDL::Frame baseframe(KDL::Rotation::Quaternion(-0.444, 0.231, 0.40, 0.768), KDL::Vector(-0.4, 0.15, 0.35) ) ;
	threadParams params = *(threadParams*)par1void;
	KDL::JntArray q(jointNumber);
	
			q(2) = (-25.0f+(float)params.slicenumber*0.5)*PI/30;//k*PI/30.0f;
			q(6) = 0;
			


	
	KDL::Frame cartpos;
	KDL::Vector position;
	KDL::Vector handposition;
	for (float i = -29; i <= 29; i+=1)	//51
	{

		for (float j = -18; j <= 18; j+=2)	//37
		//for (float k = -25; k <= 25; k+=0.5) //51
		for (float l = -18; l <= 18; l+=2)	// 37
		for (float m = -25; m <= 25; m+=2)	// 37
		for (float n = -18; n <= 18; n+=2)	// 37
		{
			q(0) = i*PI/30.0f;
			q(1) = j*PI/30.0f;

			q(3) = l*PI/30.0f;
			q(4) = m*PI/30.0f;
			q(5) = n*PI/30.0f;


			kinematic_solver->JntToCart(q, cartpos);
			position = baseframe*cartpos.p;
			handposition.x(0);
			handposition.y(0);
			handposition.z(0.25);
			handposition = baseframe*cartpos*handposition;
			
			multiplet position_multiplet(toVector(q), toVector(position)); 
			multiplet hand_position_multiplet(toVector(q), toVector(handposition)); 
			
			


			
			
			
			if (!(position.x() < -0.3 || position.y() < -0.15 || position.z() < 0 || position.x() > 0.3 || position.y() > 0.45 || position.z() > 0.2))
				enqueue_multiplet(position_multiplet, params.parentThreadpool);
			
			if (!(handposition.x() < -0.3 || handposition.y() < -0.15 || handposition.z() < 0 || handposition.x() > 0.3 || handposition.y() > 0.45 || handposition.z() > 0.2))
				enqueue_multiplet(hand_position_multiplet, params.parentThreadpool);
			
			
			kinematic_solver->JntToCart(q, cartpos, 4);
			KDL::Vector position2 = baseframe*cartpos.p;
			if (!(position2.x() < -0.3 || position2.y() < -0.15 || position2.z() < 0 || position2.x() > 0.3 || position2.y() > 0.45 || position2.z() > 0.2)){
				multiplet position_multiplet_2(toVector(q), toVector(position2)); 
				enqueue_multiplet(position_multiplet_2, params.parentThreadpool);
			}
			KDL::Vector position3;
			position3.x(position.x()/3.00+position2.x()*2.00/3.00);
			position3.y(position.y()/3.00+position2.y()*2.00/3.00);
			position3.z(position.z()/3.00+position2.z()*2.00/3.00);
			if (!(position3.x() < -0.3 || position3.y() < -0.15 || position3.z() < 0 || position3.x() > 0.3 || position3.y() > 0.45 || position3.z() > 0.2)){
				multiplet position_multiplet_3(toVector(q), toVector(position2)); 
				enqueue_multiplet(position_multiplet_3, params.parentThreadpool);
			}
			position3.x(position.x()*2.00/3.00+position2.x()/3.00);
			position3.y(position.y()*2.00/3.00+position2.y()/3.00);
			position3.z(position.z()*2.00/3.00+position2.z()/3.00);
			if (!(position3.x() < -0.3 || position3.y() < -0.15 || position3.z() < 0 || position3.x() > 0.3 || position3.y() > 0.45 || position3.z() > 0.2)){
				multiplet position_multiplet_4(toVector(q), toVector(position2)); 
				enqueue_multiplet(position_multiplet_4, params.parentThreadpool);
			}
			
			
		}
	printf("slice %i at %f percent \n", params.slicenumber, ((float)i+29)*100.0f/58.0f);
	}
	pthread_exit(NULL);
}



int threadpool::run(){
	printf("Starting up\n");
	void **status;	
	
	for (int i = 0; i < max_concurrent_generator_threads; i++){
		std::cout << launch_next_generator_thread(i, current_generator_threads) << std::endl;
		threadstatus_generators[i] = BUSY;
		current_generator_threads ++;
	}
	int lbound = 0;
	int ubound = 0;
	for (int i = 1; i <= max_concurrent_sorter_threads; i++){
		lbound = ubound;
		ubound = floor(((float)i*61.0F)/((float)max_concurrent_sorter_threads));
		launch_next_sorter_thread(i, lbound, ubound);
		printf("created sorter from %i to %i \n", lbound, ubound);
	}
		
	int current_examined_thread_index = 0;
	while (finished_generator_threads < total_generator_threads){
	///see if generator thread i is finished already	
		if (threadstatus_generators[current_examined_thread_index] == FINISHED){
			pthread_join(generators[current_examined_thread_index], NULL);
			finished_generator_threads++;
			current_generator_threads--;
			if (finished_generator_threads + current_generator_threads < total_generator_threads){
				threadstatus_generators[current_examined_thread_index] = BUSY;
				launch_next_generator_thread(current_examined_thread_index, current_generator_threads+finished_generator_threads);
				current_generator_threads++;
			}
		}
		current_examined_thread_index++;
		if (current_examined_thread_index == max_concurrent_generator_threads)
			current_examined_thread_index = 0;
	}
	generators_finished = true;
}
int threadpool::launch_next_sorter_thread(int thread_index, int lowerbound, int upperbound){
	queue_saving_params* qparams = new queue_saving_params(lowerbound, upperbound, this);
	return pthread_create(&sorters[thread_index], NULL, save_queues_to_disk, (void*)qparams);
}
	
int threadpool::launch_next_generator_thread(int thread_index, int slicenumber){
	threadParams* thrdprm = new threadParams(this, slicenumber, thread_index);
	return pthread_create(&generators[thread_index], NULL, generate_poses, (void*)thrdprm);
}
