#ifndef THREADPOOL_H_
#define THREADPOOL_H
#include <pthread.h>
#include "koa_wqueue.h"
#include <list>
#include "cspaceconverter.h"
#define UNCREATED 0
#define BUSY 1
#define FINISHED 2
using namespace std;
using namespace KDL;

class threadpool;

struct multiplet{
	vector<float> cspaceposition;
	vector<float> rspaceposition;
	multiplet(vector<float> parcspace, vector<float> parrspace){
		cspaceposition = parcspace;
		rspaceposition = parrspace;
	}
	multiplet(int c0, int c1, int c2, int c3, int c4, int c5, int r0, int r1, int r2){
		cspaceposition.resize(6);
		cspaceposition[0] = c0;
		cspaceposition[1] = c1;
		cspaceposition[2] = c2;
		cspaceposition[3] = c3;
		cspaceposition[4] = c4;
		cspaceposition[5] = c5;
		rspaceposition.resize(3);
		rspaceposition[0] = r0;
		rspaceposition[1] = r1;
		rspaceposition[2] = r2;
	}
};

struct queuerow{
	koa_wqueue<multiplet> queue[21];
	koa_wqueue<multiplet>* get_queue(int z){
		if (z >= 0 && z < 21) 
			return &(queue[z]);
		else return NULL;
	}
};


struct queueslice{
	queuerow row[61];
	koa_wqueue<multiplet>* get_queue(int y, int z){
		if (y >= 0 && y < 61)
			return row[y].get_queue(z);
		else return NULL;
	}
};

struct queuepool{
	queueslice slices[61];
	koa_wqueue<multiplet>* get_queue(int x, int y, int z){
		if (x >= 0 && x < 61)
			return slices[x].get_queue(y, z);
		else return NULL;
	}
};
struct queue_saving_params{
	int start_slice, end_slice;
	threadpool * parentThreadpool;
	queue_saving_params(int par_start, int par_end, threadpool * par_thrpl)
	{
		start_slice = par_start;
		end_slice = par_end;
		parentThreadpool = par_thrpl;
	}
	
};
struct coarsening_params{
	int x,y,z,threadnumber;
	threadpool * parentThreadpool;
	coarsening_params(int par_x, int par_y, int par_z, threadpool * par_thrpl, int par_threadnumber)
	{
		x = par_x;
		y = par_y;
		z = par_z;
		parentThreadpool = par_thrpl;
		threadnumber = par_threadnumber;
	}
	
};

struct threadParams{
	threadpool * parentThreadpool;	
	int threadnumber, slicenumber;
	threadParams(threadpool* par_threadpool, int par_slicenumber, int par_threadnumber) {
		parentThreadpool = par_threadpool; 
		threadnumber = par_threadnumber;
		slicenumber = par_slicenumber;
	}
};



class threadpool {

public:
threadpool() {threadpool(51, 6, 16);};
threadpool(int param_totalgenthreads, int param_maxsortthreads, int param_maxconcthreads);
int total_generator_threads, max_concurrent_threads, current_generator_threads, max_concurrent_generator_threads, max_concurrent_sorter_threads, finished_generator_threads;
queuepool qpool;
bool generators_finished;
pthread_t *generators, *sorters, *binners;
int *threadstatus_generators, *threadstatus_sorters, *threadstatus_binners ;
FILE* mainlog;

float x_low, y_low, z_low, x_high, y_high, z_high;

int* get_generator_status_pointer(int i);
int run();
int launch_next_generator_thread(int thread_index, int slicenumber);
int launch_next_sorter_thread(int thread_index, int lowerbound, int upperbound);
int launch_next_binning_thread(int par_i, int par_j, int par_k, int par_thrdnmbr);
};




#endif
