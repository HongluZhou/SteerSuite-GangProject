#define BLOCK_SIZE 256
#define MAX_DEPTH 5
#include <math.h>
#include <iostream>
#include "Structs.h"
#include "cuda_runtime.h"
#include <device_launch_parameters.h>
#include "GraphSearch.h"
#include <time.h>
#include <vector>

using namespace std;

bool selectBestGPU()
{
	int numDevices;
	cudaError_t cudaResultCode = cudaGetDeviceCount(&numDevices);
	if (cudaResultCode != cudaSuccess){
		return false;
	}

	int maxMultiProc = 0;
	for (int i = 0; i < numDevices; i++) {
		cudaDeviceProp prop;
		cudaGetDeviceProperties(&prop, i);
		if (prop.multiProcessorCount > maxMultiProc) {
			maxMultiProc = prop.multiProcessorCount;
			cudaSetDevice(i);
		}
	}

	return true;
}

__device__ unsigned return_thread_index()
{
	unsigned thread_num_in_block = threadIdx.x + (threadIdx.y * blockDim.x);
	unsigned block_num_in_grid = blockIdx.x + (blockIdx.y * gridDim.x);
	return (block_num_in_grid * blockDim.x * blockDim.y) + thread_num_in_block;
}

__device__ float solve_line_z(float xTarget, float x1, float z1, float tangent)
{
	return tangent*(xTarget - x1) + z1;
}

__device__ float solve_line_x(float zTarget, float x1, float z1, float tangent)
{
	return (zTarget - z1) / tangent + x1;
}


__device__ void rotate_axis(float& x1r, float& z1r, float tilt, float x1, float z1)
{
	float sinVal, cosVal;
	sinVal = sinf(tilt);
	cosVal = cosf(tilt);
	z1r = cosVal*z1 - sinVal*x1;
	x1r = sinVal*z1 + cosVal*x1;
}

__device__ int ray_cast(Point_Struct p1, Point_Struct p2, Obstacle_Struct* obstacles, int numObstacles)
{
	for (int i = 0; i < numObstacles; i++)
	{
		Obstacle_Struct obstacle = obstacles[i];
		float x1, z1, x2, z2, target;
		if (obstacle.type == SHADOW_OBSTACLE)
			continue;

		x1 = obstacle.pMin.x;
		z1 = obstacle.pMin.z;
		x2 = obstacle.pMax.x;
		z2 = obstacle.pMax.z;

		// Do axis rotation to get axis aligned obstacle
		float tilt, x1r, z1r, x2r, z2r, px1r, pz1r, px2r, pz2r, tangent;
		tilt = obstacle.tiltDegree;
		rotate_axis(x1r, z1r, tilt, x1, z1);
		rotate_axis(x2r, z2r, tilt, x2, z2);
		rotate_axis(px1r, pz1r, tilt, p1.x, p1.z);
		rotate_axis(px2r, pz2r, tilt, p2.x, p2.z);
		tangent = (pz2r - pz1r) / (px2r - px1r);

		float xpMin = (px1r < px2r) ? px1r : px2r;
		float xpMax = (px1r > px2r) ? px1r : px2r;
		float zpMin = (pz1r < pz2r) ? pz1r : pz2r;
		float zpMax = (pz1r > pz2r) ? pz1r : pz2r;

		if (x1r > x2r)
		{
			float tmp = x1r;
			x1r = x2r;
			x2r = tmp;
		}
		if (z1r > z2r)
		{
			float tmp = z1r;
			z1r = z2r;
			z2r = tmp;
		}

		// check if the obstacle is within points
		if (xpMin > x2r || xpMax < x1r || zpMin > z2r || zpMax < z1r)
			continue;

		// check if any of the points are inside the obstacle
		if (px1r >= x1r && px1r <= x2r && pz1r >= z1r && pz1r <= z2r)
			return obstacle.type;
		if (px2r >= x1r && px2r <= x2r && pz2r >= z1r && pz2r <= z2r)
			return obstacle.type;

		if (x1r >= xpMin && x1r <= xpMax)
		{
			target = solve_line_z(x1r, px1r, pz1r, tangent);
			if (target >= z1r && target <= z2r)
				if (target >= zpMin && target <= zpMax)
					return obstacle.type;
		}

		if (x2r >= xpMin && x2r <= xpMax)
		{
			target = solve_line_z(x2r, px1r, pz1r, tangent);
			if (target >= z1r && target <= z2r)
				if (target >= zpMin && target <= zpMax)
					return obstacle.type;
		}
		if (z1r >= zpMin && z1r <= zpMax)
		{
			target = solve_line_x(z1r, px1r, pz1r, tangent);
			if (target >= x1r && target <= x2r)
				if (target >= xpMin && target <= xpMax)
					return obstacle.type;
		}

	}
	return SHADOW_OBSTACLE;
}

__device__ bool hasLineOfSight(Point_Struct point1, Point_Struct point2, Obstacle_Struct* obstacles, int numObstacles)
{
	if (ray_cast(point1, point2, obstacles, numObstacles))
		return false;
	else
		return true;
}


__global__ void cuda_generate_graph(Point_Struct* grid, int* adjMatrix_ptr, int totalNodes, Obstacle_Struct* obstacles, int numObstacles)
{
	int threadIndex = return_thread_index();
	Point_Struct pt1 = grid[threadIndex];
	if (threadIndex < totalNodes)
	{
		for (int i = 0; i < totalNodes; i++)
		{
			Point_Struct pt2 = grid[i];
			if (hasLineOfSight(pt1, pt2, obstacles, numObstacles))
			{
				int index = totalNodes * threadIndex + i;
				adjMatrix_ptr[index] = 1;
			}
		}
	}
}

__global__ void cuda_generate_graph_mv1(Point_Struct* grid, char* adjMatrix_ptr, int totalNodes, Obstacle_Struct* obstacles, int numObstacles, char obsType)
{
	unsigned threadIndex = return_thread_index();
	unsigned threadIndexY = threadIndex / totalNodes;
	unsigned threadIndexX = threadIndex % totalNodes;
	Point_Struct pt1 = grid[threadIndexY];
	Point_Struct pt2 = grid[threadIndexX];
	if (threadIndexY <= threadIndexX)
	{
		unsigned index = totalNodes * threadIndexY + threadIndexX;
		unsigned index2 = totalNodes * threadIndexX + threadIndexY;
		if (adjMatrix_ptr[index] == STATIC_OBSTACLE) // static obstacle has blocked the path
			return;
		if (hasLineOfSight(pt1, pt2, obstacles, numObstacles))
		{
			// if there is no block, set matrix element to 0 (connected)
			adjMatrix_ptr[index] = 0; //use symmetry
			adjMatrix_ptr[index2] = 0;
		}
		else
		{
			// if there is an obstacle blocking the line of sight, set matrix element to obsType (1-2)
			adjMatrix_ptr[index] = obsType; //use symmetry
			adjMatrix_ptr[index2] = obsType;
		}
	}
}

__global__ void cuda_sequence_list(bool* frontMat, int* seqFront, int totalRef, int totalQuery)
{
	unsigned tid, count, ntree;
	count = 0;
	tid = return_thread_index();
	if (tid >= totalQuery)
		return;
	//ntree = tid % totalNodes;
	ntree = tid;
	for (unsigned s = 0; s < totalRef; s++)
	{
		if (frontMat[s + ntree * totalRef])
		{
			seqFront[count + ntree * totalRef] = s;
			count++;
		}
	}
}

__global__ void cuda_expand_forest(bool* parentMat, bool* childMat, int* seqFront, int* depthMat, Point_Struct* nodeList, char* adjMatrix, int depth, int totalNodes, int totalRef, int totalQuery)
{
	int tid, ntree, idX, idY;
	ntree = blockIdx.y;
	if (ntree >= totalQuery)
		return;
	//if (!nodeList[ntree].isQ)
	//	return;

	idX = threadIdx.x + (threadIdx.y * blockDim.x) + (blockIdx.x * BLOCK_SIZE);
	tid = threadIdx.x + (threadIdx.y * blockDim.x);
	if (idX >= totalRef)
		return;

	//extern __shared__ int seqf[];
	__shared__ int frontDepth;
	if (tid == 0)
	{
		if (depth == 0)
			frontDepth = 1;
		else
			frontDepth = depthMat[depth - 1 + ntree * MAX_DEPTH];
	}
	/*
	for (unsigned s = 0; s < totalNodes; s += BLOCK_SIZE)
	{
	if (tid + s < totalNodes)
	{
	seqf[tid + s] = seqFront[tid + s + ntree * totalNodes];
	}
	}*/
	__syncthreads();

	//if (!nodeList[idX].isRef)
	//	return;
	for (unsigned s = 0; s < frontDepth; s++)
	{
		if (parentMat[idX + ntree * totalRef])
			return;
		//idY = seqf[s];
		idY = seqFront[s + ntree * totalRef];
		//if (idY < 0)
		//	return;
		if (!adjMatrix[idX + idY * totalNodes]) // 0 is connected
		{
			childMat[idX + ntree * totalRef] = true;
			parentMat[idX + ntree * totalRef] = true;
		}
	}
}

__global__ void cuda_init_forest(bool* parentMat, bool* childMat, bool* frontMat, int* seqFront, int* depthMat, int totalRef, int totalQuery, int refOnly)
{
	int tid;
	tid = return_thread_index();
	if (tid >= totalQuery)
		return;
	seqFront[totalRef * tid] = tid + refOnly;
	/*
	unsigned tid, idX, idY;
	tid = return_thread_index();
	idX = tid % totalNodes;
	idY = tid / totalNodes;

	if (idY >= totalNodes)
		return;

	//childMat[idX + idY * totalNodes] = false;
	//seqFront[idX + idY * totalNodes] = -1;
	if (idX == idY)
	{
		frontMat[idX + idY * totalNodes] = true;
		//parentMat[idX + idY * totalNodes] = true;
	}
	else
	{
		frontMat[idX + idY * totalNodes] = false;
		//parentMat[idX + idY * totalNodes] = false;
	}

	//if (idX < MAX_DEPTH)
	//{
	//	depthMat[idX + idY * MAX_DEPTH] = 0;
	//}*/
}

__global__ void cuda_update_forest(bool* parentMat, bool* childMat, bool* frontMat, int* seqFront, int* depthMat, int depth, int totalRef, int totalQuery)
{
	unsigned tid, idX, idY;
	tid = return_thread_index();
	idX = tid % totalRef;
	idY = tid / totalRef;
	extern __shared__ int sumVal[];

	if (idY >= totalQuery)
		return;

	// Init shared memory (only where needed)
	if (sumVal[idY] != 0)
		sumVal[idY] = 0;
	__syncthreads();


	if (childMat[idX + idY * totalRef])
		atomicAdd(&sumVal[idY], 1);
	__syncthreads();

	if ((threadIdx.x == 0 || idX == 0) && sumVal[idY] > 0)
		atomicAdd(&depthMat[depth + idY * MAX_DEPTH], sumVal[idY]);

	//if (childMat[idX + idY * totalNodes])
	//	atomicAdd(&depthMat[depth + idY * MAX_DEPTH], 1);
	frontMat[idX + idY * totalRef] = childMat[idX + idY * totalRef];
	childMat[idX + idY * totalRef] = false;
	//seqFront[idX + idY * totalNodes] = -1;
}

__global__ void cuda_extract_forest(int* dresults, float* eresults, int* depthMat, Point_Struct* nodeList, int totalNodes)
{
	unsigned depth, ntree;
	depth = threadIdx.x;
	ntree = blockIdx.x;
	__shared__ int dval, total;
	__shared__ float eval;
	if (depth == 0)
	{
		dval = 0;
		eval = 0;
		total = 0;
	}
	__syncthreads();

	if (depthMat[depth + ntree*MAX_DEPTH] > 0)// && nodeList[ntree].isQ)
	{
		atomicAdd(&total, depthMat[depth + ntree*MAX_DEPTH]);
		atomicAdd(&dval, 1);
	}
	__syncthreads();

	if (depthMat[depth + ntree*MAX_DEPTH] > 0)// && nodeList[ntree].isQ)
	{
		float p;
		p = (float)depthMat[depth + ntree*MAX_DEPTH] / (float)total;
		atomicAdd(&eval, -p*log2(p));
	}
	__syncthreads();

	if (depth == 0)
		dresults[ntree] = dval;
	else if (depth == 1)
		eresults[ntree] = eval;

}

__global__ void cuda_calc_degree(int* kresults, char* adjMat, Point_Struct* nodeList, int totalNodes, int totalRef, int totalQuery, int refOnly)
{
	unsigned tid, idX, idY;
	tid = return_thread_index();
	idX = tid % totalRef;
	idY = tid / totalRef;
	extern __shared__ int sumVal1[];

	if (idY >= totalQuery)
		return;
	if (idY == 0)
	{
		kresults[idX] = 0;
	}

	// Init shared memory (only where needed)
	if (sumVal1[idY] != 0)
		sumVal1[idY] = 0;
	__syncthreads();

	if (!adjMat[idX + (idY+refOnly) * totalNodes])// && nodeList[idX].isRef) // 0 is connected
		atomicAdd(&sumVal1[idY], 1);
	__syncthreads();

	if (sumVal1[idY] > 0 && ((threadIdx.x + threadIdx.y) == 0 || idX == 0))
		atomicAdd(&kresults[idY], sumVal1[idY]);
}

__global__ void reduced_add_int(int* g_idata, int* g_odata, int totalNodes)
{
	extern __shared__ int sdata[];

	// Read chuncks into shared memory of each block
	unsigned tid = threadIdx.x + threadIdx.y * blockDim.x;
	unsigned i = return_thread_index();
	if (i < totalNodes)
		sdata[tid] = g_idata[i];
	else
		sdata[tid] = 0;
	__syncthreads();

	// Do reduction
	for (unsigned s = blockDim.x * blockDim.y / 2; s > 0; s >>= 1)
	{
		if (tid < s)
		{
			sdata[tid] += sdata[tid + s];
		}
		__syncthreads();
	}
	// Write resutls
	if (tid == 0)
		atomicAdd(g_odata, sdata[0]);
}

__global__ void reduced_add_float(float* g_idata, float* g_odata, int totalNodes)
{
	extern __shared__ float sdata1[];

	// Read chuncks into shared memory of each block
	unsigned tid = threadIdx.x + threadIdx.y * blockDim.x;
	unsigned i = return_thread_index();
	if (i < totalNodes)
		sdata1[tid] = g_idata[i];
	else
		sdata1[tid] = 0;
	__syncthreads();

	// Do reduction
	for (unsigned s = blockDim.x * blockDim.y / 2; s > 0; s >>= 1)
	{
		if (tid < s)
		{
			sdata1[tid] += sdata1[tid + s];
		}
		__syncthreads();
	}

	// Write resutls
	if (tid == 0)
		atomicAdd(g_odata, sdata1[0]);
}

/**************GPU VISIBILITY GRAPH********************/

gpuVG::~gpuVG()
{
	clear();
}

void gpuVG::clear()
{
	//Free memory
	cudaFree(_adjMatrix);
	cudaFree(_nodeList);
	cudaFree(_obsList);
	cudaFree(_depthMat);
	cudaFree(_parentMat);
	cudaFree(_childMat);
	cudaFree(_frontMat);
	cudaFree(_kresults);
	cudaFree(_dresults);
	cudaFree(_eresults);
	cudaFree(_output);
	_isInit = false;
}

void gpuVG::init(vector<Point_Struct> nodeList, unsigned obsCount, unsigned refCount, unsigned queryCount, unsigned refOnlyCount)
{
	if (_isInit)
		return;

	int totalNodes = nodeList.size();
	_totalNodes = totalNodes;
	_totalQuery = queryCount;
	_totalRef = refCount;
	_refOnly = refOnlyCount;

	// Visibility Grid
	cudaMalloc((void**)&_nodeList, totalNodes*sizeof(Point_Struct));
	cudaMemcpy(_nodeList, &nodeList[0], totalNodes*sizeof(Point_Struct), cudaMemcpyHostToDevice);

	//Initialize adjacency matrix to 0
	cudaMalloc((void**)&_adjMatrix, totalNodes*totalNodes*sizeof(char));
	cudaMalloc((void**)&_parentMat, _totalQuery*_totalRef*sizeof(bool));
	cudaMalloc((void**)&_childMat, _totalQuery*_totalRef*sizeof(bool));
	cudaMalloc((void**)&_frontMat, _totalQuery*_totalRef*sizeof(bool));
	cudaMalloc((void**)&_seqFront, _totalQuery*_totalRef*sizeof(int));
	cudaMemset(_adjMatrix, 0, totalNodes*totalNodes*sizeof(bool));

	// Initialize depth container
	cudaMalloc((void**)&_depthMat, _totalQuery*MAX_DEPTH*sizeof(int));

	// Init results
	cudaMalloc((void**)&_kresults, _totalQuery*sizeof(int));
	cudaMalloc((void**)&_dresults, _totalQuery*sizeof(int));
	cudaMalloc((void**)&_eresults, _totalQuery*sizeof(float));
	cudaMalloc((void**)&_output, 3 * sizeof(int));

	// MEMORY LEAK DANGER IN CASE OBSTABLE NUMBER IS NOT CONSTANT
	cudaMalloc((void**)&_obsList, obsCount*sizeof(Obstacle_Struct));

	_isInit = true;
}

void gpuVG::generate_graph(vector<Obstacle_Struct> obstacles, char obsType)
{
	// Setup block and grid size
	int blockLength = sqrt((double)BLOCK_SIZE);
	int gridLength = ceil(sqrt((double)_totalNodes*_totalNodes / ((double)BLOCK_SIZE)));
	int gridLengthRQ = ceil(sqrt((double)_totalRef*_totalQuery / ((double)BLOCK_SIZE)));
	dim3 threads(blockLength, blockLength, 1);
	dim3 blocks(gridLength, gridLength, 1);
	dim3 blocks_rq(gridLengthRQ, gridLengthRQ, 1);

	//Set obstacles to GPU memory
	Obstacle_Struct* obstacle_ptr = &obstacles[0];
	int numObstacles = obstacles.size();
	cudaMemcpy(_obsList, obstacle_ptr, obstacles.size()*sizeof(Obstacle_Struct), cudaMemcpyHostToDevice);

	// Run kernel
	cuda_generate_graph_mv1 << < blocks, threads >> >(_nodeList, _adjMatrix, _totalNodes, _obsList, obstacles.size(), obsType);

	// Calculate degree
	cuda_calc_degree << <blocks_rq, threads, _totalQuery*sizeof(int) >> >(_kresults, _adjMatrix, _nodeList, _totalNodes, _totalRef, _totalQuery, _refOnly);

	// *****************SYNC
	cudaDeviceSynchronize();
}

char* gpuVG::get_adjMatrix()
{
	//Copy memory back to host
	char* adjMatrix_ptr = (char*)malloc(_totalNodes*_totalNodes*sizeof(char));
	cudaMemcpy(adjMatrix_ptr, _adjMatrix, _totalNodes*_totalNodes*sizeof(char), cudaMemcpyDeviceToHost);
	return adjMatrix_ptr;
}

int* gpuVG::get_depthMatrix()
{
	//Copy memory back to host
	int* depthMatrix_ptr = (int*)malloc(_totalQuery*MAX_DEPTH*sizeof(int));
	cudaMemcpy(depthMatrix_ptr, _depthMat, _totalQuery*MAX_DEPTH*sizeof(int), cudaMemcpyDeviceToHost);
	return depthMatrix_ptr;
}

int* gpuVG::get_degree()
{
	int blockLength = ceil(sqrt((double)BLOCK_SIZE));
	int gridLength = ceil((double)_totalQuery / BLOCK_SIZE);
	int gridLengthX = ceil((double)_totalNodes*_totalNodes / BLOCK_SIZE);
	dim3 threads(blockLength, blockLength, 1);
	dim3 blocks(gridLength, 1, 1);
	dim3 blocks_2(gridLengthX, 1, 1);

	// Initialize output
	cudaMemset(_output, 0, sizeof(int));

	// Calculate degree
	//cuda_calc_degree << <blocks_2, threads, _totalNodes*sizeof(int) >> >(_kresults, _adjMatrix, _nodeList, _totalNodes);

	// Take sum over nodes
	reduced_add_int << <blocks, threads, BLOCK_SIZE*sizeof(int) >> >(_kresults, &_output[0], _totalNodes);
	int* output = (int*)malloc(sizeof(int));
	cudaMemcpy(output, _output, sizeof(int), cudaMemcpyDeviceToHost);
	return output;
}

void gpuVG::generate_forest()
{
	// Setup block and grid size
	int blockLength = ceil(sqrt((double)BLOCK_SIZE));
	int gridLengthX = ceil((double)_totalNodes*_totalNodes / BLOCK_SIZE);
	int gridLengthY = ceil((double)_totalNodes / BLOCK_SIZE);
	int gridLengthQ = ceil((double)_totalQuery / BLOCK_SIZE);
	int gridLengthR = ceil((double)_totalRef / BLOCK_SIZE);
	int gridLengthRQ = ceil((double)_totalRef*_totalQuery / BLOCK_SIZE);
	dim3 threads(BLOCK_SIZE, 1, 1);
	dim3 blocks_2(gridLengthX, 1, 1);
	dim3 blocks_1(gridLengthY, 1, 1);
	dim3 blocks_q(gridLengthQ, 1, 1);
	dim3 blocks_rq(gridLengthRQ, 1, 1);
	dim3 blocks_3q(gridLengthR, _totalQuery, 1);
	dim3 blocks_3(gridLengthY, _totalNodes, 1);
	cudaError_t err;

	// Init tree
	cudaMemset(_childMat, false, _totalQuery*_totalRef*sizeof(bool));
	cudaMemset(_parentMat, false, _totalQuery*_totalRef*sizeof(bool));
	cudaMemset(_depthMat, 0, _totalQuery*MAX_DEPTH*sizeof(int));
	//cuda_init_forest << < blocks_2, threads >> >(_parentMat, _childMat, _frontMat, _seqFront, _depthMat, _totalNodes);
	cuda_init_forest << < blocks_q, threads >> >(_parentMat, _childMat, _frontMat, _seqFront, _depthMat, _totalRef, _totalQuery, _refOnly);

	//int* res = (int*)malloc(_totalNodes*_totalNodes*sizeof(int));
	for (int i = 0; i < MAX_DEPTH; i++)
	{
		// Run kernel
		//cuda_expand_forest << < blocks_3, threads, _totalNodes*sizeof(int) >> >(_parentMat, _childMat, _seqFront, _depthMat, _nodeList, _adjMatrix, i, _totalNodes);
		cuda_expand_forest << < blocks_3q, threads >> >(_parentMat, _childMat, _seqFront, _depthMat, _nodeList, _adjMatrix, i, _totalNodes, _totalRef, _totalQuery);

		// Update tree
		//cuda_update_forest << < blocks_2, threads, _totalNodes*sizeof(int) >> >(_parentMat, _childMat, _frontMat, _seqFront, _depthMat, i, _totalRef, _totalQuery);
		cuda_update_forest << < blocks_rq, threads, _totalQuery*sizeof(int) >> >(_parentMat, _childMat, _frontMat, _seqFront, _depthMat, i, _totalRef, _totalQuery);

		// Make front list
		if (i < MAX_DEPTH - 1)
			cuda_sequence_list << < blocks_q, threads >> >(_frontMat, _seqFront, _totalRef, _totalQuery);
	}
	dim3 threadsD(MAX_DEPTH, 1, 1);
	dim3 blocksD(_totalQuery, 1, 1);

	// Calculate depth and entropy
	cuda_extract_forest << <blocksD, threadsD >> >(_dresults, _eresults, _depthMat, _nodeList, _totalNodes);
}

int* gpuVG::get_output()
{
	int blockLength = ceil(sqrt((double)BLOCK_SIZE));
	int gridLength = ceil((double)_totalQuery / BLOCK_SIZE);
	int gridLengthX = ceil((double)_totalNodes*_totalNodes / BLOCK_SIZE);
	dim3 threads(blockLength, blockLength, 1);
	dim3 blocks(gridLength, 1, 1);
	dim3 blocks_2(gridLengthX, 1, 1);

	// Initialize output
	cudaMemset(_output, 0, 3 * sizeof(int));

	// Calculate degree
	//cuda_calc_degree << <blocks_2, threads, _totalNodes*sizeof(int) >> >(_kresults, _adjMatrix, _nodeList, _totalNodes);

	// Take sum over nodes
	reduced_add_int << <blocks, threads, BLOCK_SIZE*sizeof(int) >> >(_kresults, &_output[0], _totalQuery);
	reduced_add_int << <blocks, threads, BLOCK_SIZE*sizeof(int) >> >(_dresults, &_output[1], _totalQuery);
	reduced_add_float << <blocks, threads, BLOCK_SIZE*sizeof(float) >> >(_eresults, (float*)&_output[2], _totalQuery);
	int* output = (int*)malloc(3 * sizeof(int));
	cudaMemcpy(output, _output, 3 * sizeof(int), cudaMemcpyDeviceToHost);
	return output;
}

int* gpuVG::get_nodesDegree()
{
	int* output = (int*)malloc(_totalQuery * sizeof(int));
	cudaMemcpy(output, _kresults, _totalQuery * sizeof(int), cudaMemcpyDeviceToHost);
	return output;
}

int* gpuVG::get_nodesDepth()
{
	int* output = (int*)malloc(_totalQuery * sizeof(int));
	cudaMemcpy(output, _dresults, _totalQuery * sizeof(int), cudaMemcpyDeviceToHost);
	return output;
}

float* gpuVG::get_nodesEntropy()
{
	float* output = (float*)malloc(_totalQuery * sizeof(float));
	cudaMemcpy(output, _eresults, _totalQuery * sizeof(float), cudaMemcpyDeviceToHost);
	return output;
}
