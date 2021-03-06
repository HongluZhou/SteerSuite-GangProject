#ifndef _GraphSearch_H_
#define _GraphSearch_H_

#include "Structs.h"
#include <vector>

using namespace std;

bool selectBestGPU();

class gpuVG
{
public:
	gpuVG() : _isInit(false) {}
	~gpuVG();
	void init(vector<Point_Struct> nodeList, unsigned obsCount, unsigned refCount, unsigned queryCount, unsigned refOnlyCount);
	void generate_graph(vector<Obstacle_Struct> obstacles, char obsType = BLOCK_OBSTACLE);
	char* get_adjMatrix();
	int* get_depthMatrix();
	int* get_depthSize();
	void clear();
	bool is_init() { return _isInit; }
	void generate_forest();
	int* get_output();
	int* get_degree();
	int* get_nodesDegree();
	int* get_nodesDepth();
	float* get_nodesEntropy();

private:
	char* _adjMatrix;
	bool* _parentMat;
	bool* _childMat;
	bool* _frontMat;
	Point_Struct* _nodeList;
	Obstacle_Struct* _obsList;
	int _totalNodes, _totalObs, _totalRef, _totalQuery, _refOnly;
	bool _isInit;
	int* _depthMat;
	float* _eresults;
	int* _dresults, *_kresults;
	int* _output;
	int* _seqFront;
	std::vector<Point_Struct> _host_nodeList;
};

#endif