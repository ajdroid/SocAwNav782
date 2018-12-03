#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <queue>
#include <unordered_set>
#include <set>
#include "xtensor/pyarray.hpp"
#include "xtensor/xio.hpp"
#include "xtensor/xview.hpp"


using namespace std;

#define numNeighbors 8
int neighborX[numNeighbors] = {-1,-1,-1,0,1,1,1,0};
int neighborY[numNeighbors] = {-1,0,1,1,1,0,-1,-1};

int sizeX;
int sizeY;
float epsilon = 1;
int goalX;
int goalY;

struct node{
	int x;
	int y;
	double t;
	int parentx;
	int parenty;
	double parentT;
	double g;
	double f;
	bool open;
	bool closed;
	bool inconsistent;
};

struct nodeComparator
{
    bool operator()(const node& lhs, const node& rhs) const
    {
        return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.t == rhs.t);
    }
};

struct nodeHasher
{
    size_t operator()(const node& state) const
    {
        return state.y + sizeY*state.x + sizeX*sizeY*round(state.t*10000);
    }
};

double heuristic(int x, int y);
void indexToXY(int index, int* x, int* y);
double distance(int x1, int y1, int x2, int y2);
int XYtoIndex(int x, int y);
node ComputePathWithReuse(double speed, unordered_set<node,nodeHasher,nodeComparator> *states, 
	int startX, int startY, xt::pyarray<double> &predictions, xt::pyarray<double> &predictionTimes);
xt::pyarray<double> ARAstar(double speed, int startX, int startY, int _goalX, int _goalY, xt::pyarray<double> &predictions, xt::pyarray<double> &predictionTimes);
bool reachedGoal(node nodeToCheck);
xt::pyarray<double> backTrace(unordered_set<node,nodeHasher,nodeComparator> *states, node lastNode,int startX, int startY);
double fVal(double g, int x, int y);
xt::pyarray<double> testFunc(xt::pyarray<double> input);

class fCompare
{
	public:
	//comp();
	bool operator() (const node& lhs, const node& rhs) const
	{
		return (lhs.f > rhs.f);//(lhsF > rhsF);
	}
};


PYBIND11_MODULE(searcher, m)
{
    xt::import_numpy();
    m.doc() = "Searches graph for path to goal";

    m.def("graphSearch", ARAstar, "");
}


PYBIND11_MODULE(testMod, m)
{
    xt::import_numpy();
    m.doc() = "Searches graph for path to goal";

    m.def("testFunc", testFunc, "");
}

PYBIND11_MODULE(mainMod, m)
{
    xt::import_numpy();
    m.doc() = "Searches graph for path to goal";

    m.def("main", main, "");
}

main()
{
	// set size of map and goal position
	int _sizeX = 200;
	int _sizeY = 200;
	int _goalX = 199;
	int _goalY = 199;

	/*vector<double> predictionTimes;
	vector<xt::pyarray<double>> predictions;
	predictionTimes.push_back(0);
	predictionTimes.push_back(1000);
	predictions.push_back(xt::zeros<double>({_sizeX,_sizeY}));
	predictions.push_back(xt::zeros<double>({_sizeX,_sizeY}));*/
	xt::pyarray<double> predictionTimes;
	xt::pyarray<double> predictions;
	predictionTimes = {0,100,200};
	predictions = xt::zeros<double>({3,_sizeX,_sizeY});	

	cout << "starting search\n";
	int startX = 0; int startY = 0; double speed = 10;
	vector<int> PathX; vector<int> PathY; vector<double> PathT;
	xt::pyarray<double> solution = ARAstar(speed, startX, startY,_goalX,_goalY,predictions,predictionTimes);
	cout << solution << endl;
}

xt::pyarray<double> testFunc(xt::pyarray<double> input)
{
	return input+2;
}

xt::pyarray<double> ARAstar(double speed, int startX, int startY, int _goalX, int _goalY, xt::pyarray<double> &predictions, xt::pyarray<double> &predictionTimes)
{
	sizeX = predictions.shape()[1];//_sizeX;
	sizeY = predictions.shape()[2];//_sizeY;
	goalX = _goalX;
	goalY = _goalY;
	// initialize g values and open list for the first weighted Astar
	unordered_set<node, nodeHasher, nodeComparator> states;
	node newState;
	newState.x = startX;
	newState.y = startY;
	newState.t = 0;
	newState.g = 0;
	newState.open = true;
	states.insert(newState);
	int numOfEpsilons = 1;
	float epsilonList[numOfEpsilons] = {1};
	xt::pyarray<double> solution;

	for (int i = 0; i < numOfEpsilons; i++)
	{
		epsilon = epsilonList[i];

		// init values for search
		unordered_set<node, nodeHasher, nodeComparator> tempStates;
		for (node thisNode : states)
		{
			thisNode.f = fVal(thisNode.g, thisNode.x,thisNode.y);//thisNode.g + epsilon*heuristic(thisNode.x,thisNode.y);
			thisNode.open = (thisNode.open || thisNode.inconsistent);
			thisNode.inconsistent = false;
			thisNode.closed = false;
			tempStates.insert(thisNode);
		}
		states = tempStates;
		double tFound;
		node lastNode = ComputePathWithReuse(speed, &states, startX, startY,predictions,predictionTimes);
		//publish solution
		solution = backTrace(&states, lastNode, startX, startY);
	}
	return solution;
}

node ComputePathWithReuse(double speed, unordered_set<node,nodeHasher,nodeComparator> *states, 
	int startX, int startY, xt::pyarray<double> &predictions, xt::pyarray<double> &predictionTimes)
{
	// initialize priority queue used to choose states to expand
	priority_queue<node,vector<node>,fCompare> OPEN;
	
	// add nodes that should be in OPEN to the priority queue
	for (node thisNode : *states)
	{
		if (thisNode.open)
		{
			OPEN.push(thisNode);
		}
	}

	// Loop until either goal is next to expand (f goal is the smallest in open list) or no more nodes in open list
	while((OPEN.top().t < predictionTimes(predictionTimes.size()-1)) && !reachedGoal(OPEN.top()) && !OPEN.empty())
	{
		auto expand = states->find(OPEN.top());
		OPEN.pop();
		//cout << "state to expand1 x = " << OPEN.top().x << ", y = " << OPEN.top().y << ", t = " << OPEN.top().t << endl;		// Don't expand nodes that were already expanded and put into the CLOSED list
		//cout << "f = " << OPEN.top().f << endl;
		if (!(expand->closed))
		{
			//cout << "state to expand x = " << expand->x << ", y = " << expand->y << ", t = " << expand->t << endl;
			// Get X and Y position of the node to be expanded
			int thisX = expand->x; int thisY = expand->y;
			// Loop through node's neghbor
			for (int i =0; i < numNeighbors; i++)
			{
				// Get X and Y position as well as index for this neighbor
				int tempX = thisX + neighborX[i];
				int tempY = thisY + neighborY[i];
				double tempT = expand->t + distance(tempX,tempY,thisX,thisY)/speed;
				// make sure it is actually a valid location
				if ((tempX >= 0) && (tempX < sizeX) && (tempY >= 0) && (tempY < sizeY))
				{
					// update g value of this neighbor (g value of expanded node + distance times linearly interpolated prediction)
					int upper = 0;
					int lower = 0;
					while ((predictionTimes(upper) > tempT))
					{
						lower = upper;
						if (upper < predictionTimes.size())
						{
							upper++;
						}
						else
							break;
					}
					double lastPredict = predictions(lower,tempX,tempY);
					double nextPredict = predictions(upper,tempX,tempY);
					double tempP = lastPredict + (nextPredict-lastPredict)*(tempT-predictionTimes(lower));
					double tempG = (expand->g) + distance(thisX, thisY, tempX,tempY) + tempP;
					node tempState;
					tempState.x = tempX;
					tempState.y = tempY;
					tempState.t = tempT;
					auto thisNeighbor = states->find(tempState);
					if (thisNeighbor == states->end()) // new state. Add to states list
					{
						tempState.parentx = thisX;
						tempState.parenty = thisY;
						tempState.parentT = expand->t;
						tempState.g = tempG;
						tempState.f = fVal(tempG, tempX, tempY);//tempG + epsilon * heuristic(tempX,tempY);
						tempState.open = true;
						tempState.closed = false;
						tempState.inconsistent = false; // inconsistency recorded in the fact that it is in open
						states->insert(tempState);
						OPEN.push(tempState);
						//cout << "new state x = " << tempState.x << ", y = " << tempState.y << ", t = " << tempT << endl;
					}
					else
					{
						node thisNeighborModified = *(thisNeighbor);
						if (tempG < thisNeighbor->g)
						{
							thisNeighborModified.g = tempG;
							thisNeighborModified.f = fVal(tempG, tempX, tempY); //tempG + epsilon * heuristic(tempX,tempY);
							thisNeighborModified.parentx = thisX;
							thisNeighborModified.parenty = thisY;
							thisNeighborModified.parentT = expand->t;
							if(!(thisNeighbor->closed)) // insert this neighbor into OPEN list only if it isn't in closed list
							{
								thisNeighborModified.open = true;
								OPEN.push(thisNeighborModified);
							}
							else // otherwise, the neighbor becomes inconsistent
							{
								thisNeighborModified.inconsistent = true;
							}
						}
						states->erase(thisNeighbor);
						states->insert(thisNeighborModified);
					}
				}
			}
					// remove the expanded node from the OPEN list and insert into the CLOSED list
			node expandModified = *(expand);
			expandModified.open = false;
			expandModified.closed = true;
			states->erase(expand);
			states->insert(expandModified);
		}
	}
	return OPEN.top();
}


xt::pyarray<double> backTrace(unordered_set<node,nodeHasher,nodeComparator> *states, node lastNode,
	int startX, int startY)
{
	vector<int> PathX; vector<int> PathY; vector<double> PathT;
	node tempState = lastNode;
	auto it = states->find(tempState);

	while ((tempState.x != startX) && (tempState.y != startY))
	{
		PathX.insert(PathX.begin(),it->x); PathY.insert(PathY.begin(),it->y); PathT.insert(PathT.begin(),it->t);
		tempState.x = it->parentx; tempState.y = it->parenty; tempState.t = it->parentT;
		it = states->find(tempState);
	}
	PathX.insert(PathX.begin(),it->x); PathY.insert(PathY.begin(),it->y); PathT.insert(PathT.begin(),it->t);
	xt::pyarray<double> solution = xt::zeros<double>({3,(int)PathX.size()});
	for (int i = 0; i < PathX.size();i++)
	{
		solution(0,i) = PathX[i];
		solution(1,i) = PathY[i];
		solution(2,i) = PathT[i];
	}
	return solution;
}

double fVal(double g, int x, int y)
{
	//return heuristic(x,y) + 1;
	return (g + epsilon*heuristic(x,y));
}

double heuristic(int x, int y)
{
	double diffX = (goalX-x);
	double diffY = (goalY-y);
	double cost = sqrt(diffX*diffX + diffY*diffY);
	return cost;
}

double distance(int x1, int y1, int x2, int y2)
{
	double diffX = (x2 - x1);
	double diffY = (y2 - y1);
	double cost = sqrt(diffX*diffX + diffY*diffY);
	return cost;
}

bool reachedGoal(node nodeToCheck)
{
	return	(nodeToCheck.x == goalX) && (nodeToCheck.y == goalY);
}
