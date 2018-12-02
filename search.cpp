#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <queue>
#include <unordered_set>
#include <set>

using namespace std;

#define numNeighbors 8
int neighborX[numNeighbors] = {-1,-1,-1,0,1,1,1,0};
int neighborY[numNeighbors] = {-1,0,1,1,1,0,-1,-1};

int sizeX;
int sizeY;
float epsilon;
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
	int startX, int startY, double ***predictions,double *predictionTimes, int numPredictions);
void ARAstar(double speed, int startX, int startY,double ***predictions, double *predictionTimes, int numPredictions, 
	vector<int> *PathX, vector<int> *PathY, vector<double> *PathT);
bool reachedGoal(node nodeToCheck);
void backTrace(unordered_set<node,nodeHasher,nodeComparator> *states, node lastNode,
	int startX, int startY, vector<int> *PathX, vector<int> *PathY, vector<double> *PathT);
double fVal(double g, int x, int y);

class fCompare
{
	public:
	comp();
	bool operator() (const node& lhs, const node& rhs) const
	{
		return (lhs.f > rhs.f);//(lhsF > rhsF);
	}
};

main()
{
	// set size of map and goal position
	sizeX = 200;
	sizeY = 200;
	goalX = 199;
	goalY = 199;
	epsilon = 1;

	// random people position at t=0
	int numPeople = 0;
	double initXpos[numPeople]; 
	double initYpos[numPeople];
	srand (time(NULL));
	for (int i = 0; i < numPeople;i++)
	{
		initXpos[i] = rand()%(sizeX-1);
		initYpos[i] = rand()%(sizeY-1);
	}

	// set up matrices of predictions
	int numPredictions = 12;
	double predictionTimes[numPredictions];
	double timeBetweenPredictions = 100;
	double ***predictions = (double***) malloc(numPredictions*sizeof(double**));
	for (int i = 0; i < numPredictions; i++)
	{
		predictionTimes[i] = i*timeBetweenPredictions;
		predictions[i] = (double**) malloc(sizeX*sizeof(double*));
		for (int j = 0;j < sizeX;j++)
		{
			predictions[i][j] = (double*) malloc(sizeY*sizeof(double));
		}
	}

	// set up initial gaussian distribution
	for (int i = 0; i < sizeX; i ++)
	{
		for (int j = 0; j < sizeX; j ++)
		{
			double val = 0;
			for (int k = 0;k < numPeople;k++)
			{
				val = val + 100*exp(-pow(i-initXpos[k],2)/(40)-pow(j-initYpos[k],2)/(40));
			}
			predictions[0][i][j] = max(val,(double)1);
		}
	}

	for (int i = 1;i < numPredictions; i++)
	{
		for (int j = 0; j < sizeX; j ++)
		{
			predictions[i][j][0] = predictions[i-1][j][sizeY-1];
			for (int k = 1; k < sizeY; k++)
			{
				predictions[i][j][k] = predictions[i-1][j][k-1];
			}
		}
	}
	cout << "starting search\n";
	int startX = 0; int startY = 0; double speed = 10;
	vector<int> PathX; vector<int> PathY; vector<double> PathT;
	ARAstar(speed, startX, startY,predictions,predictionTimes,numPredictions, &PathX, &PathY, &PathT);
	for (int i=0;i<PathX.size();i++)
	{
		cout << "X = " << PathX[i] << ", Y = " << PathY[i] << ", T = " << PathT[i] << endl;
	}
}

void ARAstar(double speed, int startX, int startY,double ***predictions, double *predictionTimes, int numPredictions, 
	vector<int> *PathX, vector<int> *PathY, vector<double> *PathT)
{
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
		node lastNode = ComputePathWithReuse(speed, &states, startX, startY,predictions,predictionTimes,numPredictions);
		//publish solution
		backTrace(&states, lastNode, startX, startY, PathX, PathY, PathT);
	}
}

node ComputePathWithReuse(double speed, unordered_set<node,nodeHasher,nodeComparator> *states, 
	int startX, int startY, double ***predictions,double *predictionTimes, int numPredictions)
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
	while((OPEN.top().t < predictionTimes[numPredictions-1]) && !reachedGoal(OPEN.top()) && !OPEN.empty())
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
					while ((predictionTimes[upper] > tempT))
					{
						lower = upper;
						if (upper < numPredictions)
						{
							upper++;
						}
						else
							break;
					}
					double lastPredict = predictions[lower][tempX][tempY];
					double nextPredict = predictions[upper][tempX][tempY];
					double tempP = lastPredict + (nextPredict-lastPredict)*(tempT-predictionTimes[lower]);
					double tempG = (expand->g) + distance(thisX, thisY, tempX,tempY)*tempP;
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

void backTrace(unordered_set<node,nodeHasher,nodeComparator> *states, node lastNode,
	int startX, int startY, vector<int> *PathX, vector<int> *PathY, vector<double> *PathT)
{
	node tempState = lastNode;
	auto it = states->find(tempState);

	while ((tempState.x != startX) && (tempState.y != startY))
	{
		//cout << "X = " << it->x << ", Y = " << it->y << ", t = " << it->t << endl;
		PathX->insert(PathX->begin(),it->x); PathY->insert(PathY->begin(),it->y); PathT->insert(PathT->begin(),it->t);
		tempState.x = it->parentx; tempState.y = it->parenty; tempState.t = it->parentT;
		it = states->find(tempState);
	}
	//cout << "X = " << it->x << ", Y = " << it->y << ", t = " << it->t << endl;
	PathX->insert(PathX->begin(),it->x); PathY->insert(PathY->begin(),it->y); PathT->insert(PathT->begin(),it->t);
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