/*
 * RRTplanner.h
 *
 *  Created on: Feb 28, 2018
 *      Author: abhishek
 */

#ifndef RRTPLANNER_H_
#define RRTPLANNER_H_

#pragma once
#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <cmath>
#include <random>
#include <limits>
#include <cstdint>
#include <stdlib.h>
#include <fstream>
using namespace std;
using namespace OpenRAVE;

bool debug = false;
#define FORWARD 1
#define BACKWARD -1
//template <typename T>
class RRTNode{
	vector<double> _q;
	RRTNode* parent;
public:
	RRTNode(vector<double> cfg, RRTNode* prnt);

	double distanceMeasure(RRTNode* nde, RobotBasePtr rob);

	RRTNode* getParent(){
		return parent;
	}

	void setParent(RRTNode* prnt) {
		parent = prnt;
	}

	vector<double> getNodeConfig(){
		return _q;
	}

	void setNodeConfig(vector<double> q) {
		_q = q;
	}
};

class NodeTree{
private:
	vector<RRTNode*> _nodes;
public:
	NodeTree();
	void addNode(RRTNode* nde);
	void deleteNode(RRTNode* nde);
	RRTNode* getNode(int index);
	vector<vector<double> > generatePath(RRTNode* final);
	RRTNode* getNearestNeighbor(RRTNode* nde, RobotBasePtr rob);
	vector<vector<double> > generatePath(int finalIndex);
	int getSize(){
		return _nodes.size();
	}
};

class RRT{
	RRTNode* startState;
	RRTNode* goalState;
	double goalBias;
	int maxIterations;
	vector<double> lowerLimits;
	vector<double> upperLimits;
	double stepSize;
	EnvironmentBasePtr env;
	RobotBasePtr pr2;
	vector<GraphHandlePtr> handles;

public:
	RRT(vector<double> srt, vector<double> gl, double gB, int maxIter, double step, EnvironmentBasePtr aenv);
	bool isGoal(RRTNode* nde);
	string buildRRTConnect(ofstream& outdata);
	string buildBiRRTConnect();
	RRTNode* newConfig(RRTNode* firstState, RRTNode* secondState);
	RRTNode* randomConfig();
	string connect(NodeTree* treeAlg, RRTNode* &firstState);
	string extend(NodeTree* treeAlg, RRTNode* &firstState, RRTNode* &secondState);
	void swapTrees(NodeTree* treeOne, NodeTree* treeTwo);
	void printLimits();
	bool safePoints(vector<double> point);
	vector<vector<double> > smoothPath(vector<vector<double> > generatedPath);
	void plotPath(vector<vector<double> > path, int color);
	void executeTrajectory(vector<vector<double> > path);
	double getPathLength(vector<vector<double> > path);
};
#endif /* RRTPLANNER_H_ */

double* convertVectortoArray(vector<double> config){
	double* temp = new double[config.size()];
	for (int index = 0; index<config.size(); index++){
		temp[index] = (double) config[index];
	}
	return temp;
}

void printVector(vector<double> v, ostream &out){
	out<<"<";
	for (int index = 0; index<v.size(); index++){
		out<<v[index]<<" ";
	}
	out<<">\n";
}


void RRT::printLimits(){
	for (int i = 0; i<lowerLimits.size(); i++){
		cout << "The limits are <"<<lowerLimits[i]<<", "<<upperLimits[i]<<">\n";
	}
}

RRTNode::RRTNode(vector<double> cfg, RRTNode* prnt) {
	_q = cfg;
	parent = prnt;
}
double RRTNode::distanceMeasure(RRTNode* nde, RobotBasePtr rob)
{
	double distance = 0.0;
	vector<double> first;
	vector<double> second;
	first = nde->getNodeConfig();
	second = _q;
	rob->SubtractActiveDOFValues(first, second);
	for (int i = 0; i<first.size(); i++){
		distance += pow((first[i]),2);
		// distance += pow((first[i]-second[i]),2);
	}
	distance = sqrt(distance);
	return distance;
}


/**
 * Constructor for NodeTree
 */
NodeTree::NodeTree() {
}

/**
 * Add nodes to the node tree
 * @param node: Node to be added
 */
void NodeTree::addNode(RRTNode* nde) {
	_nodes.push_back(nde);
}

/**
 * Delete a node from the node tree
 * @param node: Node to be deleted
 */
void NodeTree::deleteNode(RRTNode* nde) {
	for(auto it = _nodes.begin(); it<_nodes.end(); it++)
	{
		if((*it)->getNodeConfig() == nde->getNodeConfig())
		{
			_nodes.erase(it);
		}
	}
}

/**
 * Get the node according to the index in the tree.
 * The index starts from 0 and goes until size of tree-1.
 * @param index
 * @return
 */
RRTNode* NodeTree::getNode(int index) {
	return _nodes[index];
}

vector<vector<double> > NodeTree::generatePath(int finalIndex){
	return generatePath(_nodes[finalIndex]);
}

vector<vector<double> > NodeTree::generatePath(RRTNode* final) {
	vector<vector<double> > path;
	path.push_back(final->getNodeConfig());
	while(final->getParent()!=NULL){
		final = final->getParent();
		path.push_back(final->getNodeConfig());
	}
	reverse(path.begin(), path.end());
	return path;
}

RRTNode* NodeTree::getNearestNeighbor(RRTNode* nde, RobotBasePtr rob) {
	double distance = numeric_limits<double>::max();
	RRTNode* nearestNeighbor;
	for(auto it = _nodes.begin(); it!=_nodes.end(); it++){
		double tempDistance = (*it)->distanceMeasure(nde, rob);
		if (distance > tempDistance){
			distance = tempDistance;
			nearestNeighbor = (*it);
		}
	}
	return nearestNeighbor;
}


RRT::RRT(vector<double> srt, vector<double> gl, double gB, int maxIter, double step, EnvironmentBasePtr aenv) {
	goalBias = gB;
	maxIterations = maxIter;
	stepSize = step;
	env = aenv;
	vector<RobotBasePtr> robots;
	env->GetRobots(robots);
	env->GetCollisionChecker()->SetCollisionOptions(CO_RayAnyHit);
	pr2 = robots[0];
	pr2->GetActiveDOFLimits(lowerLimits, upperLimits);
	lowerLimits[4] = -M_PI;
	lowerLimits[6] = -M_PI;
	upperLimits[4] = M_PI;
	upperLimits[6] = M_PI;
	for (uint i =0 ;i< lowerLimits.size(); i++){
        if (srt[i] < lowerLimits[i]){
          srt[i] = lowerLimits[i];
        }
        if (gl[i] < lowerLimits[i]){
          gl[i] = lowerLimits[i];
        }
        if (srt[i] > upperLimits[i]){
          srt[i] = upperLimits[i];
        }
        if (gl[i] > upperLimits[i]){
          gl[i] = upperLimits[i];
        }
        if (lowerLimits[i] == -10000){ //Rotational Joint with no constraints;
          lowerLimits[i] = -M_PI;
          upperLimits[i] = M_PI;
        }
  	}
	startState = new RRTNode(srt, NULL);
	goalState = new RRTNode(gl, NULL);
	cout<<gB;
}

	// if (graphing){
	NodeTree* tree = new NodeTree();
	string RRT::buildRRTConnect(ofstream& out){
	tree->addNode(startState);

	RRTNode* qRand;
	vector<vector<double> > finalPath;
	vector<vector<double> > smPath;
	clock_t timeOn = clock();
	while(double(clock()-timeOn)<(180*CLOCKS_PER_SEC*2)){
		qRand = randomConfig();
		string status = connect(tree, qRand);
		if ((status.compare("Reached") == 0)&&isGoal(qRand)){
			finalPath = tree->generatePath(tree->getNode(tree->getSize()-1));
			double diff = double(clock()-timeOn)/CLOCKS_PER_SEC;
			cout << "Reached the goal";
			cout<<"The tree size is "<<tree->getSize();
			cout<<"The finalpath length is "<<finalPath.size();
			for (int i = 0; i<finalPath.size(); i++)
			{
				cout<<endl;
				printVector(finalPath[i],cout);
			}
			plotPath(finalPath,1);
			smPath = smoothPath(finalPath);
			plotPath(smPath,2);
			executeTrajectory(smPath);
      // out << "The RRT computation time is "<<diff<<endl;
			// out << "The number of nodes sampled are "<<tree->getSize()<<endl;
			// out << "The length of unsmoothed path is "<<finalPath.size()<<endl;
			// out << diff <<","<< tree->getSize() <<","<< finalPath.size()<<",";
			// clock_t startSmoothing = clock();
			// vector<vector<double> > smPath = smoothPath(finalPath);
			// diff = double(clock()-startSmoothing)/CLOCKS_PER_SEC;
			// out << "The smoothing computation time is "<<diff<<endl;
			// out << "The length of smoothed path is "<<smPath.size()<<endl;
			// out << diff <<","<< smPath.size()<<endl;
			return "Reached";
		}
	}
	cout<<"Failure";
	return "Failure";
}

RRTNode* RRT::randomConfig() {
	double r = rand()/(double) RAND_MAX;
	vector<double> randConfig;
	if (r<goalBias){
		return goalState;
	}
	else{
		do{
			for(int i = 0; i < goalState->getNodeConfig().size(); i++)
			{
				double range = upperLimits[i] - lowerLimits[i];
				double rnd = lowerLimits[i] + ((range * (double)rand()) / (double)RAND_MAX);
				randConfig.push_back(rnd);
			}
			//TODO Removed the collision module from here. This improves speed conssiderably.
			// pr2->SetActiveDOFValues(randConfig);
			// if (env->CheckCollision(pr2)||pr2->CheckSelfCollision()||!(this->safePoints(randConfig)))
			// {
			// 	randConfig.clear();
			// }
		}while(randConfig.size() != goalState->getNodeConfig().size());
	}
	RRTNode* temp = new RRTNode(randConfig, NULL);
	return temp;
}

string RRT::connect(NodeTree* tree, RRTNode* &randState) {
	string status = "Advanced";
	RRTNode* nearState = tree->getNearestNeighbor(randState, pr2);
	while(status == "Advanced"){
		status = extend(tree, randState, nearState);
	}
	return status;
}

double RRT::getPathLength(vector<vector<double> > path)
{
	double distance = 0.0;
	RRTNode* first;
	RRTNode* second;
	for(int index = 0; index<path.size()-1; index++){
		first = new RRTNode(path[index], NULL);
		second = new RRTNode(path[index+1], NULL);
		distance = distance+second->distanceMeasure(first, pr2);
	}
	return distance;
}
void RRT::swapTrees(NodeTree* treeOne, NodeTree* treeTwo){
	NodeTree* temp = treeOne;
	treeOne = treeTwo;
	treeTwo = temp;
}
string RRT::buildBiRRTConnect() {
	int direction = FORWARD;
	NodeTree* treeForward = new NodeTree();
	NodeTree* treeBackward = new NodeTree();
	treeForward->addNode(startState);
	treeBackward->addNode(goalState);
	vector<vector<double> > finalForwardPath;
	vector<vector<double> > finalBackwardPath;
	vector<vector<double> > smoothFinalPath;
	for (int k = 0; k<maxIterations; k++){
		RRTNode* qRand = randomConfig();
		if (debug) cout<<k<<endl;
		if (debug) cout<<direction<<endl;
		if (direction == FORWARD)
		{
			RRTNode* nearState = treeForward->getNearestNeighbor(qRand, pr2);
			string statusExtend = extend(treeForward, qRand, nearState);
			if (debug) cout<<statusExtend<<endl;
			if (statusExtend=="Reached"){
				RRTNode aim = *qRand;
				RRTNode* aimState = &aim;
				if (connect(treeBackward, aimState) == "Reached"){
					finalForwardPath = treeForward->generatePath(treeForward->getNode(treeForward->getSize()-1));
					finalBackwardPath = treeBackward->generatePath(treeBackward->getNode(treeBackward->getSize()-1));
					reverse(finalBackwardPath.begin(), finalBackwardPath.end());
					finalForwardPath.insert(finalForwardPath.end(), finalBackwardPath.begin(), finalBackwardPath.end());
					cout<<"Reached the goal";
					for(int index=0; index<finalForwardPath.size(); index++){
						cout<<endl;
						printVector(finalForwardPath[index], cout);
						cout<<endl;
					}
					cout<<"The path size is "<<finalForwardPath.size();
					plotPath(finalForwardPath,1);
					smoothFinalPath = smoothPath(finalForwardPath);
					plotPath(smoothFinalPath,2);
					executeTrajectory(smoothFinalPath);
					cout<<"The path size is "<<smoothFinalPath.size();

					return "success";
					// return path(treeForward, treeBackward);
				}
			}
			else if(statusExtend=="Advanced"){
				RRTNode aim = *nearState;
				RRTNode* aimState = &aim;
				if (connect(treeBackward, aimState) == "Reached"){
					finalForwardPath = treeForward->generatePath(treeForward->getNode(treeForward->getSize()-1));
					finalBackwardPath = treeBackward->generatePath(treeBackward->getNode(treeBackward->getSize()-1));
					reverse(finalBackwardPath.begin(), finalBackwardPath.end());
					finalForwardPath.insert(finalForwardPath.end(), finalBackwardPath.begin(), finalBackwardPath.end());
					cout<<"Reached the goal";
					for(int index=0; index<finalForwardPath.size(); index++){
						cout<<endl;
						printVector(finalForwardPath[index], cout);
						cout<<endl;
					}
					cout<<"The path size is "<<finalForwardPath.size();
					plotPath(finalForwardPath,1);
					smoothFinalPath = smoothPath(finalForwardPath);
					plotPath(smoothFinalPath,2);
					executeTrajectory(smoothFinalPath);
					cout<<"The path size is "<<smoothFinalPath.size();
					return "success";
					// return path(treeForward, treeBackward);
				}
			}
		}
		else if(direction == BACKWARD){
			RRTNode* nearState = treeBackward->getNearestNeighbor(qRand, pr2);
			if (debug) cout<<"I am before statusExtend"<<endl;
			if (debug) cout<<treeBackward->getSize()<<endl;
			if (debug) printVector(qRand->getNodeConfig(), cout);
			string statusExtend = extend(treeBackward, qRand, nearState);
			if (debug) cout<<"Backward "<<statusExtend<<endl;
			if (statusExtend=="Reached"){
				RRTNode aim = *qRand;
				RRTNode* aimState = &aim;
				if (connect(treeForward, aimState) == "Reached"){
					finalForwardPath = treeForward->generatePath(treeForward->getNode(treeForward->getSize()-1));
					finalBackwardPath = treeBackward->generatePath(treeBackward->getNode(treeBackward->getSize()-1));
					reverse(finalBackwardPath.begin(), finalBackwardPath.end());
					finalForwardPath.insert(finalForwardPath.end(), finalBackwardPath.begin(), finalBackwardPath.end());
					cout<<"Reached the goal";
					for(int index=0; index<finalForwardPath.size(); index++){
						cout<<endl;
						printVector(finalForwardPath[index], cout);
						cout<<endl;
					}
					cout<<"The path size is "<<finalForwardPath.size();
					plotPath(finalForwardPath,1);
					smoothFinalPath = smoothPath(finalForwardPath);
					plotPath(smoothFinalPath,2);
					executeTrajectory(smoothFinalPath);
					cout<<"The path size is "<<smoothFinalPath.size();
					return "success";
					// return path(treeForward, treeBackward);
				}
			}
			else if(statusExtend=="Advanced"){
				RRTNode aim = *nearState;
				RRTNode* aimState = &aim;
				if (connect(treeForward, aimState) == "Reached"){
					finalForwardPath = treeForward->generatePath(treeForward->getNode(treeForward->getSize()-1));
					finalBackwardPath = treeBackward->generatePath(treeBackward->getNode(treeBackward->getSize()-1));
					reverse(finalBackwardPath.begin(), finalBackwardPath.end());
					finalForwardPath.insert(finalForwardPath.end(), finalBackwardPath.begin(), finalBackwardPath.end());
					cout<<"Reached the goal";
					for(int index=0; index<finalForwardPath.size(); index++){
						cout<<endl;
						printVector(finalForwardPath[index], cout);
						cout<<endl;
					}
					cout<<"The path size is "<<finalForwardPath.size();
					plotPath(finalForwardPath,1);
					smoothFinalPath = smoothPath(finalForwardPath);
					plotPath(smoothFinalPath,2);
					executeTrajectory(smoothFinalPath);
					cout<<"The path size is "<<smoothFinalPath.size();
					return "success";
					// return path(treeForward, treeBackward);
				}
			}

		}
		direction = -1*direction;
	}
	return "Failure";
}

RRTNode* RRT::newConfig(RRTNode* nearState, RRTNode* randState) {
	vector<double> nearConfig = nearState->getNodeConfig();
	vector<double> randConfig = randState->getNodeConfig();
	vector<double> tempConfig;
	vector<double> newConfig;
	RRTNode* nearNode = new RRTNode(nearConfig, NULL);
	RRTNode* randNode = new RRTNode(randConfig, NULL);

	double distance = nearNode->distanceMeasure(randNode, pr2);
	tempConfig = randConfig;
	pr2->SubtractActiveDOFValues(tempConfig, nearConfig);
	for(int i = 0; i<nearConfig.size();i++){
		double temp = nearConfig[i] + (stepSize*(tempConfig[i])/distance);
		newConfig.push_back(temp);
	}
//	while(true){
	if ((this->safePoints(newConfig)))
	{
		pr2->SetActiveDOFValues(newConfig);
		if (env->CheckCollision(pr2)||pr2->CheckSelfCollision())
		{
			return NULL;
		}
		else{
			RRTNode* newState = new RRTNode(newConfig, nearState);
			return newState;
		}
	}
	else{
		return NULL;
	}
}
bool RRT::safePoints(vector<double> randConfig){
	for(int i = 0; i<randConfig.size(); i++)
	{
		if ((randConfig[i]<lowerLimits[i])||(randConfig[i]>upperLimits[i]))
		{
			return false;
		}
	}
	return true;
}

string RRT::extend(NodeTree* tree, RRTNode* &randState, RRTNode* &nearState) {
	RRTNode* newState = newConfig(nearState, randState);
	string status = "";
	if (newState == NULL)
	{
		status = "Trapped";
		return status;
	}
	else{
		if (randState->distanceMeasure(newState, pr2)<stepSize){
			tree->addNode(newState);
			randState->setParent(newState);
			tree->addNode(randState);
			status = "Reached";
			return status;
		}
		else{
			tree->addNode(newState);
			status = "Advanced";
			nearState = newState;
			return status;
		}
	}
}

bool RRT::isGoal(RRTNode* node) {
	if (node->distanceMeasure(goalState, pr2)<stepSize){
		return true;
	}
	return false;
}

// ostream& operator<< (ostream &out, RRT const& data)
// {
// 	// out<<"The start state is ";
// 	// printVector(startState->getNodeConfig(), out);
// 	// out<<"The goal state is ";
// 	// printVector(goalState->getNodeConfig(), out);
// 	out<< "The goal bias, max iterations, step size and limits are "<<endl;
// 	out<<data.goalBias<<data.maxIterations<<data.stepSize;
// 	data.printLimits();
// }
//
// ostream& operator<< (ostream &out, RRTNode const& data)
// {
// 	printVector(data._q, out);
// 	out<<"The parent of the node is "<<data.parent;
// }
//
// ostream& operator<< (ostream &out, NodeTree const& data)
// {
// 	out <<"The tree size is "<<data.getSize();
// }

template <typename T>
ostream& operator<< (ostream &out, vector<T>& data)
{
	if (!data.empty()){
		out <<"The vector is ";
		for (int index = 0; index<data.size(); index++){
			out<<data[index];
		}
		out<<endl;
	}
	return out;
}

//TODO Can try the stepSize/2 formula
vector<vector<double> > RRT::smoothPath(vector<vector<double> > generatedPath){
	RRTNode* firstNode;
	RRTNode* secondNode;
	vector<vector<double> > unsmoothedPath = generatedPath;
	vector<vector<double> > smoothedPath;
	ofstream smoothCSV;
	smoothCSV.open("/home/abhishek/workspace/rrt/Data/pathLength.csv");
	for(int iteration = 0; iteration<200; iteration++){
		int firstIndex = 0;
		int secondIndex = 0;
		do{
			firstIndex = rand()%(generatedPath.size());
			secondIndex = rand()%(generatedPath.size());
		}while(firstIndex == secondIndex);
		if (firstIndex>secondIndex){
			unsigned int temp;
			temp = firstIndex;
			firstIndex = secondIndex;
			secondIndex = temp;
		}
		firstNode = new RRTNode(generatedPath[firstIndex],NULL);
		secondNode = new RRTNode(generatedPath[secondIndex],NULL);
		if ((firstNode->distanceMeasure(secondNode, pr2))<=stepSize)
		{
			//If the distance between the nodes is smaller than the stepSize we are considering,then we simply add the line between the points
			//by deleting the nodes between these two nodes.
			// for(auto it = generatedPath.begin(); it<generatedPath.begin()+firstIndex+1; it++)
			// {
			// 	smoothedPath.push_back(*it);
			// }
			// for (auto it = generatedPath.begin()+secondIndex; it!=generatedPath.end(); it++){
			// 	smoothedPath.push_back(*it);
			// }
			// generatedPath = smoothedPath;
			generatedPath.erase(generatedPath.begin()+firstIndex+1,generatedPath.begin()+secondIndex);
		}
		else{
			bool success = true;
			while (firstNode->distanceMeasure(secondNode, pr2)>stepSize){
				vector<double> newConfig;
				vector<double> tempConfig;
				double distance = firstNode->distanceMeasure(secondNode, pr2);
				tempConfig = secondNode->getNodeConfig();
				pr2->SubtractActiveDOFValues(tempConfig, firstNode->getNodeConfig());
				for(int i = 0; i<firstNode->getNodeConfig().size();i++){
					//TODO the stepSize*(firstNode[i] - secondNode[i]) term here and in the newConfig function would take bigger leaps
					//when the distance between the nodes is more which may be error prone. I feel a small distance that is
					//safe should be used instead.
					//Edit: I have added the step distance extending to this function correctly. Try to change the newConfig also

					double temp = firstNode->getNodeConfig()[i] + (stepSize*(tempConfig[i])/distance);
					newConfig.push_back(temp);
				}
			//	while(true){
				pr2->SetActiveDOFValues(newConfig);
				if (env->CheckCollision(pr2)||pr2->CheckSelfCollision())
				{
					//If obstacle is encountered in between the path to the second goal. Make another iteration so break
					success = false;
					break;
				}
				else{
					//If obstacle is not encountered change the firstNode so that we move towards the secondNode in an attempt
					//to make a line between the two nodes.
					firstNode->setNodeConfig(newConfig);
				}
			}
			if (success){
				//Path between the firstNode and secondNode is possible.
				// for(auto it = generatedPath.begin(); it<generatedPath.begin()+firstIndex+1; it++)
				// {
				// 	smoothedPath.push_back(*it);
				// }
				// for (auto it = generatedPath.begin()+secondIndex; it!=generatedPath.end(); it++){
				// 	smoothedPath.push_back(*it);
				// }
				// generatedPath = smoothedPath;
				generatedPath.erase(generatedPath.begin()+firstIndex+1,generatedPath.begin()+secondIndex);
			}
			else{
				//Path between the firstNode and secondNode contains obstacles.
			}
		}
		bool plotPathLength = false;
		if (plotPathLength){
			double distance = this->getPathLength(generatedPath);
			smoothCSV <<iteration<<","<<distance<<endl;
		}
	}
	smoothCSV.close();
	return generatedPath;
}

void RRT::plotPath(vector<vector<double> > path, int color){
	vector<double> activeDOF;
  vector<float> locations;
  float red[4]={1,0,0,1},blue[4]={0,0,1,1};
  for(int i=0; i<path.size(); i++)
  {
    locations.clear();
    activeDOF.clear();
    for(int j=0;j<path[i].size();j++)
      activeDOF.push_back(path[i][j]);
    pr2->SetActiveDOFValues(activeDOF);
    //getting end effector position
    Transform T=pr2->GetLinks()[49]->GetTransform();
    locations.push_back((float)T.trans.x);
    locations.push_back((float)T.trans.y);
    locations.push_back((float)T.trans.z);
    locations.push_back(1);
    if(color==1)
      handles.push_back(env->plot3(&locations[0],1,1,5,red,0));
    else
      handles.push_back(env->plot3(&locations[0],1,1,5,blue,0));
  }
}

void RRT::executeTrajectory(vector<vector<double> > path)
{
	TrajectoryBasePtr ptraj = RaveCreateTrajectory(env,"");
	ConfigurationSpecification conspec=pr2->GetActiveConfigurationSpecification("linear");
	conspec.AddDeltaTimeGroup();
	ptraj->Init(conspec);
	vector<double> path_pt;
  for(unsigned int i=0;i<path.size();i++)
  {
    path_pt=path[i];
    path_pt.push_back(i*0.01);
    ptraj->Insert(i,path_pt,conspec,true);
  }
  pr2->GetController()->SetPath(ptraj);
}
