#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include "RRTplanner.h"

using namespace std;
using namespace OpenRAVE;

class planner : public ModuleBase
{
public:
    planner(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("RRT",boost::bind(&planner::MyCommand,this,_1,_2),
                        "Input format: RRT goal %f,%f,%f,%f,%f,%f,%f; goalBias %f; step 0.6;start %f,%f,%f,%f,%f,%f,%f;");
    }
    virtual ~planner() {}

    void generateStatistics(vector<double>  start, double step, double goalBias, vector<double> goal){
      ofstream outdata;
    	outdata.open("/home/abhishek/workspace/rrt/Data/pathPlot.csv");
      outdata << "RRT computation time, number of nodes sampled, length of unsmoothed path, smoothing computation time, length of smoothed path"<<endl;
      for (int index = 0; index<1; index++){
        cout<<endl<<"The RRT experiment "<<index<<" stats are "<<endl;
        RRT* plan = new RRT(start, goal, goalBias, 200000, step, GetEnv());
        string status = plan->buildRRTConnect(outdata);
        if (status == "Failure"){
          index--;
        }
      }
    }

    void graphBuilder(vector<double>  start, double step, vector<double> goal){
      ofstream outdata;
    	outdata.open("/home/abhishek/workspace/rrt/Data/rubbish.txt");
      vector<float> compTimes;
      for (double goalBias = 0.01; goalBias<1.0; goalBias= goalBias+0.05){
        vector<float> compTimeSingleBias;
        for(int i=0; i<5; i++){
          RRT* plan = new RRT(start, goal, goalBias, 200000, step, GetEnv());
          clock_t timeStart = clock();
          string status = plan->buildRRTConnect(outdata);
          float diff = float(clock()-timeStart)/CLOCKS_PER_SEC;
          if (status == "Reached")
          {
            compTimeSingleBias.push_back(diff);
          }
          else{
            compTimeSingleBias.push_back(diff);
          }
          // delete plan;
        }
        float averageCompTime = accumulate(compTimeSingleBias.begin(), compTimeSingleBias.end(), 0.0)/compTimeSingleBias.size();
        cout<<compTimeSingleBias.size();
        compTimes.push_back(averageCompTime);
      }
      cout<<endl;
      for (int index = 0; index<compTimes.size(); index++){
        cout<<compTimes[index]<<"  ";
      }
    }

    bool MyCommand(std::ostream& sout, std::istream& sinput)
    {
      string input;
      vector<double> start, goal;
      char inputChar = '!';
      sinput >> input;
      double value;
      double goalBias;
      double step;
      if (input=="goal")
      {
      	while(inputChar!=';')
      	{
  				sinput >> value;
  				goal.push_back(value);
  				sinput >> inputChar;
  			}
      	sinput >> input;
      }

      if (input == "goalBias"){
      	inputChar = '!';
      	while(inputChar!=';')
  			{
  				sinput >> value;
  				goalBias = value;
  				sinput >> inputChar;
  			}
      	sinput >> input;
      }
      if (input == "step")
      {
      	inputChar = '!';
  			while(inputChar!=';')
  			{
  				sinput >> value;
  				step = value;
  				sinput >> inputChar;
  			}
  			sinput >> input;
      }

      if (input == "start"){
      	inputChar = '!';
  			while(inputChar!=';')
  			{
  				sinput >> value;
  				start.push_back(value);
  				sinput >> inputChar;
  			}
      }
      //For RRT
      // graphBuilder(start, step, goal);
      if (goalBias != 0.0){
        generateStatistics(start, step, goalBias, goal);
      }
      else{
        //For buildBiRRTConnect
        cout<<"Running the BiDirectional RRT\n";
        RRT* plan = new RRT(start, goal, 0.0, 200000, step, GetEnv());
        string status = plan->buildBiRRTConnect();
      }
      return true;
    }
};



// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "planner" ) {
        return InterfaceBasePtr(new planner(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("planner");

}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
