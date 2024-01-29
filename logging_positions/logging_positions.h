#ifndef LOGGING_POSITIONS_H
#define LOGGING_POSITIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <fstream>

using namespace argos;

class CLoggingPositions : public CLoopFunctions {
   
public:
   virtual ~CLoggingPositions() {}

   virtual void Init(TConfigurationNode& t_tree);

   virtual void Reset();

   virtual void PostStep();

private:

    std::ofstream log_file;
    CVector3 center;
    Real time;
    UInt32 length;
    bool halve;
    UInt32 tick = 0;
    Real max_obs_dist = 0;
};

#endif
