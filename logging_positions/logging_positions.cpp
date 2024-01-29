#include "logging_positions.h"
#include <iostream>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include <string>
#include <ctime>
#include <cmath>

void CLoggingPositions::Init(TConfigurationNode &t_tree)
{
  char file_name[48];
  GetNodeAttribute(t_tree, "file", file_name);
  GetNodeAttribute(t_tree, "halve", halve);
  GetNodeAttribute(t_tree, "length", length);
  if (!log_file.is_open())
    log_file.open(strcat(file_name, ".experiment"), std::ios::app);

  time = 0;
  tick = 0;
}

/****************************************/
/****************************************/

void CLoggingPositions::Reset()
{
  log_file.close();
  log_file.clear();
}

/****************************************/
/****************************************/

void CLoggingPositions::PostStep() {
  if (halve)
    if (tick % 2 != 0) {
      tick++;
      return;
    }

  tick++;

  if (length > 0 && tick < length - 50)
    return;

  CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("foot-bot");

  int i = 0;
  for (CSpace::TMapPerType::iterator it = tFBMap.begin(); it != tFBMap.end(); ++it) {
    CFootBotEntity *pcFB = any_cast<CFootBotEntity *>(it->second);
    pcFB->GetId();
    CVector3 &pos = pcFB->GetEmbodiedEntity().GetOriginAnchor().Position;
    Real obs_dist = pcFB->GetControllableEntity().GetController().current_displacement.GetX();

    if (length > 0)
      log_file << (tick - (length - 50) + 1) << " " << pos.GetX() << " " << pos.GetY() << " " << obs_dist << std::endl;
    else {
      log_file << pos.GetX() << "," << pos.GetY();
      if (i + 1 < tFBMap.size()) {
        log_file << ",";
      } else {
        log_file << std::endl;
      }
    }

    i++;
  }

  log_file.flush();
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CLoggingPositions, "logging_positions")
