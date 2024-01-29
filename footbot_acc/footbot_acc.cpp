/* Include the controller definition */
#include "footbot_acc.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

void CFootBotFlocking::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/

void CFootBotFlocking::SFlockingInteractionParams::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "target_distance", TargetDistance);
      GetNodeAttribute(t_node, "stabilization", Stabilization);
      GetNodeAttribute(t_node, "stabilization_timer", StabilizationTimer);
      GetNodeAttribute(t_node, "gamma_tolerance", GammaTolerance);
      GetNodeAttribute(t_node, "gain", Gain);
      GetNodeAttribute(t_node, "beta_gain", BetaGain);
      GetNodeAttribute(t_node, "swarm_size", SwarmSize);
      GetNodeAttribute(t_node, "cutoff", CutOff);
      GetNodeAttribute(t_node, "noise", NoiseLevel);
      GetNodeAttribute(t_node, "noise_deg", NoiseDegLevel);
      GetNodeAttribute(t_node, "cognitive_speed", CognitiveSpeed);
      GetNodeAttribute(t_node, "seed", Seed);
      GetNodeAttribute(t_node, "scale", Scale);
      GetNodeAttribute(t_node, "target_gain", TargetGain);
      GetNodeAttribute(t_node, "consensus", ConsensusOn);
      GetNodeAttribute(t_node, "gabor_p", GaborP);
      GetNodeAttribute(t_node, "gabor_a", GaborA);
      GetNodeAttribute(t_node, "_2_c_a", _2_c_a);
      GetNodeAttribute(t_node, "_2_c_r", _2_c_r);
      GetNodeAttribute(t_node, "_2_l_r", _2_l_r);
      GetNodeAttribute(t_node, "_2_l_r", _2_l_a);
      GetNodeAttribute(t_node, "_2_c_ap", _2_c_ap);
      GetNodeAttribute(t_node, "_LJ_a", _LJ_a);
      GetNodeAttribute(t_node, "_LJ_b", _LJ_b);
      GetNodeAttribute(t_node, "_10_C", _10_C);
      GetNodeAttribute(t_node, "_10_Sigma", _10_Sigma);
      GetNodeAttribute(t_node, "_10_h", _10_h);
      GetNodeAttribute(t_node, "_10_delta_l", _10_delta_l);
      GetNodeAttribute(t_node, "_10_delta_r", _10_delta_r);
      GetNodeAttribute(t_node, "_10_k", _10_k);
      GetNodeAttribute(t_node, "_12_a", _12_a);
      GetNodeAttribute(t_node, "_12_b", _12_b);
      GetNodeAttribute(t_node, "_12_c", _12_c);
      GetNodeAttribute(t_node, "_16_a", _16_a);
      GetNodeAttribute(t_node, "_16_b", _16_b);
      GetNodeAttribute(t_node, "_16_c1", _16_c1);
      GetNodeAttribute(t_node, "_16_c2", _16_c2);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller flocking parameters.", ex);
   }
}

/****************************************/
/****************************************/

CFootBotFlocking::CFootBotFlocking() :
   m_pcWheels(NULL),
   m_pcRABAct(NULL),
   m_pcRABSens(NULL),
   m_pcCamera(NULL) {}

/****************************************/
/****************************************/

void CFootBotFlocking::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the XML tag of the
    * device whose handle we want to have. For a list of allowed values, type at the
    * command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors internally, on the basis of
    *       the lists provided the configuration file at the
    *       <controllers><footbot_diffusion><actuators> and
    *       <controllers><footbot_diffusion><sensors> sections. If you forgot to
    *       list a device in the XML and then you request it here, an error occurs.
    */
   m_pcWheels  = GetActuator<CCI_DifferentialSteeringActuator >("differential_steering");
   m_pcRABAct  = GetActuator<CCI_RangeAndBearingActuator      >("range_and_bearing" );
   m_pcRABSens = GetSensor  <CCI_RangeAndBearingSensor        >("range_and_bearing" );
   m_pcCamera = GetSensor  <CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor>("footbot_proximity");


   /*
    * Parse the config file
    */
   try {
      /* Wheel turning */
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
      /* Flocking-related */
      m_sFlockingParams.Init(GetNode(t_node, "flocking"));
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
   }

   Reset();
}

/****************************************/
/****************************************/

UInt8 CFootBotFlocking::swarmConnection() {
   const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
   if (!tMsgs.empty()) {
      UInt8 max_id = 0;
      for(size_t i = 0; i < tMsgs.size(); ++i) {
         if (tMsgs[i].Data[8] >= Msg::CONNECTED) {
            if (STATUS < STOPPED)
               STATUS = CONNECTED;
            return (UInt8)255;
         }

         if (tMsgs[i].Data[8] == Msg::CONNECTING) {
            STATUS = CONNECTING;
            if (tMsgs[i].Data[9] > max_id) {
               max_id = tMsgs[i].Data[9];
            }
         }
      }

      if (max_id + 1 == m_sFlockingParams.SwarmSize) {
         STATUS = CONNECTED;
         return (UInt8)255;
      }

      if (max_id == IDENTIFIER - 1)
         return IDENTIFIER;
      else
         return max_id;
   }

   return (UInt8)0;
}

void CFootBotFlocking::emitConsensus(CVector2& speed) {
   UInt8 bytes[2*sizeof(float)+2] = {0};
   RealsToBytes(speed.GetX(), speed.GetY(), bytes);
   /*UInt8 msg = swarmConnection();
   if (IDENTIFIER == 0 && msg != 255) {
      STATUS = CONNECTING;
      bytes[8] = (UInt8) STATUS;
      bytes[9] = 0;
   } else {
      bytes[8] = (UInt8) STATUS;
      bytes[9] = msg;
   }
*/
   CByteArray data = CByteArray(bytes, 2*sizeof(float)+2);
   m_pcRABAct->SetData(data);
}

void CFootBotFlocking::ControlStep() {
   if (m_sFlockingParams.CognitiveSpeed > 1)
      if (tick % m_sFlockingParams.CognitiveSpeed != 0) {
         SetWheelSpeedsFromVector(current_speed);
         tick++;
         return;
      }

   tick++;
   if (STATUS == STOPPED) {
      current_speed = CVector2();
      emitConsensus(current_speed);
      return;
   }

   CVector2 phi_alpha = UiAlpha();
   CVector2 phi_gamma = VectorToLight();
   CVector2 phi_beta = UiBeta();

   CVector2 consensus = stabilizer*AlphaConsensus();

   old_phi_gamma = phi_gamma;

   current_displacement = phi_alpha + consensus + phi_gamma - stabilizer*current_speed + phi_beta;

   current_speed += current_displacement;

   if (current_speed.Length() > 0.0)
      current_speed = current_speed.Normalize()*m_sWheelTurningParams.MaxSpeed;
   else
      current_speed = CVector2();

   current_displacement = CVector2(GetMinObstacleDistance(), 0);
   
   emitConsensus(current_speed);
   SetWheelSpeedsFromVector(current_speed);
}


void CFootBotFlocking::Reset() {
   m_pcCamera->Enable();
   IDENTIFIER = (UInt8) std::stoi(m_strId.substr(2));
   timer = m_sFlockingParams.StabilizationTimer;
   SetWheelSpeedsFromVector(CVector2(0,0));
   max_phi_gamma = 1.25*sqrt(m_sFlockingParams.SwarmSize)*m_sFlockingParams.TargetDistance;
   state = VectorToLight()/m_sFlockingParams.TargetGain;
}

/****************************************/
/****************************************/

CVector2 CFootBotFlocking::VectorToLight() {
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& tReadings = m_pcCamera->GetReadings();
   if(!tReadings.BlobList.empty()) {
      CVector2 toTarget = CVector2();
      for(size_t i = 0; i < tReadings.BlobList.size(); ++i) {
         toTarget += CVector2(m_sFlockingParams.PhiGamma(computeNoise(tReadings.BlobList[i]->Distance)), computeDegNoise(tReadings.BlobList[i]->Angle));
      }
      return toTarget*m_sFlockingParams.TargetGain;
   }

   return CVector2();
}

/****************************************/
/****************************************/

CVector2 CFootBotFlocking::CVector2FromBytes(const UInt8* byteArray) {
    float first, second;
    std::memcpy(&first, byteArray, sizeof(float));
    byteArray += sizeof(float);
    std::memcpy(&second, byteArray, sizeof(float));

    return CVector2((Real)first, (Real)second);
}

void CFootBotFlocking::RealsToBytes(Real first, Real second, UInt8* byteArray) {
    float a = static_cast<float>(first);
    float b = static_cast<float>(second);

    std::memcpy(byteArray, &a, sizeof(float));
    byteArray += sizeof(float);
    std::memcpy(byteArray, &b, sizeof(float));
}


//LINEAR
Real CFootBotFlocking::SFlockingInteractionParams::PhiGamma(Real z) {
   return TargetGain*z;
}

/*
// ACTION FUNCTION [2] -- GAUSSIAN POTENTIAL
Real CFootBotFlocking::SFlockingInteractionParams::PhiAlpha(Real z) {
   z = z/TargetDistance;

   return Gain*((2*_2_c_r*z/(_2_l_r*_2_l_r))*pow(M_E, -(z*z/(_2_l_r*_2_l_r))) - (2*_2_c_a*z/(_2_l_a*_2_l_a))*pow(M_E, -(z*z/(_2_l_a*_2_l_a))) - 2*_2_c_ap*z);
}
*/

/*
// ACTION FUNCTION - LENNARD JONES 
Real CFootBotFlocking::SFlockingInteractionParams::PhiAlpha(Real z) {
   z = z/TargetDistance;
   Real target_dist = TargetDistance/100;
   return Gain*(_LJ_a/target_dist)*(pow(target_dist/z, _LJ_a+1) - pow(target_dist/z, _LJ_b+1));
}



// MY OWN
Real CFootBotFlocking::SFlockingInteractionParams::PhiAlpha(Real z) {
   z = z/TargetDistance;
   return Gain*(-atan(CutOff*z - CutOff) + tanh(CutOff*z - CutOff))/(pow(z, GaborA));
}


*/
// [6] - PIECE-WISE EXPONENTIAL
Real CFootBotFlocking::SFlockingInteractionParams::PhiAlpha(Real z) { 
   if (z <= TargetDistance)
      return Gain*pow(M_E, -z/100);
   else
      return -GaborP/(TargetDistance/100)*pow(M_E, -z/100);
}
/*



// [10] PIECE-WISE MARR WAVELET - PARABOLIC
Real CFootBotFlocking::SFlockingInteractionParams::PhiAlpha(Real z) { 
   Real d = TargetDistance/100; 
   z = z/TargetDistance;
   if (z < d-_10_delta_l) 
      return -Gain*_10_C*pow(M_E, -z*z/(2*_10_Sigma*_10_Sigma))*(z*z -(_10_h + 2)*_10_Sigma*_10_Sigma)/(2*M_PI*pow(_10_Sigma, 5));
   else if (z >= d-_10_delta_l && z <= d + _10_delta_r)
      return 0;
   else
      return -2*Gain*_10_k*z;
}




// [12] GAUSSIAN PARABOLIC
Real CFootBotFlocking::SFlockingInteractionParams::PhiAlpha(Real z) { 
   z = (z-TargetDistance)/TargetDistance;
   return -Gain*z*(_12_a - _12_b*pow(M_E, -z*z/_12_c));
}
*/




CVector2 CFootBotFlocking::UiAlpha() {
   const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();

   if(!tMsgs.empty()) {
      CVector2 phi_alpha;
      Real magnitude;
      neighbors = 0;
      for(size_t i = 0; i < tMsgs.size(); ++i) {
         neighbors++;
         magnitude = m_sFlockingParams.PhiAlpha(computeNoise(tMsgs[i].Range));
         phi_alpha += CVector2(magnitude, computeDegNoise(tMsgs[i].HorizontalBearing));
      }

      return -1*phi_alpha/tMsgs.size();
   }

   return CVector2();
}

Real CFootBotFlocking::GetMinObstacleDistance() {
   const CCI_FootBotProximitySensor::TReadings& tReadings = m_pcProximity->GetReadings();
   if(!tReadings.empty()) {
      Real max = 0;
      for(size_t i = 0; i < tReadings.size(); ++i) {
         if (max < tReadings[i].Value)
            max = tReadings[i].Value;
      }
      return max;
   }

   return 0;
}

CVector2 CFootBotFlocking::UiBeta() {
   const CCI_FootBotProximitySensor::TReadings& tReadings = m_pcProximity->GetReadings();
   if(!tReadings.empty()) {
      CVector2 phi_beta;
      Real magnitude;
      for(size_t i = 0; i < tReadings.size(); ++i) {
         magnitude = m_sFlockingParams.BetaGain*tReadings[i].Value;
         phi_beta += CVector2(magnitude, tReadings[i].Angle);
      }
      return -1*phi_beta/tReadings.size();
   }

   return CVector2();
}

CVector2 CFootBotFlocking::AlphaConsensus() {
   if (!m_sFlockingParams.ConsensusOn)
      return CVector2();

   const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();

   if(!tMsgs.empty()) {
      CVector2 consensus;
      for(size_t i = 0; i < tMsgs.size(); ++i) {
         consensus += (CVector2FromBytes(tMsgs[i].Data.ToCArray()) - current_speed);
      }
      return consensus/tMsgs.size();
   }

   return CVector2();
}

// 0,0 -- 1%,2deg -- 1.5%, 5 (default) -- 5%,10deg -- 20%, 20deg
// packet drop rate = 0.1

// cognitive speed = 20 (drop), 10 (drop), 5 (drop), 1, 0.5, 0.25 

Real CFootBotFlocking::computeNoise(Real scalar) {
   std::default_random_engine generator(m_sFlockingParams.Seed);
   std::normal_distribution<Real> distribution(scalar, scalar*m_sFlockingParams.NoiseLevel);
   return distribution(generator);
}

CRadians CFootBotFlocking::computeDegNoise(CRadians degrees) {
   std::default_random_engine generator(m_sFlockingParams.Seed);
   std::normal_distribution<Real> distribution(ToDegrees(degrees).GetValue(), m_sFlockingParams.NoiseDegLevel);
   return ToRadians(CDegrees(distribution(generator)));
}

/****************************************/
/****************************************/

void CFootBotFlocking::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   /* State transition logic */
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }
   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this controller.
 * When ARGoS reads that string in the XML file, it knows which controller class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotFlocking, "footbot_acc_controller")
