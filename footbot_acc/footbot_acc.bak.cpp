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
      GetNodeAttribute(t_node, "gain", Gain);
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

   /*
    * Other init stuff
    */
   Reset();
}

/****************************************/
/****************************************/

void CFootBotFlocking::emitConsensus() {
   UInt8 bytes[2*sizeof(float)+2] = {0};
   RealsToBytes(current_speed.GetX(), current_speed.GetY(), bytes);
   bytes[8] = (UInt8) STATUS;
   CByteArray data = CByteArray(bytes, 2*sizeof(float)+2);
   m_pcRABAct->SetData(data);
}

bool CFootBotFlocking::allNeighborsHaveStopped() {
   const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
   if(!tMsgs.empty()) {
      for(size_t i = 0; i < tMsgs.size(); ++i) {
         UInt8 neighborStatus = tMsgs[i].Data[8];
         if (neighborStatus == (UInt8) FLOCKING)
            return false;
      }
      return true;
   }

   return false;
}

void CFootBotFlocking::ControlStep() {
   CVector2 phi_alpha = UiAlpha();
   CVector2 phi_gamma = VectorToLight();
   CVector2 consensus = AlphaConsensus();

   if (abs(old_phi_gamma.Length() - phi_gamma.Length()) < 0.0001) {
      counter++;
      if (counter >= 100)
         STATUS = STOPPING;
   } else {
      counter = 0;
      STATUS = FLOCKING;
   }

   current_speed += phi_alpha + phi_gamma - current_speed + consensus;
   
   if (current_speed.Length() > 0.0)
      current_speed = current_speed.Normalize()*m_sWheelTurningParams.MaxSpeed;
   else 
      current_speed = CVector2();

   old_phi_gamma = phi_gamma;
   
   if (counter > 100 || allNeighborsHaveStopped()) {
      STATUS = STOPPING;
      if (allNeighborsHaveStopped())
         current_speed = CVector2();
   }

   emitConsensus();
   SetWheelSpeedsFromVector(current_speed);
}


void CFootBotFlocking::Reset() {
   m_pcCamera->Enable();
   SetWheelSpeedsFromVector(CVector2(0,0));
}

/****************************************/
/****************************************/

Real CFootBotFlocking::Bump(Real z) {
   Real x = z/(m_sFlockingParams.Scale*m_sFlockingParams.TargetDistance);
   if (x <= m_sFlockingParams.CutOff)
      return 1.0;
   if (x > m_sFlockingParams.CutOff && x < 1)
      return (0.5 + 0.5*cos(M_PI*(x-m_sFlockingParams.CutOff)/(1-m_sFlockingParams.CutOff)));
   
   return 0;
}

CVector2 CFootBotFlocking::VectorToLight() {
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& tReadings = m_pcCamera->GetReadings();
   if(!tReadings.BlobList.empty()) {
      CVector2 toTarget = CVector2();
      for(size_t i = 0; i < tReadings.BlobList.size(); ++i) {
         toTarget += CVector2(m_sFlockingParams.PhiGamma(tReadings.BlobList[i]->Distance), tReadings.BlobList[i]->Angle);
      }

      return toTarget;
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
//GABOR'S BRAKING FUNCTION
Real CFootBotFlocking::SFlockingInteractionParams::PhiGamma(Real z) {
   if (z <= 0) 
      return 0;
   
   if (z*GaborP > 0 && z*GaborP < GaborA/GaborP)
      return z*GaborP;
   else
      return sqrt(2*GaborA*z - pow(GaborA, 2)/pow(GaborP, 2));
}
*/

/*
// ACTION FUNCTION [2] -- A VERSION OF MORSE POTENTIAL
Real CFootBotFlocking::SFlockingInteractionParams::PhiAlpha(Real z) {
   z = z/TargetDistance;

   return Gain*((2*_2_c_r/(_2_l_r*_2_l_r))*pow(M_E, -(z*z/(_2_l_r*_2_l_r))) - (2*_2_c_a/(_2_l_a*_2_l_a))*pow(M_E, -(z*z/(_2_l_a*_2_l_a))) - 2*_2_c_ap*z);
}
*/



/*
// ACTION FUNCTION - LENNARD JONES 
Real CFootBotFlocking::SFlockingInteractionParams::PhiAlpha(Real z) {
   z = z/TargetDistance;
   return Gain*(_LJ_a/TargetDistance)*(pow(1/z, _LJ_a+1) - pow(1/z, _LJ_b+1));
}
*/

/*
// INTEGRAL EXPONENTIAL POTENTIAL - MY OWN
Real CFootBotFlocking::SFlockingInteractionParams::PhiAlpha(Real z) {
   z = z/TargetDistance;
   return Gain*(pow(M_E, -z)/z - 1/M_E);
}
*/



// [6] - PIECE-WISE EXPONENTIAL
Real CFootBotFlocking::SFlockingInteractionParams::PhiAlpha(Real z) { 
   z = (z - TargetDistance)/TargetDistance;
   if (z <= TargetDistance)
      return Gain*pow(M_E, -z);
   else
      return -Gain/(TargetDistance/100)*pow(M_E, -z);
}


/*
// [10] PIECE-WISE MARR WAVELET - PARABOLIC
Real CFootBotFlocking::SFlockingInteractionParams::PhiAlpha(Real z) { 
   Real d = TargetDistance/100; 
   z = z/TargetDistance;
   if (z < d-_10_delta_l) 
      return -(Gain/(M_PI*pow(_10_Sigma, 5)))*(pow(M_E, -(z/pow(_10_Sigma, 2))))*(z*z - (_10_h + 1)*_10_Sigma*_10_Sigma);
   else if (z >= d-_10_delta_l && z <= d + _10_delta_r)
      return 0;
   else
      return -2*Gain*_10_k*z*z;
}
*/

/*
// [12] EXPONENTIAL
Real CFootBotFlocking::SFlockingInteractionParams::PhiAlpha(Real z) { 
   Real d = TargetDistance/100; 
   z = z/TargetDistance;
   return -Gain*z*(_12_a - _12_b*pow(M_E, -z*z/_12_c));
}
*/

/*
// [16] HYPERBOLIC
Real CFootBotFlocking::SFlockingInteractionParams::PhiAlpha(Real z) {
   z = z/TargetDistance;
   return Gain*(-_16_c1*pow(z, -_16_a) + _16_c2*pow(z, - _16_b));
}
*/

CVector2 CFootBotFlocking::UiAlpha() {
   const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
   if(!tMsgs.empty()) {
      CVector2 phi_alpha;
      Real magnitude;
      for(size_t i = 0; i < tMsgs.size(); ++i) {
         magnitude = m_sFlockingParams.PhiAlpha(tMsgs[i].Range);
         phi_alpha += CVector2(magnitude, tMsgs[i].HorizontalBearing);
      }
      return -1*phi_alpha;
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
         consensus += (CVector2FromBytes(tMsgs[i].Data.ToCArray()) - m_sFlockingParams.Stabilization*current_speed);
      }
      return consensus;
   }

   return CVector2();
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
