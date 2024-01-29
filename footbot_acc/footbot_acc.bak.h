/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example flocking controller for the foot-bot.
 *
 * This controller lets a group of foot-bots flock in an hexagonal lattice towards
 * a light source placed in the arena. To flock, it exploits a generalization of the
 * well known Lennard-Jones potential. The parameters of the Lennard-Jones function
 * were chosen through a simple trial-and-error procedure on its graph.
 *
 * This controller is meant to be used with the XML file:
 *    experiments/flocking.argos
 */

#ifndef FOOTBOT_ACC_H
#define FOOTBOT_ACC_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the range-and-bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range-and-bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Vector2 definitions */
#include <argos3/core/utility/math/vector2.h>
#include <set>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotFlocking : public CCI_Controller {

public:

   /*
    * The following variables are used as parameters for
    * turning during navigation. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_flocking_controller><parameters><wheel_turning>
    * section.
    */
   struct SWheelTurningParams {
      /*
       * The turning mechanism.
       * The robot can be in three different turning states.
       */
      enum ETurningMechanism
      {
         NO_TURN = 0, // go straight
         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
         HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism;
      /*
       * Angular thresholds to change turning state.
       */
      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      /* Maximum wheel speed */
      Real MaxSpeed;

      void Init(TConfigurationNode& t_tree);
   };

   /*
    * The following variables are used as parameters for
    * flocking interaction. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_flocking_controller><parameters><flocking>
    * section.
    */
   struct SFlockingInteractionParams {
      /* Target robot-robot distance in cm */
      Real TargetDistance;
      /* Gain of the Lennard-Jones potential */
      Real Gain;
      /* Exponent of the Lennard-Jones potential */
      Real Scale;
      Real Stabilization;
      Real TargetGain;
      Real GaborP;
      Real GaborA;
      bool ConsensusOn;
      Real CutOff;
      /* MORSE POTENTIAL [2]*/
      Real _2_c_a;
      Real _2_c_r;
      Real _2_l_r;
      Real _2_l_a;
      Real _2_c_ap;

      /* LENNARD JONES */
      Real _LJ_a;
      Real _LJ_b;

      /* [10] MARR-WAVELET + PARABOLIC */
      Real _10_C;
      Real _10_Sigma;
      Real _10_h;
      Real _10_delta_l;
      Real _10_delta_r;
      Real _10_k;

      /* [12] EXPONENTIAL */
      Real _12_a;
      Real _12_b;
      Real _12_c;

      /* [16] HYPERBOLIC */
      Real _16_a;
      Real _16_b;
      Real _16_c1;
      Real _16_c2;

      void Init(TConfigurationNode& t_node);
      Real PhiAlpha(Real f_distance);
      Real PhiGamma(Real z);
   };

public:

   /* Class constructor. */
   CFootBotFlocking();

   /* Class destructor. */
   virtual ~CFootBotFlocking() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML file
    * in the <controllers><footbot_flocking_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything, so
    * the function could have been omitted. It's here just for completeness.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up, so
    * the function could have been omitted. It's here just for completeness.
    */
   virtual void Destroy() {}

protected:

   /*
    * Calculates the vector to the closest light.
    */
   virtual CVector2 VectorToLight();
   /*
    * Calculates the flocking interaction vector.
    */
   virtual CVector2 UiAlpha();
   virtual CVector2 AlphaConsensus();
   /*
    * Gets a direction vector as input and transforms it into wheel actuation.
    */

   Real Bump(Real value);
   bool allNeighborsHaveStopped();
   void SetWheelSpeedsFromVector(const CVector2& c_heading);
   bool checkConvergence();

   void emitConsensus();

   CVector2 CVector2FromBytes(const UInt8* byteArray);
   void RealsToBytes(Real first, Real second, UInt8* byteArray);

private:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the range-and-bearing actuator */
   CCI_RangeAndBearingActuator* m_pcRABAct;
   /* Pointer to the range-and-bearing sensor */
   CCI_RangeAndBearingSensor* m_pcRABSens;
   /* The turning parameters. */
   SWheelTurningParams m_sWheelTurningParams;
   /* The flocking interaction parameters. */
   SFlockingInteractionParams m_sFlockingParams;
   /* Pointer to the omnidirectional camera sensor */
   CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;
   CCI_ProximitySensor* m_pcProximity;

   Real max_no_update_time;
   Real max_displacement;
   Real remaining_displacement = 0.0f;
   UInt8 IDENTIFIER;
   std::set<UInt8> neighbors;
   Real stabilizer = 1;
   int convergenceTimer = 0;
   CVector2 old_phi_gamma;

   int counter = 0;

   enum Status {
      FLOCKING,
      STOPPING
   };

   Status STATUS = FLOCKING;
   //CVector2 current_speed;
   //CONSENSUS SU STOP: -SE I VICINI SONO FERMI E IO SONO FERMO ALLORA ALZO LA MANO
   //SE CAMBIA IL NUMERO DI VICINI O SI MUOVONO, LA RIABBASSO
   //SE TUTTI I VICINI HANNO LA MANO ALZATA, MI POSSO FERMARE
};

#endif
