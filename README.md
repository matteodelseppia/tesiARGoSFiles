# tesiARGoSFiles

1. First, download and install [ARGoS3](https://www.argos-sim.info/index.php).
2. Then, download the folder containing the [argos3-examples](https://www.argos-sim.info/examples.php). Put it in a path of your choice, called here ``$EXAMPLES_PATH``.
3. Copy the content of ``CMakeLists1.txt`` from this repository and paste it into ``$EXAMPLES_PATH/controllers/CMakeLists.txt``.
4. Download the ``footbot_acc`` folder from this repository and put it into ``$EXAMPLES_PATH/controllers/``. This contains the code of the robots controller. In the controller code, you can comment/uncomment the particular artificial potential field you want to use. 
5. Copy the content of ``CMakeLists2.txt`` from this repository and paste it into ``$EXAMPLES_PATH/loop_functions/CMakeLists.txt``.
6. Download the ``logging_positions`` folder from this repository and put it into ``$EXAMPLES_PATH/loop_functions/``.
7. Download the ``manual_test.argos`` file from this repository and put it into ``$EXAMPLES_PATH/experiments/``. This is the configuration to run visual experiments. It already constains some parameters for the artificial potential fields implemented in the controller code. The parameters are read by the controller directly from this XML configuration, so you can make some changes to see how the potential behaves with different settings.
8. Download the ``experiment.argos`` file from this repository and put it into ``$EXAMPLES_PATH/experiments/``. This is the configuration to run the experiments, without visual interface.
9. Download the ``flocking_exp_LJ.argos`` file from this repository and put it into ``$EXAMPLES_PATH/experiments/``. This is the configuration to run the simulations employed by the genetic algorithm.
10. Download and ``genetic_LJ.py`` ``manual_eval.py`` and put it into ``$EXAMPLES_PATH``. They are respectively the Python code of the genetic algorithm and a file containing some code used to evaluate the performance of the individuals of the genetic algorithm.
11. Download ``compile.sh`` and put it into ``$EXAMPLES_PATH``. 

### Running visual simulations
To run a visual simulation, just move into ``$EXAMPLES_PATH`` and run ``./compile.sh``. This should compile the code of your controller. Then run ``argos3 -c experiments/manual_test.argos``. 

### Running genetic algorithm for Lennard-Jones
The genetic algorithm is run in a multi-threaded fashion with Pymoo. You should just run ``python genetic_LJ.py [pop_size]`` where "pop_size" is a number representing the size of your population. This code particularly bloats the folders with .argos files generated to run simulations, so you can use the ``clean.sh`` script in this repo to clean everything up once in a while.
If you need to run the genetic algorithm for another potential, just copy and paste the configuration file ``flocking_LJ.argos`` and ``genetic_LJ.py`` with different names into the respective folders. In the Python code, you have to change the parameters plugged into the configuration file (look at the code, it's easy to understand). 

### Running experiments
You can use the code in ``run_exp.py`` to run the experiments. The code runs by default a set of experiments with different values of noise and density, for all the potentials implemented in the controller.
