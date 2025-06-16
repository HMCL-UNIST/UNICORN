<a href="http://arxiv.org/abs/2411.17508">
    <img src="https://img.shields.io/badge/arXiv.org-2411.17508-b31b1b" alt="arXiv e-print Badge">
</a>

On-track system identification is a data-driven approach to identifying tire-dynamics by simply driving around. Check out our preprint [Learning-Based On-Track System Identification for Scaled Autonomous Racing in Under a Minute](https://arxiv.org/abs/2411.17508) on ArXiv for more information. Or check out our explanatory [Youtube video](https://www.youtube.com/watch?v=4kLpSiZoAsE&feature=youtu.be).

![On-Track SysID](./misc/graphicalabstract.png)

## Introduction

This node aims to identify the pacejka tire model of a vehicle with on-track data. It utilizes a neural network to learn model error in a nominal vehicle model. Combining the trained neural network and the nominal model, it generates steady state data and identifies Pacejka parameters from the generated data. This process is done iteratively to improve the vehicle model until the model converges.
The identified parameters are then used to generate a Look-Up Table (LUT) for the MAP controller.

## Installation
On-Track SysID is part of the [ForzaETH Race Stack](https://github.com/ForzaETH/race_stack). Please refer to the [installation guide](https://github.com/ForzaETH/race_stack/blob/main/INSTALLATION.md) for detailed instructions and perform the quickstart guide below to run the sysid procedure.

## Usage (Detailed)

To use this node, follow these steps:

1. **Modify Parameters**:
   - Ensure the vehicle's model parameters are correctly set in the file:
     `race_stack/system_identification/on_track_sys_id/src/models/(racecar_version)/(racecar_version)_pacejka.txt` 
     
     The required parameters are:

        - `I_z`: Moment of inertia about the vehicle's vertical axis (kg.m^2).
        - `h_cg`: Height of the vehicle's center of gravity (m).
        - `l_f`: Distance from the center of gravity to the front axle (m).
        - `l_r`: Distance from the center of gravity to the rear axle (m).
        - `l_wb`: Wheelbase of the vehicle (m).
        - `m`: Mass of the vehicle (kg).
        - `model_name`: Name of the vehicle model (e.g., NUC5).
        - `tire_model`: Type of tire model used (e.g., pacejka).

        Example:
        ```yaml
        I_z: 0.0627
        h_cg: 0.02
        l_f: 0.165
        l_r: 0.155
        l_wb: 0.32
        m: 3.7
        model_name: NUC5
        tire_model: pacejka
        ```
        (Identified pacejka parameters will also be written to this file after training, the pacejka parameters that exist here prior to training are not important.)
    - (Optional) Inside `race_stack/system_identification/on_track_sys_id/src/params/nn_params.yaml` file, you can adjust the following parameters:
    
        - `data_collection_duration` (IMPORTANT): Duration of data collection in seconds. The default is 50 seconds, which is sufficient in most cases. It can be increased more if time is not limited.
        - `num_of_iterations`: Number of iterations for training. The default is 6 iterations. If the model is not converging after 6th iteration, it can be increased.
        - `num_of_epochs`: Number of epochs for training at each iteration. The default is 100 epochs.
        - `lr`: Learning rate for training. The default is 0.0005.
        - `weight_decay`: Weight decay parameter. The default is 0.0.

    - (Optional) Inside `race_stack/system_identification/on_track_sys_id/src/params/pacejka_params.yaml` file, you can adjust the following parameters:

        - `pacejka_model`: Initial values for the Pacejka tire model parameters. These can be left as they are, as convergence does not depend on initial parameters. However, you can adjust them if you have an initial guess for the parameters.
        - `pacejka_ref`: Reference Pacejka parameters are not used in training but plotted in figures for comparison purposes. It can help evaluate the identified Pacejka model. For example, if the surface you want to identify the Pacejka model on has less grip than a known surface, you would expect the identified Pacejka model to have smaller lateral forces, etc.

2. **Launch the Node**: After launchning base_system.launch and time_trials.launch, execute the following command to launch the node along with the required parameters:

    ```bash
    roslaunch stack_master sys_id.launch save_LUT_name:=<save_LUT_name> plot_model:=<True/False>
    ```

    - The `save_LUT_name` parameter specifies the name of the Lookup Table (LUT) that will be saved. By default, it's set to "NUCx_on_track_pacejka".
    - The `plot_model` parameter specifies whether to plot the model evolution at each iteration. If set to `False`, only the Pacejka model in the last iteration will be plotted. If `True`, the identified Pacejka model will be plotted at the end of each iteration.

3. **Drive the Car**: Drive the car at its maximum possible speed without collision to collect data. Driving at its maximum speed will provide data with a high range of slip angles, so the identified pacejka model will be better also at higher slip angles.

4. **Monitor the Progress**: 
    - Launching the node will start data collection for the specified amount of time (only saves the data when longitudinal velocity is higher than 1m/s). 
    - The terminal will display the progress of data collection and training. Follow the instructions on the screen to proceed. 
    - You will be prompted to press 'Y' to export the collected data.

5. **Move the Lookup Table**: 
    - After identifying Pacejka model parameters, a LUT will automatically be generated in `race_stack/system_identification/on_track_sys_id/src/models/(racecar_version)/`
    - Move this lookup table into `race_stack/system_identification/steering_lookup/cfg/`.
    - New LUT can be used after relaunching time trials with the new LUT name.
6. (OPTIONAL) **Repeat**: Repeat all the steps if you can drive the car faster than before with the new generated LUT and you think going even faster is possible with a better model. This would presumably provide a better model that covers higher ranges of slip angles more accurately.

## Usage (Short)
 - Make sure `race_stack/system_identification/on_track_sys_id/src/models/(racecar_version)/(racecar_version)_pacejka.txt` exist with correct parameters.

 - Launch sys_id.launch
  ```bash
  roslaunch stack_master sys_id.launch save_LUT_name:=<save_LUT_name> plot_model:=<True/False>
  ```
 - Drive the car with a controller until data collection is done.
 - Move generated lookup table to `race_stack/system_identification/steering_lookup/cfg/`. 
 - Relaunch time trials.

## Files and Directory Structure

- `on_track_sys_id.py`: This is the main Python script that contains the main node responsible for collecting data, calling training, exporting data, and more.

- `nn_train.py`: This script handles neural network training. It is called from the main script. It trains a neural network, identifies Pacejka parameters, and calls `simulate_model.py` to generate the Look-Up Table (LUT).

- `simulate_model.py`: This script generates a Look-Up Table (LUT) from the identified Pacejka model parameters.

- `dynamics/`: This directory contains vehicle dynamics used for Look-Up Table (LUT) generation.

- `data/`: This directory stores collected data if exported.

- `model/`: Directory containing vehicle models.

- `params/`: Directory containing YAML files for model parameters.

- `helpers/`: Directory containing helper functions and modules.
    - `pacejka_formula.py`: Module to calculate lateral tire forces using Pacejka tire model.
    - `plot_results.py`: Module to plot the system identification results.
    - `generate_inputs_errors.py`: Module to generate input tensors and target error tensors for neural network training.
    - `generate_predictions.py`: Module to generate predictions for the next step's lateral velocity and yaw rate.
    - `load_model.py`: Module to load vehicle model params from `model/`.
    - `save_model.py`: Module to save pacejka model parameters to corresponding txt files in `model/`.
    - `SimpleNN.py`: Module defining the structure of the neural network.
    - `solve_pacejka.py`: Module to solve for Pacejka tire model coefficients.

## Citing On Track SysID

If you found our stack helpful in your research, we would appreciate if you cite it as follows:
```
@misc{dikici2024learningbasedontrackidentificationscaled,
      title={Learning-Based On-Track System Identification for Scaled Autonomous Racing in Under a Minute}, 
      author={Onur Dikici and Edoardo Ghignone and Cheng Hu and Nicolas Baumann and Lei Xie and Andrea Carron and Michele Magno and Matteo Corno},
      year={2024},
      eprint={2411.17508},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2411.17508}, 
}
```
