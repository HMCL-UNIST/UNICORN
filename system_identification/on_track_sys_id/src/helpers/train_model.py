import os
import numpy as np
import matplotlib.pyplot as plt
import torch
import torch.nn as nn
import yaml
import shutil
from scipy.signal import butter, filtfilt
from torch.optim import Adam
from helpers.generate_predictions import generate_predictions
from helpers.generate_inputs_errors import generate_inputs_errors
from helpers.SimpleNN import SimpleNN
from helpers.pacejka_formula import pacejka_formula
from helpers.plot_results import plot_results
from helpers.solve_pacejka import solve_pacejka
from helpers.save_model import save
from helpers.load_model import get_dotdict
from helpers.simulate_model import LookupGenerator
import rospy
import rospkg
from tqdm import tqdm

def filter_data(training_data, model):
    """
    Filter training data.

    Applies a low-pass Butterworth filter to each column of the training data array.
    """
    b, a = butter(N=3, Wn=0.1, btype='low')
    training_data[:,0] = filtfilt(b, a, training_data[:,0]) # v_x, longitudinal velocity
    training_data[:,1] = filtfilt(b, a, training_data[:,1]) # v_y, lateral velocity
    training_data[:,2] = filtfilt(b, a, training_data[:,2]) # omega, yaw rate
    training_data[:,3] = filtfilt(b, a, training_data[:,3]) # delta, steering angle
    training_data[:,4] = filtfilt(b, a, training_data[:,4]) # long_acc
    training_data[:,3] = np.roll(training_data[:,3], 5)
    # training_data[:,4] = np.roll(training_data[:,4], 5)

    training_data = training_data[5:,:]
    
    # If the model is not a simulation, adjusts lateral velocity based on the car's rear axle length.
    if model["racecar_version"] != "SIM":
        training_data[:,1] = training_data[:,1] + model["l_r"] * training_data[:,2]
    
    return training_data

def negate_data(training_data):
    """
    Negate training data along the y-axis.

    Negates the lateral velocity (v_y), yaw rate (omega) and steering angle (delta) components of the training data
    to generate additional training samples with the opposite direction.
    """
    negate_data = np.zeros((2*training_data.shape[0], training_data.shape[1]))
    negate_data[:,0] = np.append(training_data[:,0], training_data[:,0])
    negate_data[:,1] = np.append(training_data[:,1], -training_data[:,1])
    negate_data[:,2] = np.append(training_data[:,2], -training_data[:,2])
    negate_data[:,3] = np.append(training_data[:,3], -training_data[:,3])
    negate_data[:,4] = np.append(training_data[:,4], training_data[:,4])

    return negate_data

def process_data(training_data, model):
    """
    Process training data.
    Filters and negates the training data to prepare it for training the neural network.
    """
    filtered_data = filter_data(training_data, model)
    negated_data = negate_data(filtered_data)
    
    return negated_data

def simulated_data_gen(nn_model, model, avg_vel):
    C_Pf_model = model['C_Pf_model']
    C_Pr_model = model['C_Pr_model']

    l_f = model['l_f']
    l_r = model['l_r']
    l_wb = model['l_wb']
    m = model['m']
    I_z = model['I_z']
    h = model['h_cg']

    timesteps = 500 # Number of timesteps to simulate

    # F_zf = m * 9.81 * l_r / l_wb
    # F_zr = m * 9.81 * l_f / l_wb
    
    long_acc = -2 / (1 + np.exp(np.linspace(-4, 4, timesteps))) + 1
    long_acc *= -0.5
    long_acc = np.zeros(timesteps)
    
    dt = 0.02 # 0.02 for 50 Hz

    v_y = np.zeros(timesteps)  # Initial lateral velocity
    omega = np.zeros(timesteps)  # Initial yaw rate
    alpha_f = np.zeros(timesteps)  # Initial lateral velocity
    alpha_r = np.zeros(timesteps)  # Initial yaw rate
    
    v_x = np.ones(timesteps)*avg_vel  # Constant longitudinal velocity
    # mini_delta = np.concatenate([np.linspace(0.0, 0.4, int(timesteps/4)), -np.linspace(-0.4, 0.0, int(timesteps/4))])
    # delta = np.concatenate([mini_delta,mini_delta])
    delta = np.linspace(0.0, 0.4, timesteps)

    # Simulation loop
    for t in range(timesteps-1):
        F_zf = m * (-long_acc[t] * h + 9.81 * l_r) / l_wb
        F_zr = m * (long_acc[t] * h + 9.81 * l_f) / l_wb

        alpha_f[t] = -np.arctan((v_y[t] + omega[t] * l_f) / v_x[t]) + delta[t]
        alpha_r[t] = -np.arctan((v_y[t] - omega[t] * l_r) / v_x[t])

        # Calculate Pacejka lateral forces
        F_f = pacejka_formula(C_Pf_model, alpha_f[t], F_zf)
        F_r = pacejka_formula(C_Pr_model, alpha_r[t], F_zr)
        input = torch.tensor([v_x[t], v_y[t], omega[t], delta[t], long_acc[t]], dtype=torch.float32)

        # Making predictions
        with torch.no_grad():
            predicted_means = nn_model(input)
        # Update vehicle states using the dynamics equations
        v_x_dot = long_acc[t]
        v_y_dot = (1/m) * (F_r + F_f * np.cos(delta[t]) - m * v_x[t]* omega[t])
        omega_dot = (1/I_z) * (F_f * l_f * np.cos(delta[t]) - F_r * l_r)

        # Euler integration for the next state
        v_x[t+1] = v_x[t] + v_x_dot * dt
        v_y[t+1] = v_y[t] + v_y_dot * dt + predicted_means[0]
        omega[t+1] = omega[t] + omega_dot * dt + predicted_means[1]


    # plt.figure(figsize=(8, 5))
    # plt.title('V_Y')
    # plt.plot(v_y, label='Train Loss')
    # ax1.scatter(alpha_f, F_yf, color="Green", alpha=0.95, label='Generated Data', s=0.5 )

    # plt.show()

    # plt.title('omega')

    # plt.plot(omega, label='Train Loss')
    # plt.show()


    return v_x, v_y, omega, delta, long_acc

def get_model_param(racecar_version):
    """
    Retrieve model parameters for a given racecar version.
    Loads Pacejka tire and vehicle parameters from YAML files and constructs a model dictionary.
    
    Returns:
        dict: Model parameters including tire and vehicle properties.
    """

    ############## YH ##############
    # rospack = rospkg.RosPack()
    # package_path = rospack.get_path('on_track_sys_id')  # Replace with your package name
    # yaml_file = os.path.join(package_path, 'params/pacejka_params.yaml')
    # with open(yaml_file, 'r') as file:
    #     pacejka_params = yaml.safe_load(file)
        
    # # Load vehicle parameters
    # yaml_file = os.path.join(package_path, 'models', racecar_version, racecar_version + '_pacejka.txt')
    # with open(yaml_file, 'r') as file:
    #     vehicle_params = yaml.safe_load(file)

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('on_track_sys_id') 
    models_path = os.path.join(package_path, 'models')
    racecar_path = os.path.join(models_path, racecar_version)
    
    if not os.path.exists(racecar_path):
        os.makedirs(racecar_path)
        print(f"Created directory: {racecar_path}")
        
    pacejka_file = os.path.join(racecar_path, f'{racecar_version}_pacejka.txt')
    initial_pacejka_file = os.path.join(models_path, 'initial_pacejka.txt')
    
    # Check if the racecar version directory exists; if not, create it
    if not os.path.exists(racecar_path):
        os.makedirs(racecar_path)
        print(f"Created directory: {racecar_path}")
    
    # Check if the pacejka file exists; if not, copy from initial_pacejka.txt
    if not os.path.exists(pacejka_file):
        if os.path.exists(initial_pacejka_file):
            shutil.copy(initial_pacejka_file, pacejka_file)
            print(f"Created file: {pacejka_file} from initial_pacejka.txt")
        else:
            raise FileNotFoundError("initial_pacejka.txt not found. Please provide an initial template.")
    
    # Append model_name to the file if not already present
    with open(pacejka_file, 'a+') as file:
        file.seek(0)
        content = file.read()
        if "model_name:" not in content:
            file.write(f"\nmodel_name: {racecar_version}\n")
            print(f"Appended model_name: {racecar_version} to {pacejka_file}")
    
    # Load Pacejka parameters
    yaml_file = os.path.join(package_path, 'params/pacejka_params.yaml')
    with open(yaml_file, 'r') as file:
        pacejka_params = yaml.safe_load(file)
    
    # Load vehicle parameters
    with open(pacejka_file, 'r') as file:
        vehicle_params = yaml.safe_load(file)
    ############## YH ##############

    # Construct model dictionary
    model = {
    "C_Pf_model": pacejka_params['pacejka_model']['C_Pf_model'],
    "C_Pr_model": pacejka_params['pacejka_model']['C_Pr_model'],
    "C_Pf_ref": pacejka_params['pacejka_ref']['C_Pf_ref'],
    "C_Pr_ref": pacejka_params['pacejka_ref']['C_Pr_ref'],
    "m": vehicle_params['m'],
    "I_z": vehicle_params['I_z'],
    "l_f": vehicle_params['l_f'],
    "l_r": vehicle_params['l_r'],
    "l_wb": vehicle_params['l_wb'],
    "h_cg": vehicle_params['h_cg'],
    "racecar_version": racecar_version
    }
    return model

def get_nn_params():
    """
    Retrieve neural network parameters.

    Loads neural network parameters from a YAML file.

    Returns:
        dict: Neural network parameters.
    """
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('on_track_sys_id')  # Replace with your package name
    yaml_file = os.path.join(package_path, 'params/nn_params.yaml')
    with open(yaml_file, 'r') as file:
        nn_params = yaml.safe_load(file)
        
    return nn_params

def generate_training_set(training_data, model):
    """
    Generate training set for neural network training.

    Predicts the next step's lateral velocity and yaw rate using the vehicle model.
    Constructs input tensors and error tensors for training the neural network.

    Args:
        training_data (numpy.ndarray): Input training data with shape (n_samples, n_features).
        model (dict): Dictionary containing vehicle model parameters.

    Returns:
        tuple: Tuple containing input tensor and target error tensor for training.
    """
    
    # Generate predictions for the next step's lateral velocity and yaw rate
    v_y_next_pred, omega_next_pred = generate_predictions(training_data, model)
    
    # Generate input tensors and error tensors for training
    X_train, y_train = generate_inputs_errors(v_y_next_pred, omega_next_pred, training_data)
    
    return X_train, y_train

def nn_train(training_data, racecar_version, save_LUT_name, plot_model):
    """
    Train the neural network.
    
    Trains the neural network using the provided training data and model parameters.
    After training, it simulates the car behavior with the trained model and identifies
    Pacejka tire model coefficients. Then it iteratively refines the model and repeats
    the training process. Finally, it saves the trained model and generates a
    Look-Up Table (LUT) for the controller. 

    """
    # Get model and neural network parameters
    model = get_model_param(racecar_version)
    nn_params = get_nn_params()
    num_of_epochs = nn_params['num_of_epochs']
    lr = nn_params['lr']
    weight_decay = nn_params['weight_decay']
    num_of_iterations = nn_params['num_of_iterations']

    training_data = process_data(training_data, model)   
     
    avg_vel = np.mean(training_data[:,0]) # Defining average velocity for the simulation, NN will have more accurate predictions
    print("\n\nAvg_vel: ", avg_vel)
    
    #-----------------------------0331 hj------------------------------
    # avg_vel = np.clip(avg_vel, 2.75, 4)
    avg_vel = np.clip(avg_vel, 2.75, 6.5)
    #-----------------------------0331 hj------------------------------

    # Iterative training loop
    for i in range(1, num_of_iterations+1):
        if i == num_of_iterations: # Determine if it's the last iteration to enable plotting (if plot_model is False)
            plot_model = True
            
        # Process training data and generate inputs and targets
        X_train, y_train = generate_training_set(training_data, model)
        
        # Initialize the network
        nn_model = SimpleNN(weight_decay = weight_decay)

        # Loss and optimizer
        criterion = nn.MSELoss()
        optimizer = Adam(nn_model.parameters(), lr=lr)

        nn_model.train()
        pbar = tqdm(total=num_of_epochs, desc=f"Iteration: {i}/{num_of_iterations}, Epoch:", ascii=True)
        loss_history = []

        # Training loop
        for epoch in range(1, num_of_epochs+1):
            pbar.update(1)
            # Forward pass on training data
            outputs = nn_model(X_train)
            
            #-----------------------------0331 hj------------------------------
            # train_loss = criterion(outputs, y_train) # + nn_model.l2_regularization_loss() # TODO add regularization if needed
            train_loss = criterion(outputs, y_train) + nn_model.l2_regularization_loss() # TODO add regularization if needed
            #-----------------------------0331 hj------------------------------

            optimizer.zero_grad()
            train_loss.backward()
            optimizer.step()
            loss_history.append(train_loss.item())
            # If it's the last epoch, simulate car behavior and identify model coefficients
            if (epoch == num_of_epochs):
                pbar.close()
                nn_model.eval()
                v_x, v_y, omega, delta, long_acc = simulated_data_gen(nn_model, model, avg_vel)   
                C_Pf_identified, C_Pr_identified = solve_pacejka(model, v_x, v_y, omega, delta, long_acc)

                print(f"C_Pf_identified at Iteration {i}:", C_Pf_identified)
                print(f"C_Pr_identified at Iteration {i}:", C_Pr_identified)
                
                if plot_model:
                    rospy.logwarn("Close the plot window (press Q) to continue... ")
                    plot_results(model, v_x, v_y, omega, delta, C_Pf_identified, C_Pr_identified, i)   
                    
                # Update model with identified coefficients
                model['C_Pf_model'] = C_Pf_identified
                model['C_Pr_model'] = C_Pr_identified
            
        # plt.figure(figsize=(8, 5))
        # plt.title('loss')
        # plt.plot(loss_history, label='Train Loss')
        # plt.show()


    # Save the trained model with identified coefficients
    model_name = racecar_version +"_pacejka"
    car_model = get_dotdict(model_name)
    car_model.C_Pf = C_Pf_identified
    car_model.C_Pr = C_Pr_identified
    save(car_model)
    
    # Generate Look-Up Table (LUT) with the updated model
    rospy.loginfo("LUT is being generated...")
    LookupGenerator(racecar_version, save_LUT_name).run_generator()
