import torch

def generate_inputs_errors(v_y_next_pred, omega_next_pred, data):
  """
  Generate inputs and errors for the neural network.

  Computes errors between predicted and actual lateral velocity (v_y) and yaw rate (omega).
  Constructs input tensors for the neural network from the input data.

  Args:
      v_y_next_pred (numpy.ndarray): Predicted lateral velocity for the next timestep.
      omega_next_pred (numpy.ndarray): Predicted yaw rate for the next timestep.
      data (numpy.ndarray): Input data array with shape (n_samples, 4).

  Returns:
      tuple: Tuple containing input tensor and error tensor.
  """
  # Make copies of the input arrays
  v_y_next_pred = v_y_next_pred.copy()
  omega_next_pred = omega_next_pred.copy()
  v_x = data[:, 0].copy()
  v_y = data[:, 1].copy()
  omega = data[:, 2].copy()
  delta = data[:, 3].copy()
  long_acc = data[:, 4].copy()

  # Convert to PyTorch tensors if not already tensors
  v_y_next_pred = torch.tensor(v_y_next_pred, dtype=torch.float32)
  omega_next_pred = torch.tensor(omega_next_pred, dtype=torch.float32)
  v_x = torch.tensor(v_x, dtype=torch.float32)
  v_y = torch.tensor(v_y, dtype=torch.float32)
  omega = torch.tensor(omega, dtype=torch.float32)
  delta = torch.tensor(delta, dtype=torch.float32)
  long_acc = torch.tensor(delta, dtype=torch.float32)

  # Compute errors between predicted and actual v_y and omega
  error_v_y = (v_y[1:] - v_y_next_pred)
  error_omega = (omega[1:] - omega_next_pred)

  # Assemble inputs tensor directly from the already tensor-typed variables
  inputs = torch.stack([v_x[:-1], v_y[:-1], omega[:-1], delta[:-1], long_acc[:-1]], dim=1)
  errors = torch.stack([error_v_y, error_omega], dim=1)
  return inputs, errors