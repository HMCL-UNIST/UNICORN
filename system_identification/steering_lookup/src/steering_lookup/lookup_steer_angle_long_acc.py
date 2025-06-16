import numpy as np
import rospkg

def find_nearest(array, value):
    # array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx], idx

def find_closest_neighbors(array, value):
    # index of first nan
    is_nan_array = np.argwhere(np.isnan(array))
    if len(is_nan_array) > 0:
      first_nan = is_nan_array[0][0]
      array = array[0:first_nan]
    closest, closest_idx = find_nearest(array, value)
    if closest_idx == 0:
        return array[0], 0, array[0], 0
    elif closest_idx == (len(array) - 1):
        return array[closest_idx], closest_idx, array[closest_idx], closest_idx
    else:
      second_closest, second_idx = \
        find_nearest(array[[max(closest_idx-1, 0), 
        min(closest_idx+1, len(array)-1)]], value)
      second_idx = -1 + closest_idx + 2 * second_idx
      return closest, closest_idx, second_closest, second_idx


class LookupSteerAngle:
    """
    LookupSteerAngle:
    """
    def __init__(self, model_name, logger):
      rospack = rospkg.RosPack()
      path = rospack.get_path('steering_lookup')
      file_path = path + '/cfg/' + model_name + '_pacejka_lookup_table.npy'
      try:
          self.lu = np.load(file_path)
      except IOError:
          raise IOError("Lookup table not found at " + file_path + ". Please check the file path.")
      self.logger = logger

    def lookup_steer_angle(self, lat_acc, vel, long_acc):
      """
      Input: 
          lat_acc   - Lateral acceleration [m/s²]
          vel       - Vehicle speed [m/s]
          long_acc  - Longitudinal acceleration [m/s²]
      Output:
          Required steering angle
      
      Lookup table structure:
          self.lu[0, 1:, 0]   : Speed values (velocities)
          self.lu[0, 0, 1:]   : Longitudinal acceleration values
          self.lu[1:, 0, 0]   : Steering angle values
          self.lu[1:, 1:, 1:]: Lateral acceleration for each (steer, vel, long_acc) combination
      """
      # Store the sign of lateral acceleration to be applied to the final steering angle
      sign = 1.0 if lat_acc > 0.0 else -1.0
      lat_acc = abs(lat_acc)

      # Extract independent variable axes from the lookup table
      lu_vels = self.lu[0, 1:, 0]       # Speed values
      lu_long_accs = self.lu[0, 0, 1:]    # Longitudinal acceleration values
      lu_steers = self.lu[1:, 0, 0]       # Steering angle values

      # Find the index in the lookup table corresponding to the given speed
      _, v_idx = find_nearest(lu_vels, vel)
      # Find the index for the given longitudinal acceleration
      _, long_acc_idx = find_nearest(lu_long_accs, long_acc)

      # For the combination of speed and longitudinal acceleration, extract the lateral acceleration values across steering angles
      # Use v_idx+1 and la_idx+1 for proper indexing correction
      lat_acc_array = self.lu[1:, v_idx+1, long_acc_idx+1]

    #   print(lat_acc)
    #   print(lat_acc_array)

      # Find the two closest lateral acceleration values and their indices in the 1D array
      c_val, c_idx, s_val, s_idx = find_closest_neighbors(lat_acc_array, lat_acc)

    #   print(f"c_val : {c_val}, c_idx : {c_idx}, s_val : {s_val} , s_idx : {s_idx} ")

      if c_idx == s_idx:
          steer_angle = lu_steers[c_idx]
      else:
          # Interpolate linearly between the two neighboring values to estimate the steering angle
          steer_angle = np.interp(lat_acc, [c_val, s_val], [lu_steers[c_idx], lu_steers[s_idx]])
      return steer_angle * sign

# test case
if __name__ == "__main__":
  detective =  LookupSteerAngle("NUC1_pacejka", print)
  steer_angle = detective.lookup_steer_angle(9, 7)
  print(steer_angle)