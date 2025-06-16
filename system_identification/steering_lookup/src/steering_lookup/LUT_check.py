import numpy as np
import rospkg


# rospack = rospkg.RosPack()
# path = rospack.get_path('steering_lookup')
# ############## YH ##############
# # file_path = path + '/cfg/' + model_name + '_lookup_table.csv'
# file_path = path + '/cfg/' + model_name + '_pacejka_lookup_table.csv'
file_path = '/home/nuc14/icra_ws/unicorn/system_identification/steering_lookup/cfg/UNICORN2-0325-test-acc0_pacejka_lookup_table.npy'
############## YH ##############
try:
    lu = np.load(file_path)
except IOError:
    raise IOError("Lookup table not found at " + file_path + ". Please check the file path.")
count = 0
count_nan = 0
for mat in lu:
    for row in mat:
        for element in row:
            if element == 0:
                count += 1
            if np.isnan(element):
                count_nan +=1
print('total 0 count = ' , count)
print('total nan count = ' , count_nan)
print('lu shape = ' , np.shape(lu))
print(lu[0,2,0])