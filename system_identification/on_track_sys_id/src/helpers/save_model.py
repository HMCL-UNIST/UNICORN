import yaml
import os
import rospy
import rospkg

def save(model, overwrite_existing=True, verbose=False):

  rospack = rospkg.RosPack()
  package_path = rospack.get_path('on_track_sys_id')
  file_path = package_path + "/models/" + model['model_name'] +"/" + model['model_name'] +"_"+ model['tire_model'] + ".txt"
  if os.path.isfile(file_path):
    if (verbose): print("Model already exists")
    if overwrite_existing:
      if (verbose): print("Overwriting...")
    else:
      if (verbose): print("Not overwriting.")
      return 0

  try:
    model = model.to_dict()
  except:
    model = model

  # Write data to the file
  with open(file_path, "w") as f:
      rospy.loginfo(f"MODEL IS SAVED TO: {file_path}")
      yaml.dump(model, f, default_flow_style=False)