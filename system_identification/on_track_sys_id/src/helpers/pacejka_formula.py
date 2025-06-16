import numpy as np
def pacejka_formula(params, alpha, F_z):
  B, C, D, E = params[0], params[1], params[2], params[3]
  y = F_z * D * np.sin(C*np.arctan(B*alpha - E * (B*alpha -np.arctan(B * alpha))))
  return y