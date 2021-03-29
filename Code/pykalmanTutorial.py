# from pykalman import KalmanFilter

# kf = KalmanFilter(initial_state_mean=0, n_dim_obs=2)
# measurements = [[1,0], [0,0], [0,1]]
# kf.em(measurements).smooth([[2,0], [2,1], [2,2]])[0]



# from scipy.stats import norm
# import numpy as np
# states = np.zeros((n_timesteps, n_dim_state))
# measurements = np.zeros((n_timesteps, n_dim_obs))
# for t in range(n_timesteps-1):
#    if t == 0:
#       states[t] = norm.rvs(initial_state_mean, np.sqrt(initial_state_covariance))
#       measurements[t] = (
#           np.dot(observation_matrices[t], states[t])
#           + observation_offsets[t]
#           + norm.rvs(0, np.sqrt(observation_covariance))
#       )
#   states[t+1] = (
#       np.dot(transition_matrices[t], states[t])
#       + transition_offsets[t]
#       + norm.rvs(0, np.sqrt(transition_covariance))
#   )
#   measurements[t+1] = (
#       np.dot(observation_matrices[t+1], states[t+1])
#       + observation_offsets[t+1]
#       + norm.rvs(np.sqrt(observation_covariance))
#   )

