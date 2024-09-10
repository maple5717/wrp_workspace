import numpy as np
from filterpy.common import Q_discrete_white_noise, Q_continuous_white_noise
import argparse 

# dt = 0.25 * 100
# var_trans = 1.0 * 1000 # Variance for translation components
# var_orient = 1.0 * 1000  # Variance for orientation components (yaw, roll)
# consider_acc = False
# acc_noise = 1.0
parser = argparse.ArgumentParser(description='Process some input parameters.')
parser.add_argument('--dt', type=float, default=0.25 * 100, help='Time step (dt)')
parser.add_argument('--var_trans', type=float, default=1.0 * 1000, help='Variance for translation components')
parser.add_argument('--var_orient', type=float, default=1.0 * 1000, help='Variance for orientation components (yaw, roll)')
parser.add_argument('--consider_acc', type=bool, default=True, help='Consider acceleration noise (True/False)')
parser.add_argument('--acc_noise', type=float, default=1.0, help='Acceleration noise level')

# Parse the arguments
args = parser.parse_args()

# Use the arguments
dt = args.dt
var_trans = args.var_trans
var_orient = args.var_orient
consider_acc = args.consider_acc
acc_noise = args.acc_noise

def generate_covariance_matrix(dim, var):
    global dt, consider_acc, acc_noise
    """
    Generate a process noise covariance matrix for a discrete-time system using white noise.
    """
    if dim <= 0:
        raise ValueError("Dimension must be a positive integer")
    if var <= 0:
        raise ValueError("Variance must be a positive number")

    # Generate covariance matrix using Q_discrete_white_noise
    if dim == 3 and not(consider_acc):
        Q = np.eye(3) * acc_noise
        Q[:2, :2] = Q_discrete_white_noise(dim=2, dt=dt, var=var)
    else:
        Q = Q_discrete_white_noise(dim=dim, dt=dt, var=var)
        print(Q)
    return Q
def permute_covariance_matrix(Q):
    """
    Permute a covariance matrix that represents [x, x', x'', y, y', y'', yaw, yaw', yaw'', ...]
    to the desired order [x, y, z, roll, pitch, yaw, x', y', z', roll', pitch', yaw', x'', y'', z''].
    
    Args:
        Q (numpy.ndarray): The input covariance matrix with the initial ordering.
    
    Returns:
        numpy.ndarray: The permuted covariance matrix.
    """
    # Define the desired order
    # The input order is [x, x', x'', y, y', y'', yaw, yaw', yaw'', ...]
    # The output order is [x, y, z, roll, pitch, yaw, x', y', z', roll', pitch', yaw', x'', y'', z'']
    
    # Total dimensions should match
    n = Q.shape[0]
    
    # Initialize the permuted matrix with zeros
    permuted_Q = np.zeros_like(Q)
    
    # Mapping indices from input order to output order
    input_indices = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]

    


    # [0, 3, x, x, x, 6, 1, 4, x, x, x, 7, 2, 5, x]
    output_indices = [0, 3, 8,9,10, 6, 1, 4, 11,12,13, 7, 2, 5, 14]
    # output_indices = [0, 3, 6, 9, 12, 15, 1, 4, 7, 10, 13, 16, 2, 5, 8]
    
    # # Create a mapping from output indices to input indices
    # index_map = {output_indices[i]: input_indices[i] for i in range(len(output_indices)-1)}
    
    # # Apply the permutation
    # for i in range(n):
    #     for j in range(n):
    #         permuted_Q[i, j] = Q[index_map[i], index_map[j]]
    permuted_Q = Q[np.ix_(output_indices, output_indices)]
    # permuted_Q = Q[:, output_indices]
    # permuted_Q = np.array(permuted_Q.T)
    # permuted_Q = Q[output_indices]
    return permuted_Q

def concat_covariance_matrix(Q_trans, Q_orient, target_dim):
    """
    Permute the covariance matrices for translation and orientation into a larger matrix.
    """
    if Q_trans.shape[0] != 3 or Q_orient.shape[0] != 2:
        raise ValueError("Q_trans must be 3x3 and Q_orient must be 2x2 matrices")

    # Initialize permutation matrix with zeros
    P = np.eye(target_dim, dtype=np.float32) * 3.14
    
    # Fill the covariance matrix for translation components [x, y, z]
    P[0:3, 0:3] = Q_trans
    P[3:6, 3:6] = Q_trans
    
    # Fill the covariance matrix for orientation components [yaw, roll, pitch]
    P[6:8, 6:8] = Q_orient
    
    # # Derivatives for translation components [x', y', z']
    # P[9:12, 9:12] = Q_trans
    
    # # Derivatives for orientation components [yaw', roll', pitch']
    # P[12:14, 12:14] = Q_orient
    
    # # Second derivatives for translation components [x'', y'', z'']
    # P[15:18, 15:18] = Q_trans
    
    return P

def main():
    dim_trans = 3  # Dimension of the translation state vector [x, y, z]
    dim_orient = 2  # Dimension of the orientation state vector [yaw, roll]
    
    

    # Generate the initial covariance matrices
    Q_trans = generate_covariance_matrix(dim_trans, var_trans)
    Q_orient = generate_covariance_matrix(dim_orient, var_orient)
    
    # Target dimension for [x, y, z, roll, pitch, yaw, x', y', z', roll', pitch', yaw', x'', y'', z'']
    target_dim = 15
    
    # Permute the covariance matrix
    Q = concat_covariance_matrix(Q_trans, Q_orient, target_dim)
    permuted_Q = permute_covariance_matrix(Q)
    # permuted_Q = np.eye(15) * 0.04
    # Print the resulting permuted covariance matrix
    print("Permuted Covariance Matrix:")
    # print(permuted_Q)
    x, y = permuted_Q.shape 
    # for i in range(x):
    #     print(', '.join([str(item) for item in Q[i]]) + ', ')
    print(x,y)
    for i in range(x):
        print(', '.join([f"{item * 1.0:.4e}" for item in permuted_Q[i]]) + ', ')
    print(permuted_Q[permuted_Q > 0].min())
if __name__ == "__main__":
    main()
