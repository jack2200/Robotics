import numpy as np
import math
import matplotlib.pyplot as plt

delta_t = 0.1
Q = np.diag([0.1, 0.1, np.deg2rad(1.0), 1.0])**2  # predict state covariance
R = np.diag([1.0, 1.0])**2  # Observation x,y position covariance

# simulation parameters
Qsim = np.diag([1.0, np.deg2rad(30.0)])**2
Rsim = np.diag([0.5, 0.5])**2
def motion_model(x,u):
    A = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    B = np.array([[delta_t * math.cos(x[2, 0]), 0],
                  [delta_t * math.sin(x[2, 0]), 0],
                  [0.0, delta_t],
                  [1.0, 0.0]])

    x_ = A@x + B@u

    return x_

def jacobian_motion(x,u):
    """
    Jacobian of Motion Model

    __motion model__
    x_{t+1} = x_t+v*dt*cos(yaw)
    y_{t+1} = y_t+v*dt*sin(yaw)
    yaw_{t+1} = yaw_t+omega*dt
    v_{t+1} = v{t}
    ---
    __partial derivatives__
    dx/dyaw = -v*dt*sin(yaw)
    dx/dv = dt*cos(yaw)
    dy/dyaw = v*dt*cos(yaw)
    dy/dv = dt*sin(yaw)
    dx/dt = 1
    dy/dt = 1
    dyaw/dt = 1
    """
    yaw = x[2, 0]
    v = u[0, 0]
    J = np.array([
        [1.0, 0.0, -delta_t*v*math.sin(yaw), delta_t*math.cos(yaw)],
        [0.0, 1.0, v*delta_t*math.cos(yaw), delta_t*math.sin(yaw)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])
    return J

def jacobian_observation(x):
    H = np.array([
	    [1, 0, 0, 0],
	    [0, 1, 0, 0]
    ])
    return H

def ekf(estimated_state_mean, estimated_state_covariance, z, u):

    #  Predict
    pred_state_mean = motion_model(estimated_state_mean, u)
    G = jacobian_motion(pred_state_mean, u)
    pred_state_covariance = G@estimated_state_covariance@G.T + Q

    #  Update
    H = jacobian_observation(pred_state_mean)
    zPred = H@pred_state_mean
    y = z - zPred
    S = H@pred_state_covariance@H.T + R
    K = pred_state_covariance@H.T@np.linalg.inv(S)
    estimated_state_mean = pred_state_mean + K@y
    estimated_state_covariance = (np.eye(len(estimated_state_mean)) - K@H)@pred_state_covariance
    return estimated_state_mean, estimated_state_covariance

def plot_covariance_ellipse(mu,sigma):
    Sigma_xy = sigma[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Sigma_xy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
    R = np.array([[math.cos(angle), math.sin(angle)],
                  [-math.sin(angle), math.cos(angle)]])
    fx = R@(np.array([x, y]))
    px = np.array(fx[0, :] + mu[0, 0]).flatten()
    py = np.array(fx[1, :] + mu[1, 0]).flatten()
    plt.plot(px, py, "--r")

def main():
    SIM_TIME = 40.0
    print(__file__ + " start!!")

    time = 0.0

    # State Vector [x y yaw v]'
    estimated_state_mean = np.zeros((4, 1))
    true_state= np.zeros((4, 1))
    estimated_state_covariance = np.eye(4)


    # history
    history_estimated_state = estimated_state_mean
    history_true_state = true_state
    history_measurements = np.zeros((2, 1))
    v = 1.0
    yawrate= 0.1
    while SIM_TIME >= time:
        time += delta_t
        u = np.array([[v,yawrate]]).T

        true_state = motion_model(true_state, u)
        zx = true_state[0, 0] + np.random.randn() * Rsim[0, 0]
        zy = true_state[1, 0] + np.random.randn() * Rsim[1, 1]
        z = np.array([[zx, zy]]).T

        ud1 = u[0, 0] + np.random.randn() * Qsim[0, 0]
        ud2 = u[1, 0] + np.random.randn() * Qsim[1, 1]
        u = np.array([[ud1, ud2]]).T
        estimated_state_mean, estimated_state_covariance = ekf(estimated_state_mean, estimated_state_covariance, z, u)

        # store data history
        history_estimated_state = np.hstack((history_estimated_state, estimated_state_mean))
        history_true_state = np.hstack((history_true_state, true_state))
        history_measurements = np.hstack((history_measurements, z))

        
        plt.cla()
        plt.plot(history_measurements[0, :], history_measurements[1, :], ".g")
        plt.plot(history_true_state[0, :].flatten(),
                 history_true_state[1, :].flatten(), "-b")
        plt.plot(history_estimated_state[0, :].flatten(),
                 history_estimated_state[1, :].flatten(), "-r")
        plot_covariance_ellipse(estimated_state_mean, estimated_state_covariance)
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)


if __name__ == '__main__':
    main()
