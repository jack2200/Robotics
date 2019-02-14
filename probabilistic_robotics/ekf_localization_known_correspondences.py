import numpy as np
import math
import matplotlib.pyplot as plt

delta_t = 0.1
a1 = 0.1
a2 = 0.1
a3 = 0.1
a4 = 0.1

std_range = 0.5
std_yaw = 0.05
std_signature = 0.1

def motion_model(x,u):
    theta = x[2,0]
    v = u[0, 0]
    w = u[1, 0]
    r = v/w
    A = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[-r*(math.sin(theta) + math.sin(theta + w*delta_t))],
                  [r*(math.cos(theta) - math.cos(theta + w*delta_t))],
                  [w*delta_t]])

    x_ = A@x + B

    return x_

def ekf_localization(estimated_state_mean, estimated_state_covariance, z, u):

    
    """
    Jacobian of Motion Model

    __motion model__
    x_{t+1} = x_t - v/w*sin(yaw_t) + v/w*sin(yaw_t + w*dt)
    y_{t+1} = y_t + v/w*cos(yaw_t) - v/w*cos(yaw_t + w*dt)
    yaw_{t+1} = yaw_t + w*dt
    ---
    __partial derivatives__
    dx/dyaw = - v/w*cos(yaw_t) + v/w*cos(yaw_t + w*dt)
    dy/dyaw = -v/w*sin(yaw_t) + v/w*sin(yaw_t + w*dt)
    dy/dv = dt*sin(yaw)
    dx/dt = 1
    dy/dt = 1
    dyaw/dt = 1
    """
    yaw = estimated_state_mean[2, 0]
    v = u[0, 0]
    w = u[1, 0]
    G = np.array([
        [1.0, 0.0, -v/w*math.cos(yaw) + v/w*math.cos(yaw + w*delta_t)],
        [0.0, 1.0, -v/w*math.sin(yaw) + v/w*math.sin(yaw + w*delta_t)],
        [0.0, 0.0, 1.0],
    ])

    """
    Jacobian of Gaussian Noise in Motion Model
    __noise__
    N(0,R_t)
    __derivative of non-linear motion function with respect to control inputs__
    dx/dv = (-sin(yaw_t) + sin(yaw_t + w*dt)) / w
    dx/dw = v *(sin(yaw_t) - sin(yaw_t +w*dt))/w^2 + v*cos(yaw_t+w*dt)*dt/w
    dy/dv = (cos(yaw_t) - cos(yaw_t +w*dt))/w 
    dy/dw = - v *(cos(yaw_t) - cos(yaw_t + w*dt)) /w^2 + v*sin(yaw_t+w*dt)*dt/w
    dyaw/dv = 0
    dyaw/dw = dt
    """

    V = np.array([
        [(-math.sin(yaw) + math.sin(yaw + w*delta_t))/w, v*(math.sin(yaw) - math.sin(yaw + w*delta_t)) / (w**2) + v*math.cos(yaw + w*delta_t)*delta_t/w],
        [(math.cos(yaw) - math.cos(yaw + w*delta_t))/w, -v*(math.cos(yaw) - math.cos(yaw + w*delta_t)) / (w**2) + v*math.sin(yaw + w*delta_t)*delta_t/w],
        [0, delta_t]
    ])

    """ 
    Covariance of the additional motion noise N(0,R_t)
    in control space.
    """
    M = np.array([(a1*math.abs(v) + a2*math.abs(w))**2, 0],
                 [0, (a3*math.abs(v) + a4*math.abs(w))**2])
    #  Predict
    pred_state_covariance = G@estimated_state_covariance@G.T + V@M@V.T
    pred_state_mean = motion_model(estimated_state_mean, u)
    #  Correction

    """
    Measurement noise on landmarks
    """
    Q = np.array([
        [std_range**2, 0, 0],
        [0, std_yaw**2, 0],
        [0, 0, std_signature**2]
    ])
    
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

    # State Vector [x y yaw]'
    estimated_state_mean = np.zeros((3, 1))
    true_state= np.zeros((3, 1))
    estimated_state_covariance = np.eye(3)


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
