import numpy as np
import math
import matplotlib.pyplot as plt

delta_t = 0.1
a1 = 0.1
a2 = 0.1
a3 = 0.1
a4 = 0.1

std_range = 0.2
std_yaw = 0.05
std_signature = 0.01

Qsim = np.diag([1.0, np.deg2rad(10.0)])**2
Rsim = np.diag([1.0, np.deg2rad(15.0), 0.0])**2 
def motion_model(x,u):
    theta = x[2,0]
    v = u[0, 0]
    w = u[1, 0]
    r = v/w
    A = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[r*(-math.sin(theta) + math.sin(theta + w*delta_t))],
                  [r*(math.cos(theta) - math.cos(theta + w*delta_t))],
                  [w*delta_t]])
    x_ = A@x + B

    return x_

def ekf_localization(estimated_state_mean, estimated_state_covariance, z, u, m):  
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
    M = np.array([
                 [(a1*math.fabs(v) + a2*math.fabs(w))**2, 0],
                 [0, (a3*math.fabs(v) + a4*math.fabs(w))**2]
    ])
    #  Predict
    pred_state_covariance = G@estimated_state_covariance@G.T + V@M@V.T
    pred_state_mean = motion_model(estimated_state_mean, u)
    #print("Predicted position(",pred_state_mean.shape,"): ",pred_state_mean.T)
    #  Correction

    """
    Measurement noise on landmarks
    """
    Q = np.array([
        [std_range**2, 0, 0],
        [0, std_yaw**2, 0],
        [0, 0, std_signature**2]
    ])
    measurement_likelihood = 1.0 # can be used to reject outliers
    """
    z is a vector of (r phi s) tuples for each landmark.
    """
    for feature in z:
        feature = np.reshape(feature,(-1,1))
        H_k = []
        S_k = []
        y_k = []
        measurement_likelihoods = []
        for landmark in m:
            q = (landmark[0] - pred_state_mean[0,0])**2 + (landmark[1] - pred_state_mean[1,0])**2
            zPred = np.array([
                [math.sqrt(q)],
                [math.atan2(landmark[1] - pred_state_mean[1,0], landmark[0] - pred_state_mean[0,0])],
                [landmark[2]]
            ])
            """
            H is the Jacobian of measurement model with respect to robot pose (at pred_state_mean)
            used in the Taylor approximation to approximate non-linearity with linear tools.
            Since measurement model outputs tuples of threes - r,phi,s - the partial derivatives
            will be as follows:
            dr/dx (pred_state_mean) = - (m[j],x - pred_state_mean,x) / sqrt(q)
            dr/dy (pred_state_mean) = - (m[j],y - pred_state_mean,y) / sqrt(q)
            dphi/dx (pred_state_mean) = (m[j],y - pred_state_mean,y) / q
            dphi/dy (pred_state_mean) = - (m[j],x - pred_state_mean,x) / q
            dphi/dyaw = -1
            """
            H = np.array([
                [-(landmark[0] - pred_state_mean[0,0]) / math.sqrt(q), -(landmark[1] - pred_state_mean[1,0]) / math.sqrt(q), 0],
                [(landmark[1] - pred_state_mean[1,0]) / q , -(landmark[0] - pred_state_mean[0,0]) / q, -1],
                [0,0,0]
            ]) 
            S = H@pred_state_covariance@H.T + Q
            y = feature - zPred # innovation vector
            second_term = y.T@np.linalg.inv(S)@y
            first_term = 1.0/math.sqrt(np.linalg.det(2*math.pi*S))

            H_k.append(H)
            S_k.append(S)
            y_k.append(y)
            measurement_likelihoods.append(first_term*np.exp(-second_term/2))

        j = np.argmax(measurement_likelihoods)
        
        K = pred_state_covariance@H_k[j].T@np.linalg.inv(S_k[j])
        
        pred_state_mean = pred_state_mean + K@y_k[j]
        pred_state_covariance = (np.eye(len(pred_state_mean)) - K@H_k[j])@pred_state_covariance
        
    estimated_state_mean = pred_state_mean
    estimated_state_covariance = pred_state_covariance
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
    SIM_TIME = 60.0

    time = 0.0

    # State Vector [x y yaw]'
    estimated_state_mean = np.zeros((3, 1))
    true_state= np.zeros((3, 1))
    estimated_state_covariance = np.eye(3)


    # history
    history_estimated_state = estimated_state_mean
    history_true_state = true_state
    v = 1.0
    yawrate= 0.1
    m = np.array([
        [2,5,0],
        [5,2,1],
        [10,5,2],
        [0,-3,3],
        [-5,3,4]
    ])
    while SIM_TIME >= time:
        time += delta_t
        u = np.array([[v,yawrate]]).T
        z = np.empty(m.shape)
        true_state = motion_model(true_state, u)
        ud1 = u[0, 0] + np.random.randn() * Qsim[0, 0]
        ud2 = u[1, 0] + np.random.randn() * Qsim[1, 1]
        u = np.array([[ud1, ud2]]).T
        for i in range(0,len(m)):
            z[i][0] = math.sqrt((m[i][0] - true_state[0,0])**2 + (m[i][1] - true_state[1,0])**2) + np.random.randn() * Rsim[0,0]
            z[i][1] = math.atan2(m[i][1] - true_state[1,0], m[i][0] - true_state[0,0]) + np.random.randn() * Rsim[1,0]
            z[i][2] = m[i][2] + np.random.randn() * Rsim[2,0]
        estimated_state_mean, estimated_state_covariance = ekf_localization(estimated_state_mean, estimated_state_covariance, z, u, m)
        # store data history
        history_estimated_state = np.hstack((history_estimated_state, estimated_state_mean))
        history_true_state = np.hstack((history_true_state, true_state))
        
        plt.cla()
        for landmark in m:
            landmark_ = plt.Circle((landmark[0], landmark[1]), 0.5, color='green')
            plt.gcf().gca().add_artist(landmark_)
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
