import numpy as np
import math
import matplotlib.mlab as mlab
from matplotlib import pyplot as plt




def plot_uncertainty_ellipse(mu,sigma,c,l=None):

        D,V = np.linalg.eig(sigma)
        t = np.linspace(0, 2*3.14, 100)
        m = V*np.sqrt(D)
        n = np.reshape(np.cos(t),(1,-1))
        n = np.append(n,np.reshape(np.sin(t),(1,-1)),axis=0)
        a = m.dot(n)
        if l is not None:
            plt.plot(mu[0]+a[0,:], mu[1]+a[1,:],color=c,label=l)
        else:
            plt.plot(mu[0]+a[0,:], mu[1]+a[1,:],color=c)

def figure5():
    def kalman_filter_measurement_update_with_plot(pred_state_mean,pred_state_covariance):
        C = np.array([1,0])
        Q = 10 # measurement noise
        measurement = 5
        plot_uncertainty_ellipse(pred_state_mean,pred_state_covariance,'red')
        
        pred_measurement_covariance = C.dot(pred_state_covariance).dot(C.T) + Q
        kalman_gain = pred_state_covariance.dot(C.T) / pred_measurement_covariance
        kalman_gain = np.reshape(kalman_gain,(-1,1))
        mean = pred_state_mean + np.reshape(kalman_gain.dot(measurement - C.dot(mu)),(-1,1))
        term = np.identity(np.size(mu)) - kalman_gain.dot(np.reshape(C,(1,2)))
        covariance = term.dot(pred_state_covariance)
        print(mean)
        print(covariance)
        plot_uncertainty_ellipse(mean,covariance,'green')

    mu = np.array([[0],[0]])
    sigma = np.array([[20, 6],[6, 3]])

    plt.title('Uncertainty ellipse')
    plt.xlabel('position')
    plt.ylabel('velocity')
    kalman_filter_measurement_update_with_plot(mu,sigma)

    mu = np.array([[0],[0]])
    sigma = np.array([[20, 6],[6, 3]])

    
    plt.title('Uncertainty ellipse')
    plt.xlabel('position')
    plt.ylabel('velocity')
    kalman_filter_measurement_update_with_plot(mu,sigma)



if __name__=='__main__':
	fig = plt.figure(figsize=(8,8))
	figure5()
	plt.show()