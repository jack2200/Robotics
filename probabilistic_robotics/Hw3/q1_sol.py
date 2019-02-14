import numpy as np
import math
import matplotlib.mlab as mlab
from matplotlib import pyplot as plt

## Figure 1 can be replicated through running this cell.
def figure1():
	def true_motion_sim(c,n):
	    t = 1
	    A = np.array([[1,t],[0,1]])
	    B = np.array([[0.5 * t ** 2],[t]])
	    states = np.array([[0],
	                  [0]])
	    
	    x = np.linspace(0,10,11)
	    y = states[0]
	    for i in range(10):
	        accel = np.random.normal(size=1)
	        states = A.dot(states) + np.reshape(B.dot(accel),(-1,1))
	        y = np.append(y,states[0])
	    plt.plot(x,y,'-o',color=c,label='trajectory {0}'.format(n))
	    
	    
	    
	plt.figure()
	plt.xlabel('time')
	plt.ylabel('position')

	true_motion_sim('green',1)
	true_motion_sim('red',2)
	true_motion_sim('blue',3)
	plt.legend()

def kalman_filter_prediction_step_with_plot(mean,covariance,control,total_t,c,ax=None):
    delta_t = 1
    A = np.array([[1,delta_t],[0,1]])
    B = np.array([[0.5 * delta_t ** 2],[delta_t]])
    R = np.array([[0.25 * delta_t ** 4, 0.5 * delta_t ** 3],[0.5 * delta_t ** 3, delta_t ** 2]]) # state noise
    for i in range(total_t):
        control = np.random.normal(size=1)
        mean = A.dot(mean) + np.reshape(B.dot(control),(-1,1))
        covariance = A.dot(covariance).dot(A.T) + R
    if ax is not None:
        ax.set_xlabel('position')
        ax.set_ylabel('velocity')
    else:
        plt.xlabel('position')
        plt.ylabel('velocity')
    D,V = np.linalg.eig(covariance)
    t = np.linspace(0, 2*3.14, 100)
    m = V*np.sqrt(D)
    n = np.reshape(np.cos(t),(1,-1))
    n = np.append(n,np.reshape(np.sin(t),(1,-1)),axis=0)
    a = m.dot(n)
    if ax is not None:
        ax.plot(mean[0]+a[0,:], mean[1]+a[1,:],color=c,label='t={0}'.format(total_t))
    else:
        plt.plot(mean[0]+a[0,:], mean[1]+a[1,:],color=c,label='t={0}'.format(total_t))
    
    correlation = covariance[0][1] / (np.sqrt(covariance[0][0])*np.sqrt(covariance[1][1]))
    print("Correlation: ",correlation)


def figure2():
    np.random.seed(seed=2)
    accel = np.random.normal(size=1)
    means = np.array([[0],
                      [0]])
    covariance = np.array([[0,0],
                           [0,0]])
    fig, ax = plt.subplots(2, 3, sharex=True, sharey=True)
    fig.suptitle('Uncertainty ellipses with respect to time')
    kalman_filter_prediction_step_with_plot(means,covariance,accel,1,'green',ax[0][0])
    kalman_filter_prediction_step_with_plot(means,covariance,accel,2,'red',ax[0][1])
    kalman_filter_prediction_step_with_plot(means,covariance,accel,3,'blue',ax[0][2])
    kalman_filter_prediction_step_with_plot(means,covariance,accel,4,'yellow',ax[1][0])
    kalman_filter_prediction_step_with_plot(means,covariance,accel,5,'magenta',ax[1][1])
    fig.legend(loc='lower right',bbox_to_anchor=(0.88, 0.12))
    

def figure3():
    np.random.seed(seed=3)
    accel = np.random.normal(size=1)
    means = np.array([[0],
                      [0]])
    covariance = np.array([[0,0],
                           [0,0]])
    fig = plt.figure()
    plt.grid(color='lightgray',linestyle='--')
    kalman_filter_prediction_step_with_plot(means,covariance,accel,1,'green')
    kalman_filter_prediction_step_with_plot(means,covariance,accel,2,'red')
    kalman_filter_prediction_step_with_plot(means,covariance,accel,3,'blue')
    kalman_filter_prediction_step_with_plot(means,covariance,accel,4,'yellow')
    kalman_filter_prediction_step_with_plot(means,covariance,accel,5,'magenta')
    plt.legend()
    

def figure4():
    np.random.seed(seed=4)
    accel = np.random.normal(size=1)
    means = np.array([[0],
                      [0]])
    covariance = np.array([[0,0],
                           [0,0]])
    fig = plt.figure()
    kalman_filter_prediction_step_with_plot(means,covariance,accel,50,'green')
    kalman_filter_prediction_step_with_plot(means,covariance,accel,100,'red')
    kalman_filter_prediction_step_with_plot(means,covariance,accel,150,'blue')
    kalman_filter_prediction_step_with_plot(means,covariance,accel,250,'yellow')
    kalman_filter_prediction_step_with_plot(means,covariance,accel,400,'magenta')
    plt.legend()
    



if __name__=='__main__':
	figure1()
	figure2()
	figure3()
	figure4()
	plt.show()