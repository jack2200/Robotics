import numpy as np
import math
import matplotlib.mlab as mlab
from matplotlib import pyplot as plt


save_particles = np.empty(shape=(100,1))

def low_variance_sampler(prev_particles,weights):
    particles = np.empty(shape=prev_particles.shape)
    m = particles.shape[0]
    r = np.random.uniform(low=0.0,high=1/m)
    c = weights[0]
    i = 0
    for j in range(m):
        u = r + j/m
        while u>c:
            i+=1
            c+=weights[i]
        #print("Chose {0} due to u = {1}, c = {2}".format(i,u,c))
        particles[j] = prev_particles[i]
    return particles


def sample_motion_model(control,particle):
    delta_t = 1
    A = np.array([[1,delta_t],[0,1]])
    B = np.array([[0.5 * delta_t ** 2],[delta_t]])
    C = np.array([1,0])
    R = np.array([[0.25 * delta_t ** 4, 0.5 * delta_t ** 3],[0.5 * delta_t ** 3, delta_t ** 2]]) # state noise
    
    particle = A.dot(particle) + np.reshape(B.dot(control),(-1,1))

    x = np.random.normal(loc=particle[0],scale=0.02)
    x_dot = np.random.normal(loc=particle[1],scale=0.05)
    particle = np.array([[x],[x_dot]])
    return np.reshape(particle,(1,-1))


def measurement_model(z,x):
    C = np.array([1,0])
    Q = 10 # measurement noise
    x_ = C.dot(x)
    N = np.sqrt(2*3.14*Q)*np.exp(-1/(2*Q)*(z - x_)**2);
    return N

def figure7():
	global save_particles
	def particle_filter_prediction(particles,control,total_t):
	    for j in range(total_t):
	        for i in range(particles.shape[0]):
	            particles[i] = sample_motion_model(control,np.reshape(particles[i],(-1,1)))
	        
	        plt.plot(particles[:,0], particles[:,1], 'o',markersize=1.5,label='t = {0}'.format(j+1))
	    return particles    
	N = 1000
	particles = np.zeros((N,2))
	accel = np.random.normal(size=1)
	fig = plt.figure(figsize=(15,15))
	plt.xlabel('position')
	plt.ylabel('velocity')
	plt.title('Particles with respect to time')
	plt.plot(particles[:,0], particles[:,1], 'o',markersize=12,label='t = 0')
	save_particles = particle_filter_prediction(particles,accel,5)
	plt.legend()




def figure8():
	global save_particles
	def particle_filter_measurement_update(particles,measurement):
	    importance_factors = np.empty(shape=(particles.shape[0],1))
	    plt.plot(particles[:,0], particles[:,1], 'o',markersize=3,label='Before z-update')
	    for i in range(particles.shape[0]):
	        importance_factors[i] = measurement_model(measurement,np.reshape(particles[i],(-1,1)))
	    
	    importance_factors = (importance_factors - np.min(importance_factors)) / (np.max(importance_factors) - np.min(importance_factors))
	    particles = low_variance_sampler(particles,importance_factors)
	    plt.plot(particles[:,0], particles[:,1], 'o',markersize=3,label='After z-update')
	    
	fig = plt.figure(figsize=(10,10))
	plt.xlabel('position')
	plt.ylabel('velocity')
	plt.title('Particles Before and After Measurement update')
	measurement = 5.0
	particle_filter_measurement_update(save_particles,measurement)
	plt.legend()


def figure9():
	def particle_filter(particles,control,measurement,t):
	    importance_factors = np.empty(shape=(particles.shape[0],1))
	    for i in range(particles.shape[0]):
	        particles[i] = sample_motion_model(control,np.reshape(particles[i],(-1,1)))
	        
	        importance_factors[i] = measurement_model(measurement,np.reshape(particles[i],(-1,1)))
	    plt.plot(particles[:,0], particles[:,1], 'o',markersize=4,label='t = {0}'.format(t))    
	    importance_factors = (importance_factors - np.min(importance_factors)) / (np.max(importance_factors) - np.min(importance_factors))
	    particles = low_variance_sampler(particles,importance_factors)
	    
	    return particles

	N = 1000
	particles = np.zeros((N,2))
	np.random.seed(seed=10)
	accel = np.random.normal(size=1)
	fig = plt.figure(figsize=(10,10))
	plt.xlabel('position')
	plt.ylabel('velocity')
	plt.title('Particles with respect to time - before measurement update')

	measurement = 5.0
	for i in range(1,6):
	    particles = particle_filter(particles,accel,measurement,i)
	plt.legend()

def figure10():
	def particle_filter(particles,control,measurement,t):
	    importance_factors = np.empty(shape=(particles.shape[0],1))
	    for i in range(particles.shape[0]):
	        particles[i] = sample_motion_model(control,np.reshape(particles[i],(-1,1)))
	        
	        importance_factors[i] = measurement_model(measurement,np.reshape(particles[i],(-1,1)))
	        
	    importance_factors = (importance_factors - np.min(importance_factors)) / (np.max(importance_factors) - np.min(importance_factors))
	    particles = low_variance_sampler(particles,importance_factors)
	    plt.plot(particles[:,0], particles[:,1], 'o',markersize=4,label='t = {0}'.format(t))
	    return particles

	N = 1000
	particles = np.zeros((N,2))
	np.random.seed(seed=10)
	accel = np.random.normal(size=1)
	fig = plt.figure(figsize=(10,10))
	plt.xlabel('position')
	plt.ylabel('velocity')
	plt.title('Particles with respect to time - after measurement update')

	measurement = 5.0
	for i in range(1,6):
	    particles = particle_filter(particles,accel,measurement,i)
	plt.legend()

def comparison():
	import time
	def particle_filter(particles,control,measurement):
	    importance_factors = np.empty(shape=(particles.shape[0],1))
	    for i in range(particles.shape[0]):
	        particles[i] = sample_motion_model(control,np.reshape(particles[i],(-1,1)))
	        
	        importance_factors[i] = measurement_model(measurement,np.reshape(particles[i],(-1,1)))
	        
	    importance_factors = (importance_factors - np.min(importance_factors)) / (np.max(importance_factors) - np.min(importance_factors))
	    particles = low_variance_sampler(particles,importance_factors)
	    return particles

	N = 1000
	particles = np.zeros((N,2))
	accel = np.random.normal(size=1)

	measurement = 5.0
	start_time = time.time()
	particles = particle_filter(particles,accel,measurement)
	print("---Particle filter: %s seconds ---" % (time.time() - start_time))

	def kalman_filter(mean,covariance,control,measurement):
	    delta_t = 1
	    A = np.array([[1,delta_t],[0,1]])
	    B = np.array([[0.5 * delta_t ** 2],[delta_t]])
	    C = np.array([1,0])
	    R = np.array([[0.25 * delta_t ** 4, 0.5 * delta_t ** 3],[0.5 * delta_t ** 3, delta_t ** 2]]) # state noise
	    Q = 10
	    # prediction beliefs
	    pred_mean = A.dot(mean) + np.reshape(B.dot(control),(-1,1))
	    pred_state_covariance = A.dot(covariance).dot(A.T) + R
	        
	    pred_measurement_covariance = C.dot(pred_state_covariance).dot(C.T) + Q
	    kalman_gain = pred_state_covariance.dot(C.T) / pred_measurement_covariance
	    kalman_gain = np.reshape(kalman_gain,(-1,1))
	    mean = pred_mean + np.reshape(kalman_gain.dot(measurement - C.dot(pred_mean)),(-1,1))
	    term = np.identity(np.size(pred_mean)) - kalman_gain.dot(np.reshape(C,(1,2)))
	    covariance = term.dot(pred_state_covariance)
	    return mean,covariance

	mu = np.array([[0],[0]])
	sigma = np.array([[1, 0],[0, 1]])
	accel = np.random.normal(size=1)
	measurement = 5.0
	start_time = time.time()
	mu,sigma = kalman_filter(mu,sigma,accel,measurement)
	print("---Kalman filter: %s seconds ---" % (time.time() - start_time))


if __name__=='__main__':
	figure7()
	figure8()
	figure9()
	figure10()
	comparison()
	plt.show()