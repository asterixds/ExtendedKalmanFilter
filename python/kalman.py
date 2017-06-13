%matplotlib inline
from matrix import *
import numpy as np

"""prediction step"""
def predict(x, P):
    x = (F * x) + u
    P = F * P * F.transpose()
    return x, P

"""measurement update step"""
def update(x, P,z):
    # measurement update
    Z = matrix([z])
    y = Z.transpose() - (H * x)
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()
    x = x + (K * y)
    P = (I - (K * H)) * P
    return x, P

from filterpy.stats import plot_covariance_ellipse
from numpy.random import randn
import math

def plot_position_variance(x,P,edgecolor='r'):
    x1= np.array([x.value[0][0],x.value[1][0]])
    P1= np.array([[P.value[0][0],P.value[0][1]],[P.value[1][0],P.value[1][1]]])
    plot_covariance_ellipse(x1, P1, edgecolor=edgecolor)


def generate_experiment_data(z_var, process_var, count=1, dt=1.):
    "returns track, measurements 2D arrays"
    x, y, vel = 1.,1., 1.
    z_std = math.sqrt(z_var)
    p_std = math.sqrt(process_var)
    xs, zs = [], []
    for _ in range(count):
        v = vel + (randn() * p_std)
        x += v*dt
        y += v*dt
        xs.append([x,y])
        zs.append([x + randn() * z_std,x + randn() * z_std])
    return xs, zs

"""simulate a series of measurements in (x,y) pairs"""
measurements = [[2., 10.], [4., 8.], [6., 6.], [8., 5.], [10., 4.], [12., 2.]] #set of x,y measurements

initial_xy = [2., 9.]  #initial belief about position
dt = .1 #time interval between predictions
#truth, measurements = generate_experiment_data( 0., 0.1,count=50,dt=dt)
#print(measurements[49])


"""create the initial state vector"""
x = matrix([[initial_xy[0]], [initial_xy[1]], [0.], [0.]]) # initial state
u = matrix([[0.], [0.], [0.], [0.]]) # no external motion

"""model assumptions"""
# initial uncertainty: 10 for positions x and y, 1000 for the two velocities
P = matrix([[10.,0.,0.,0.],[0.,10.,0.,0.],[0.,0.,1000.,0.],[0.,0.,0.,1000.]])
#  state transition function
F = matrix([[1., 0.,dt,0.], [0., 1.,0.,dt],[0.,0.,1.,0.],[0.,0.,0.,1.]])
# measurement function which converts predicted state to measurement space
H = matrix([[1., 0.,0.,0.],[0.,1.,0.,0.]])
R = matrix([[.1,0.],[0.,0.1]])  # measurement noise
I = matrix([[1., 0.,0.,0.], [0., 1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]])

"""iterate through the measurements with a series of prediction and update steps"""
plot_position_variance(x,P,edgecolor='r')  #plot initial position and covariance in red
for z in measurements:
    x,P = predict(x, P)
    x,P = update(x, P,z)
    plot_position_variance(x,P,edgecolor='b') #plot updates in blue
print(x)
print(P)


"""Final state vector: [[11.965167916928769], [2.0760203171503386], [19.84771525945677], [-15.01535052717562]]
    Final state covariance: [[0.05206973927575543, 0.0, 0.14139384664494653, 0.0], [0.0, 0.05206973927575543, 0.0, 0.14139384664494653], [0.14139384664494653, 0.0, 0.5642609683506797, 0.0], [0.0, 0.14139384664494653, 0.0, 0.5642609683506797]]"""
