#! /usr/bin/env python
import cvxpy as cp
import numpy as np
from sympy import symbols, IndexedBase, Idx
#let's define the variables of the class (u inputs and x states)
u = IndexedBase('u')
n_in = symbols('n_in ', integer=True)
u[n_in]
#you can change the number of input but not the name
n_in = Idx('n_in', 2)
x = IndexedBase('x')
n_states = symbols('n_states', integer=True)
x[n_states]
#You can change the number of states not the name
n_states = Idx('n_states', 3)
class ConvexOpt():
  def __init__(self,N,x_init,x_fin,u_in,A_list,B_list,C_list):
    #init the variables of the class
    self.N = N
    self.x_init = x_init
    self.x_fin = x_fin
    self.u_in = u_in
    self.Ad_list = A_list
    self.Bd_list = B_list
    self.Cd_list = C_list
    
  def CVXOPT(self,opt_power = False,opt_velocity = False):
    #save the number of states and inputs
    x_len =(int) (x[n_states].shape[0])
    u_len = (int) (u[n_in].shape[0])
    #define the variables to be evaluate 
    xv = cp.Variable(shape=(x_len, self.N))
    uv = cp.Variable((u_len, self.N-1))
    tau = cp.Variable(shape=(self.N))
    tau_vel = cp.Variable(shape=(self.N))
    tau_u = cp.Variable(shape=(self.N-1))
    hogb = cp.Variable(self.N-1)
    hog = cp.Variable(shape = (x_len,self.N-1))
    #define the objective of the convex optimization 
    obj = cp.sum_squares(np.ones(shape=(1,self.N))*tau +np.ones(shape=(1,self.N))*tau_vel + 10*np.ones(shape=(1,self.N-1))*tau_u  + np.ones(shape=(1,self.N-1))*10**3*hogb)
    obj = cp.Minimize(obj)
    #define all constrains to be take into account but they have to be convex 
    constr = []
    #initial condition for x-y position and angular position
    constr += [xv[:,0] == self.x_init]
    #initial condition related to inputs 
    constr += [uv[:,0] == self.u_in]
    #final position constrain 
    constr += [cp.norm(xv[:,self.N-1] - self.x_fin) <= 10e-9]
    #trajectory limitation 
    for t in range(0,self.N-1):
      #discrete trajectory with virtual control 
        constr += [ xv[:,t+1] == self.Ad_list[t]*xv[:,t] + self.Bd_list[t] * uv[:,t] + self.Cd_list[t]]
        #norm(hog(:,k)) <= hogb(k)
        constr += [cp.norm(hog[:,t]) <= hogb[t]]

    #take into account only the shortest trajectory 
        constr += [cp.norm(xv[:,t-1] - xv[:,t]) <= tau[t]]

        #I tried to code linear obstacle but  working only in rectangular case  
        #constr += [xv[1,t] <= 6]
        #constr += [xv[1,t] >= 0]
        #constr += [xv[0,t] <= 5]
        #constr += [xv[0,t] >= 0]
        #non convex 
        #constr += [cp.norm2(xv[0,t] - 1) >= 1]
    
    #limit the final velocity    
    constr += [cp.norm(uv[:,self.N-2]) <= 10e-9]
    #contrain of the velocity of convergence to the final point % ||target - x_k||_2 <= taui_k
    if(opt_velocity):
      for t in range (0,self.N-1):
        constr += [cp.norm2(xv[:,t] - self.x_fin)<= tau_vel[t]]
    #constrain to optimize the power efficency related to the norm of u
    if(opt_power):
      for t in range (0,self.N-2):
        constr += [cp.norm(uv[:,t]) <= tau_u[t]]

    #resolve the problem 
    prob = cp.Problem(obj , constr)
    prob.solve(verbose=True)
    xv = np.array(xv.value)
    uv = np.array(uv.value)
    return xv,uv