#! /usr/bin/env python
from sympy import symbols, IndexedBase, Idx
import sympy as sp
import numpy as np
from scipy.integrate import odeint
#from scipy.misc import derivative
from sympy import *

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

class DiscretizeandLinearizeGeneric():
  def __init__(self,Z,xhat,uhat,n):
    #matrix A of states linearize
        self.Aj = np.zeros(shape= (x[n_states].shape[0],x[n_states].shape[0]))     
        #self.Aj[n_states,n_states]
        #matrix B of input linearize
        self.Bj = np.zeros(shape= (x[n_states].shape[0],u[n_in].shape[0]))
        #point around that we will make the linearization 
        self.xhat = xhat[:]
        self.uhat = uhat[:]
        #equation of states (ODE)
        self.Z = Z
        #matrix of the difference between the real equation and the linearize one
        self.p = []
        #matrix of the discretize system
        self.Ad=[]
        self.Bd=[]
        self.Cd=[]
        #list of matrix discrete to make the convex opt
        self.Ad_list = [None]*(n-1)
        self.Bd_list = [None]*(n-1)
        self.Cd_list = [None]*(n-1)

  def evaluatePerte(self):
      dxdt = self.Z[:]
      #linearize the A matrix with symbols
      xhat = self.xhat
      uhat = self.uhat
      #evaluate the real system around the point xhat,uhat
      for c in range(len(dxdt)):
        for i in range(len(xhat)):
            dxdt[c] = dxdt[c].subs(x[i],xhat[i])
        for i in range(len(uhat)):
            dxdt[c]= dxdt[c].subs(u[i],uhat[i])
      #save the result inside a matrix
      #p will be the waste of the linearize system
      self.p = -self.p + dxdt[:]

      #class that calculate the matrix A and B jacobien and make the evaluation around the points xhat and uhat 
  def JacobianAndEvaluate(self):
      dxdt = self.Z[:]
      #linearize the A matrix with symbols
      xhat = self.xhat
      uhat = self.uhat
      #linearize the A matrix with symbols
      for c in range (len(dxdt)):
        for h in range (x[n_states].shape[0]):
          #calculate the derivative of the c ode of the x[h] variable of states
          A = sp.diff(dxdt[c],x[h])
          for i in range(len(xhat)):
            A = A.subs(x[i],xhat[i])
          for i in range(len(uhat)):
            A = A.subs(u[i],uhat[i])
          self.Aj[c][h] = A
      #self.Aj = np.array([[sp.diff(dx1dt,x[0]),sp.diff(dx1dt,x[1]),sp.diff(dx1dt,x[2])],[sp.diff(dx2dt,x[0]),sp.diff(dx2dt,x[1]),sp.diff(dx2dt,x[2])],[sp.diff(dx3dt,x[0]),sp.diff(dx3dt,x[1]),sp.diff(dx3dt,x[2])]])
      #linearize B matrix with symbols
      for c in range (len(dxdt)):
        for h in range (u[n_in].shape[0]):
          #calculate the B matrix making the derivative of ode respect to the inputs
          B = sp.diff(dxdt[c],u[h])
          for i in range(len(xhat)):
            B = B.subs(x[i],xhat[i])
          for i in range(len(uhat)):
            B = B.subs(u[i],uhat[i])
          self.Bj[c][h] = B
      #self.Bj = np.array([[sp.diff(dx1dt,u[0]),sp.diff(dx1dt,u[1])],[sp.diff(dx2dt,u[0]),sp.diff(dx2dt,u[1])],[sp.diff(dx3dt,u[0]),sp.diff(dx3dt,u[1])]])
      #print(self.Bj)
      #pass the information of the evaluation of the equation to evaluate the waste between the linear and non linear system
      self.p = np.dot(self.Bj,uhat) + np.dot(self.Aj,xhat)
      self.evaluatePerte()

  def mod_point(self,xhat,uhat):
    #calculate the matrix of the system to make the discretization 
        self.xhat = xhat[:]
        self.uhat = uhat[:]
        self.JacobianAndEvaluate()
  
  def lin2disc(self,xhat,uhat,n,dt):
        #discretize the system 
        self.mod_point(xhat,uhat)
        resolution = 100
        Adx_list = [None] * (resolution+1)
        Adr_list = [None] * (resolution+1)
        Adx_list[0] = np.eye(x[n_states].shape[0])
        Adr_list[0] = np.eye(x[n_states].shape[0])
        delta=dt/resolution
        for i in range(resolution):
          Adx_list[i+1] = Adx_list[i] + np.dot(Adx_list[i],self.Aj)*delta
          Adr_list[i+1] = Adr_list[i] - np.dot(Adr_list[i],self.Aj)*delta
        self.Ad = Adx_list[resolution]
        self.Bd = np.zeros([x[n_states].shape[0],u[n_in].shape[0]])
        self.Cd = np.zeros([x[n_states].shape[0],])
        #self.p = np.reshape(self.p,(3,1))
        #print(self.p)
        for i in range(resolution):
          Ard = Adr_list[i+1]
          self.Bd = self.Bd + (np.dot(Ard,self.Bj)*delta)
          self.Cd = self.Cd + (np.dot(Ard,self.p)*delta)
        self.Bd = np.dot(self.Ad,self.Bd)
        self.Cd = np.dot(self.Ad,self.Cd)
        #print(self.Ad)
        #print(self.Bd)
        #print(self.Cd)
        return self.Ad,self.Bd,self.Cd


  def disc(self,uw,n,dt,x_ss):
    #evaluate the system discrete 
      tf = dt*n 
      t = np.linspace(0,tf,n)
        # store solution
      xk = np.zeros(shape =(x[n_states].shape[0],n))
      # record initial conditions
      xk[:,0] = x_ss
      for i in range(1,n):
          u0 = uw[:,i-1]
          x0 = xk[:,i-1]
          #calculate the discrete matrix around the points x0 and u0
          self.Ad_list[i-1],self.Bd_list[i-1],self.Cd_list[i-1] = self.lin2disc(x0,u0,n,dt)
          #store solutions
          xk[:,i] = np.dot(self.Ad,x0)+np.dot(self.Bd,u0) + self.Cd 
      return xk 
      
  def get_list(self):
    return np.array(self.Ad_list).astype(np.float64),np.array(self.Bd_list).astype(np.float64),np.array(self.Cd_list).astype(np.float64)
