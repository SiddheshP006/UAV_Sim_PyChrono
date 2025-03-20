# -*- coding: utf-8 -*-
"""
Created on Wed Mar 19 22:59:23 2025

@author: Melen
"""

class rk4:
  def __init__(self):
    print("Integrator CLASS")
  
  def rk4singlestep(fun, dt, t0, y0, params):
      """
      This function does a single 4th-order Runge-Kutta integration step.
  
      Parameters
      ----------
      fun : TYPE
          ODE.
      dt : TYPE
          timestep.
      t0 : TYPE
          current initial time.
      y0 : TYPE
          current initial condition.
      params : TYPE
          parameters to be passed to fun
  
      Returns
      -------
      yout : TYPE
          result of the ODE.
  
      """
      
      f1 = fun(t0, y0, params)
      f2 = fun(t0 + dt /2, y0 + (dt / 2) * f1, params)
      f3 = fun(t0 + dt /2, y0 + (dt / 2) * f2, params)
      f4 = fun(t0 + dt, y0 + dt * f3, params)
      yout = y0 + (dt / 6) * (f1 + 2 * f2 + 2 * f3 + f4)
      return yout