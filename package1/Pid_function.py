# -*- coding: utf-8 -*-
"""
Created on Wed Mar 19 22:59:23 2025

@author: Melen
"""

class rk4:
  def __init__(self):
    print("Integrator CLASS")
  
<<<<<<< HEAD
  def rk4singlestep(fun, dt, t0, y0, ode_instance):
=======
  def rk4singlestep(fun, dt, t0, y0, gains_instances,odeInputs):
>>>>>>> 76203d90eb62a290989470b76a8caea064658ae1
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
      ode_instance : TYPE
          parameters to be passed to fun
  
      Returns
      -------
      yout : TYPE
          result of the ODE.
  
      """
      
<<<<<<< HEAD
      f1 = fun(t0, y0, ode_instance)
      f2 = fun(t0 + dt /2, y0 + (dt / 2) * f1, ode_instance)
      f3 = fun(t0 + dt /2, y0 + (dt / 2) * f2, ode_instance)
      f4 = fun(t0 + dt, y0 + dt * f3, ode_instance)
=======
      f1 = fun(t0, y0, gains_instances,odeInputs)
      f2 = fun(t0 + dt /2, y0 + (dt / 2) * f1, gains_instances,odeInputs)
      f3 = fun(t0 + dt /2, y0 + (dt / 2) * f2, gains_instances,odeInputs)
      f4 = fun(t0 + dt, y0 + dt * f3, gains_instances,odeInputs)
>>>>>>> 76203d90eb62a290989470b76a8caea064658ae1
      yout = y0 + (dt / 6) * (f1 + 2 * f2 + 2 * f3 + f4)
      return yout