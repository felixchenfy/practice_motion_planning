'''
A class based on "scipy.interpolate"
    for getting smoothed trajectory function. 
'''
import numpy as np
import scipy
import scipy.interpolate
import matplotlib.pyplot as plt
import sys
#https://docs.scipy.org/doc/scipy/reference/tutorial/interpolate.html#d-example

def calc_dist(x1,y1,x2,y2):
    return np.sqrt( (x1-x2)**2+(y1-y2)**2)

class Bezier(object):
    def __init__(self):
        return
    # B-spline

class Spline(object):
    def __init__(self, xl0, yl0, T, t_prec=0.01, method="CubicSpline"):

        # check input method
        methods={"interp1d", "CubicSpline","InterpolatedUnivariateSpline","Rbf"}
        if method not in methods:
            print("In Spline __init__, wrong method")
            sys.exit(0)
        else:
            if method=="interp1d":
                spline_func=scipy.interpolate.interp1d
            elif method=="CubicSpline":
                spline_func=scipy.interpolate.CubicSpline
            elif method=="InterpolatedUnivariateSpline":
                spline_func=scipy.interpolate.InterpolatedUnivariateSpline            
            elif method=="Rbf":
                spline_func=scipy.interpolate.Rbf            

        # check input points
        xl0=1.0*np.array(xl0) # ori x positions
        yl0=1.0*np.array(yl0) # ori y positions
        T=float(T) # total time of traversing the points
        (self.xl0, self.yl0, self.T)=(
            xl0, yl0, T
        )

        # set time cost of traversing the points
        # the time cost between two points are based on the their distance
        N = len(xl0)-1
        dist_list=[]
        tl0=[0,]
        for i in range(N):
            dist=calc_dist(xl0[i],yl0[i],xl0[i+1],yl0[i+1])
            dist_list.append(dist)
            tl0.append(tl0[-1]+dist)
        tl0=1.0*np.array(tl0)/tl0[-1]*T # tl0=[0, dist_between_points, T]
        (self.N, self.dist_list, self.tl0)=(
            N, dist_list, tl0
        )

        # calc the spline function
        fxt_ = spline_func(tl0, xl0)
        fyt_ = spline_func(tl0, yl0)        
        (self.spline_func, self.fxt_, self.fyt_)=(
            spline_func, fxt_, fyt_
        )

         # compute discrete traj
         # t_prec: time interval of generating discrete trajectory
        if t_prec is not None: # compute discrete traj
            self.compute_discrete_traj(t_prec)
   

    def fxt(self, t):
        try:
            t=[min(ti, self.T) for ti in t]
        except:
            t=min(t,self.T)
        return self.fxt_(t)

    def fyt(self, t):
        try:
            t=[min(ti, self.T) for ti in t]
        except:
            t=min(t,self.T)
        return self.fyt_(t)
    
    def resample_by_speed(self):
        # this is what I can use in chapter 9: point 2 point traj
        # Polynomial Time Scaling(cubic, quintic),  Trapezoidal Motion Profiles, S-Curve Time Scalings
        return

    def compute_discrete_traj(self, t_prec):
        # generate discrete traj
        self.t_prec=t_prec
        self.tl = np.linspace(0, self.T, self.T/self.t_prec)
        self.xl = self.fxt(self.tl)
        self.yl = self.fyt(self.tl)

    def plot_spline_result(self, t_prec=0.01):
        if self.t_prec is None: # if had not been computed
            self.compute_discrete_traj(t_prec)
        plt.plot(self.xl, self.yl, 'g.-')
        plt.plot(self.xl0, self.yl0, 'ro')
        plt.title('Spline Result')

if __name__=="__main__":
    # setup data
    T=10 # total time to traverse through the trajectory
    xl0 = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
    yl0 = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0]
    t_prec=0.05
    
    
    # spline=Spline(xl0, yl0, T, t_prec, method="interp1d")
    spline=Spline(xl0, yl0, T, t_prec, method="CubicSpline")

    spline.plot_spline_result()
    plt.show()
    # print spline.xl