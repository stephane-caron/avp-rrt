#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2012 Quang-Cuong Pham <cuong.pham@normalesup.org>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.



"""
Kinodynamic planning in the configuration space
"""



from numpy import *
from pylab import *
import bisect
import MintimeProfileIntegrator
import MintimeTrajectory
import time

KINO_DEBUG = False

#----------- Robot-specific ------------#
import MintimeProblemTorque
#import MintimeProblemBottle
#---------------------------------------#




#################################################################
############## Velocity Propagation Algorithm ###################
#################################################################


def propagateVelocityRange(robot,traj,tunings,v_beg_min,v_beg_max):

    t_step=tunings.t_step


    #----------- Robot-specific ------------#
    pb=MintimeProblemTorque.MintimeProblemTorque(robot,traj)
    #pb=MintimeProblemBottle.MintimeProblemBottle(robot,traj)
    #---------------------------------------#
    pb.set_dynamics_limits(tunings.limits)


    pb.disc_thr=tunings.disc_thr
    pb.preprocess()

    if min(pb.maxvel_accel_curve)<1e-4:
        main_status="MV-MaxvelCurveTouchedBottom"
        return [-1,-1,None,main_status]


    algo=MintimeProfileIntegrator.MintimeProfileIntegrator(pb)
    algo.dt_integ=tunings.dt_integ
    algo.width=tunings.width
    algo.palier=tunings.palier
    algo.tolerance_ends=tunings.tolerance_ends



    ######################################################
    # Compute the limiting curves

    status=algo.compute_limiting_curves()

    if status=="CouldnotCrossSwitch/T" or status=="CouldnotCrossSwitch/Z" or status=="TouchedBottom":
        main_status="LC-"+status
        return [-1,-1,algo,main_status]

    [i,bound]=algo.compute_index(1e-2,algo.s_traj_list,algo.sdot_traj_list)
    bound=min(bound,pb.maxvel_curve[0])



    #############################################################
    # Compute v_end_max by integrating forward from (0,v_beg_max)

    if v_beg_min>bound:
        main_status="FW-LimitingCurvesBelowVBegMin"
        return [-1,-1,algo,main_status]

    v_beg_max=min(v_beg_max,bound)

    [s_forward,sdot_forward,status]=algo.integrate_forward(0,v_beg_max,test_list=True)
    algo.s_traj_list.append(s_forward)
    algo.sdot_traj_list.append(sdot_forward)
    if status=="TouchedBottom" or status=="CrossedMaxvel":
        main_status="FW-"+status
        return [-1,-1,algo,main_status]
    if status=="ReachedEnd":
        v_end_max=sdot_forward[-1]
    elif status=="CrossedBwTraj":
        if len(algo.s_traj_list)>0 and  algo.s_traj_list[-1][-1]>algo.pb.duration-algo.tolerance_ends:
            v_end_max=algo.sdot_traj_list[-1][-1]
        else:
            v_end_max=algo.pb.maxvel_curve[-1]
            [s_backward,sdot_backward,status]=algo.integrate_backward(algo.pb.duration,v_end_max,test_list=True)
            if status=="TouchedBottom" or status=="CrossedMaxvel":
                main_status="BW-"+status
                return [-1,-1,algo,main_status]
            if status=="ReachedBeginning" and sdot_backward[0]<v_beg_min :
                main_status="BW-BelowVBegMin"
                return [-1,-1,algo,main_status]



    ###########################################################
    # Compute v_end_min by integrating backward from the end

    # Case when v_end_min=0 is OK
    [s_backward,sdot_backward,status]=algo.integrate_backward(algo.pb.duration,0,test_list=True)
    if (status=="ReachedBeginning" and sdot_backward[0]>=v_beg_min) or status=="CrossedFwTraj":
        main_status="OK"
        return [0,v_end_max,algo,main_status]

    # Case when v_end_min=0 is not OK
    dicho_steps=tunings.dicho_steps
    v_upper=v_end_max
    v_lower=0
    # We maintain the invariant that v_upper is OK but v_lower is not OK
    while dicho_steps>=0:
        dicho_steps-=1
        v_test=(v_upper+v_lower)/2
        [s_backward,sdot_backward,status]=algo.integrate_backward(algo.pb.duration,v_test,test_list=True)
        if (status=="ReachedBeginning" and sdot_backward[0]>=v_beg_min) or status=="CrossedFwTraj":
            v_upper=v_test
        else:
            v_lower=v_test


    main_status="OK"
    return [v_upper,v_end_max,algo,main_status]





##############################################################
############################ Bi_CKRRT ########################
##############################################################


def Bi_CKRRT(robot,q_start,v_start,q_target,v_target,q_range,K,tunings,bidir=True,samples_list=None,plotting=False,dirty_backref=None):
    if plotting:
        plot(q_start[0],q_start[1],'ro',markersize=10)
        plot(q_target[0],q_target[1],'go',markersize=10)

    # Vertex(config, parent, traj_from_parent, v_min, v_max)
    root=Vertex(q_start,None,None,0,1e-4)
    root.iterstep = 0  # dirty dirty dirty

    dirty_backref.trace.append({
        'q': root.config,
        'parent': None,
        'v_min': root.v_min,
        'v_max': root.v_max,
        'iterstep': root.iterstep,
        'timestamp': time.time()
    })

    tip=Vertex(q_target,None,None,0,1e-4)
    ver_list_start=[root]
    ver_list_target=[tip]

    l=try_connect(robot,tunings,root,q_target,v_target)
    if l[0]:
        return l


    for k in range(K):
        if dirty_backref:
            dirty_backref.nb_ext += 1
        if samples_list is None:
            q_rand=array([(rand()-0.5)*(v[1]-v[0])+mean(v) for v in q_range])
        else:
            q_rand=samples_list[k]
        if KINO_DEBUG:
            print "\n\n\n"
            print "**************************************************"
            print "**************************************************"
            print "Tentative vertex "+str(k)
            print q_rand

        if tunings.checkcoll and MintimeProblemBottle.CheckCollisionConfig(robot.robotd,q_rand):
            if KINO_DEBUG:
                print 'Random config collides'
            continue

        # Try to connect the random config to start tree
        if KINO_DEBUG:
            print "\n---- Try to connect to start tree ("+str(len(ver_list_start))+" vertices) ----"
        [new_ver_start,algo]=extend(robot,tunings,ver_list_start,q_rand)
        if new_ver_start!=None:
            ver_list_start.append(new_ver_start)
            if dirty_backref:
                dirty_backref.nb_succ_ext += 1
                iterstep = dirty_backref.nb_succ_ext
                vertex = new_ver_start
                vertex.iterstep = iterstep  # put directly into the vertex object (dirty)
                dirty_backref.trace.append({
                    'q': vertex.config,
                    'parent': vertex.parent.iterstep if vertex.parent else None,
                    'v_min': vertex.v_min,
                    'v_max': vertex.v_max,
                    'iterstep': iterstep,
                    'timestamp': time.time()
                })
            traj=new_ver_start.cord.traj
            if plotting:
                plot(traj.q_vect[0,:],traj.q_vect[1,:],'r')
                plot(q_rand[0],q_rand[1],'ko',markersize=5)
                axis([-4,4,-4,4])
                grid(True)
                draw()
            #MintimeProblemBottle.PlotTraj(traj,0.1,0.3,0.11,sub=8,offset=0.05)
            # Check whether we can connect to the target from the new node
            if KINO_DEBUG:
                print "\n-- Try to connect to q_target"
            l_start=try_connect(robot,tunings,new_ver_start,q_target,v_target)
            if l_start[0]:
                if KINO_DEBUG:
                    print "Start tree was succesful!"
                l_start.append(new_ver_start)
                return [1,l_start[1],new_ver_start,ver_list_start,k]
            elif KINO_DEBUG:
                print "Could not connect to q_target"

        if not bidir:
            continue


        # Try to connect the random config to target tree
        if KINO_DEBUG:
            print "\n---- Try to connect to target tree ("+str(len(ver_list_target))+" vertices) ----"
        [new_ver_target,algo]=extend(robot,tunings,ver_list_target,q_rand)
        if new_ver_target!=None:
            ver_list_target.append(new_ver_target)
            traj=new_ver_target.cord.traj
            if plotting:
                plot(traj.q_vect[0,:],traj.q_vect[1,:],'g')
                plot(q_rand[0],q_rand[1],'ko',markersize=5)
            #MintimeProblemBottle.PlotTraj(traj,0.1,0.3,0.11,sub=8,offset=0.05)
            # Check whether we can connect to the start from the new node
            if KINO_DEBUG:
                print "\n-- Try to connect to q_start"
            l_target=try_connect(robot,tunings,new_ver_target,q_start,v_start)
            if l_target[0]:
                if KINO_DEBUG:
                    print "Target tree was succesful!"
                l_target.append(new_ver_target)
                return [2,l_target[1],new_ver_target,ver_list_target,k]
            elif KINO_DEBUG:
                print "Could not connect to q_start"


        #Can the node be common?
        if new_ver_start!=None and new_ver_target!=None:
            if new_ver_start.v_min<1e-4 and new_ver_target.v_min<1e-4:
                if KINO_DEBUG:
                    print "Start tree and target tree connected!"
                return [3,new_ver_start,new_ver_target,ver_list_start,ver_list_target,k]


    return [-1,ver_list_start,ver_list_target]





##############################################################
######################## Extensions ##########################
##############################################################



# Interpolate a polynomial
def interp(x0,v0,x1,T):
    c=x0
    b=v0
    a=(x1-c-b*T)/T**2.
    return poly1d([a,b,c])


# Generate a path
def generate_path(ver,q,tunings,zero_vel=False):
    T=norm(q-ver.config)
    if ver.parent==None or zero_vel:
        cord_v=(q-ver.config)/T
    else:
        cord_v=ver.cord_final_velocity
    P_list=[]
    for i in range(len(q)):
        P_list.append(interp(ver.config[i],cord_v[i],q[i],T))
    pwp_traj=MintimeTrajectory.PieceWisePolyTrajectory([P_list],[T])
    return [pwp_traj.GetSampleTraj(T,tunings.t_step),P_list,T]



# Extend to a new config from a vertex
def extend_ver(robot,tunings,ver,q):

    # Try first with zero velocity
    #----------- Robot-specific ------------#
    if tunings.try_zero and ver.v_min<1e-4: # Pendulum
    #if tunings.try_zero and abs(tan(ver.config[2]))<robot.mu and ver.v_min<1e-4: #Bottle
    #----------- Robot-specific ------------#
        [traj,P_list,T]=generate_path(ver,q,tunings,zero_vel=True)
        if (not tunings.checkcoll) or (not MintimeProblemBottle.CheckCollisionTraj(robot.robotd,traj)[0]):
            v_beg_min=0
            v_beg_max=1e-4
            [v_end_min,v_end_max,algo,status]=propagateVelocityRange(robot,traj,tunings,v_beg_min,v_beg_max)
            if v_end_max>0:
                if KINO_DEBUG:
                    print "OK/zero "+str([v_end_min,v_end_max])
                new_cord=Cord(traj,P_list,T)
                new_node=Vertex(q,ver,new_cord,v_end_min,v_end_max)
                new_node.disc=True
                return [new_node,algo]
            elif KINO_DEBUG:
                print status
        elif KINO_DEBUG:
            print 'Collision'


    # Now try with a continuity-preserving path
    [traj,P_list,T]=generate_path(ver,q,tunings)
    if (not tunings.checkcoll) or (not MintimeProblemBottle.CheckCollisionTraj(robot.robotd,traj)[0]):
        [v_end_min,v_end_max,algo,status]=propagateVelocityRange(robot,traj,tunings,ver.v_min,ver.v_max)

        if v_end_max>0:
            if KINO_DEBUG:
                print "OK/cont "+str([v_end_min,v_end_max])
            new_cord=Cord(traj,P_list,T)
            new_node=Vertex(q,ver,new_cord,v_end_min,v_end_max)
            new_node.disc=False
            return [new_node,algo]
        else:
            if KINO_DEBUG:
                print status
            return [None,algo]
    else:
        if KINO_DEBUG:
            print 'Collision'
        return [None,None]



# Extend to a new config from the tree
def extend(robot,tunings,ver_list,q):
    # NB: norm used by VIP-RRT is L2
    dist_list=[norm(ver.config[0:2]-q[0:2]) for ver in ver_list]
    dist_list_sorted=sorted(dist_list)
    n_ver=len(ver_list)

    n_close=min(tunings.n_close,n_ver)

    for i in range(n_close):
        if KINO_DEBUG:
            print 'Trial '+str(i)
        d=dist_list_sorted[i]
        ver=ver_list[dist_list.index(d)]
        if KINO_DEBUG:
            print ver.config
            print [ver.v_min,ver.v_max]
        [new_node,algo]=extend_ver(robot,tunings,ver,q)
        if new_node!=None:
            return [new_node,algo]

    a=dist_list_sorted[n_close:n_ver]
    shuffle(a)

    for i in range(min(tunings.n_rand,len(a))):
        if KINO_DEBUG:
            print 'Trial '+str(i+n_close)
        d=dist_list_sorted[i]
        ver=ver_list[dist_list.index(d)]
        [new_node,algo]=extend_ver(robot,tunings,ver,q)
        if new_node!=None:
            return [new_node,algo]

    if KINO_DEBUG:
        print "\nCould not find any good neighbor"
    return [None,None]






#################################################################
######################## Connect to goal ########################
#################################################################



# Try to connect to config
def try_connect(robot,tunings,ver,q_target,v_target):

    # Try first with zero velocity
    #----------- Robot-specific ------------#
    if tunings.try_zero and ver.v_min<1e-4: #Pendulum
    #if tunings.try_zero and abs(tan(ver.config[2]))<robot.mu and ver.v_min<1e-4: #Bottle
    #----------- Robot-specific ------------#
        [traj,P_list,T]=generate_path(ver,q_target,tunings,zero_vel=True)
        if (not tunings.checkcoll) or (not MintimeProblemBottle.CheckCollisionTraj(robot.robotd,traj)[0]):
            v_beg_min=0
            v_beg_max=1e-4
            [v_end_min,v_end_max,algo,main_status]=propagateVelocityRange(robot,traj,tunings,v_beg_min,v_beg_max)
            if v_end_min<=v_target and v_target<=v_end_max:
                pwp_traj=MintimeTrajectory.PieceWisePolyTrajectory([P_list],[T])
                pwp_traj_list=trace_to_root(ver,tunings,[],[])
                pwp_traj_list.append(pwp_traj)
                return [True,pwp_traj_list]

    # Now try with a continuity-preserving path
    [traj,P_list,T]=generate_path(ver,q_target,tunings)
    if (not tunings.checkcoll) or (not MintimeProblemBottle.CheckCollisionTraj(robot.robotd,traj)[0]):
        [v_end_min,v_end_max,algo,main_status]=propagateVelocityRange(robot,traj,tunings,ver.v_min,ver.v_max)
        if v_end_min<=v_target and v_target<=v_end_max:
            pwp_traj_list=trace_to_root(ver,tunings,[P_list],[T])
            return [True,pwp_traj_list]
        else:
            return [False]
    else:
        return [False]



# Trace back to the tree's root
def trace_to_root(ver,tunings,cur_P_list_list,cur_T_list):
    if ver.parent==None:
        if len(cur_P_list_list)==0:
            return []
        else:
            return [MintimeTrajectory.PieceWisePolyTrajectory(cur_P_list_list,cur_T_list)]
    else:
        cur_P_list_list.insert(0,ver.cord.P_list)
        cur_T_list.insert(0,ver.cord.T)
        if ver.disc:
            pwp_traj=MintimeTrajectory.PieceWisePolyTrajectory(cur_P_list_list,cur_T_list)
            pwp_traj_list=trace_to_root(ver.parent,tunings,[],[])
            pwp_traj_list.append(pwp_traj)
            return pwp_traj_list
        else:
            return trace_to_root(ver.parent,tunings,cur_P_list_list,cur_T_list)






############################################################
######################## Utilities #########################
############################################################




# Tree definitions

class Vertex():
    def __init__(self,config,parent,cord,v_min,v_max):
        self.config=config
        self.parent=parent
        self.cord=cord
        self.v_min=v_min
        self.v_max=v_max
        if parent!=None:
            v=[polyval(polyder(p),cord.T) for p in cord.P_list]
            self.cord_final_velocity=v

class Cord():
    def __init__(self,traj,P_list,T):
        self.traj=traj
        self.P_list=P_list
        self.T=T


def find_nearest(ver_list,c):
    res=ver_list[0]
    d_max=norm(res.config-c)
    for ver in ver_list[1:]:
        d=norm(ver.config-c)
        if d<d_max:
            d_max=d
            res=ver
    return res


def plot_tree(ver_list,q_start,q_target,traj_list=None):
    for v in ver_list:
        if v.parent!=None:
            plot(v.config[0],v.config[1],'ro',markersize=8)
            plot(v.cord.traj.q_vect[0,:],v.cord.traj.q_vect[1,:],'r',linewidth=2)
    if traj_list!=None:
        [plot(traj.q_vect[0,:],traj.q_vect[1,:],'m',linewidth=2) for traj in traj_list]
        plot(q_start[0],q_start[1],'ko',markersize=10)
        plot(q_target[0],q_target[1],'go',markersize=10)

# Tunings

class Tunings():
    def __init__(self):
        pass



# Reverse trajectories

def reverse_time_poly2(p,T):
    if len(p.coeffs)==3:
        a=p.coeffs[0]
        b=p.coeffs[1]
        c=p.coeffs[2]
        return poly1d([a,-2*a*T-b,a*T*T+b*T+c])
    else:
        a=p.coeffs[0]
        b=p.coeffs[1]
        return poly1d([-a,a*T+b])


def reverse_pwp_traj(pwp_traj):
    pieces_list=[]
    durations_list=[]
    for i in range(pwp_traj.n_pieces):
        poly_list=[]
        T=pwp_traj.durations_list[i]
        for j in range(pwp_traj.dim):
            poly_list.append(reverse_time_poly2(pwp_traj.pieces_list[i][j],T))
        pieces_list.insert(0,poly_list)
        durations_list.insert(0,T)
    return MintimeTrajectory.PieceWisePolyTrajectory(pieces_list,durations_list)


def reverse_pwp_traj_list(pwp_traj_list):
    res=[]
    for pwp_traj in pwp_traj_list:
        res.insert(0,reverse_pwp_traj(pwp_traj))
    return res


