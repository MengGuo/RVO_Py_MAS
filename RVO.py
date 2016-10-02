from math import ceil, floor, sqrt
import copy
import numpy

from math import cos, sin, tan, atan2, asin

from math import pi as PI



def distance(pose1, pose2):
    """ compute Euclidean distance for 2D """
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001


def RVO_update(X, V_des, V_current, ws_model):
    """ compute best velocity given the desired velocity, current velocity and workspace model"""
    ROB_RAD = ws_model['robot_radius']+0.1
    V_opt = list(V_current)    
    for i in range(len(X)):
        # print '--------'
        # print 'Agent', i
        # print 'loc', X[i]
        # print '--------'
        vA = [V_current[i][0], V_current[i][1]]
        pA = [X[i][0], X[i][1]]
        RVO_BA_all = []
        for j in range(len(X)):
            if i!=j:
                vB = [V_current[j][0], V_current[j][1]]
                pB = [X[j][0], X[j][1]]
                #
                transl_vB_vA = [pA[0]+0.5*(vB[0]+vA[0]), pA[1]+0.5*(vB[1]+vA[1])]
                #transl_vB_vA = [pA[0]+vB[0], pA[1]+vB[1]]
                dist_BA = distance(pA, pB)
                theta_BA = atan2(pB[1]-pA[1], pB[0]-pA[0])
                # print '----------------------------------------'
                # print 'i, j', (i,j) 
                # print 'RR, DD : %s %s' %(str(2*ROB_RAD), str(dist_BA))
                # print '----------------------------------------'
                if 2*ROB_RAD > dist_BA:
                    dist_BA = 2*ROB_RAD
                theta_BAort = asin(2*ROB_RAD/dist_BA)
                theta_ort_left = theta_BA+theta_BAort
                bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
                theta_ort_right = theta_BA-theta_BAort
                bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
                RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, 2*ROB_RAD]
                RVO_BA_all.append(RVO_BA)                
        for hole in ws_model['circular_obstacles']:
            # hole = [x, y, rad]
            vB = [0, 0]
            pB = hole[0:2]
            #tansl_vB_vA = [pA[0]+0.5*(vB[0]+vA[0]), pA[1]+0.5*(vB[1]+vA[1])]
            transl_vB_vA = [pA[0]+vB[0], pA[1]+vB[1]]
            dist_BA = distance(pA, pB)
            theta_BA = atan2(pB[1]-pA[1], pB[0]-pA[0])
            # over-approximation of square to circular
            OVER_APPROX_C2S = 1.5
            rad = hole[2]*OVER_APPROX_C2S
            if (rad+ROB_RAD) > dist_BA:
                dist_BA = rad+ROB_RAD
            theta_BAort = asin((rad+ROB_RAD)/dist_BA)
            theta_ort_left = theta_BA+theta_BAort
            bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
            theta_ort_right = theta_BA-theta_BAort
            bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
            RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, rad+ROB_RAD]
            RVO_BA_all.append(RVO_BA)
        vA_post = intersect(pA, V_des[i], RVO_BA_all)
        V_opt[i] = vA_post[:]
    return V_opt


def intersect(pA, vA, RVO_BA_all):
    # print '----------------------------------------'
    # print 'Start intersect test'
    # print '----------------------------------------'
    norm_v = distance(vA, [0, 0])
    # print 'vA, norm_v', (vA, norm_v)
    suitable_V = []
    unsuitable_V = []
    for theta in numpy.arange(0, 2*PI, 0.1):
        for rad in numpy.arange(0.02, norm_v+0.02, norm_v/5.0):
            #rad = norm_v
            new_v = [rad*cos(theta), rad*sin(theta)]
            suit = True
            # print '----------------------------------------'
            # print 'Test new_v', new_v
            # print 'theta', theta
            # print 'rad', rad
            # print '----------------------------------------'
            for RVO_BA in RVO_BA_all:
                # print 'Try RVO_BA', RVO_BA
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
                dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
                theta_dif = atan2(dif[1], dif[0])
                theta_right = atan2(right[1], right[0])
                theta_left = atan2(left[1], left[0])
                # print 'thetas', (theta_right, theta_dif, theta_left)
                if in_between(theta_right, theta_dif, theta_left):
                    suit = False
                    # print 'in between'
                    break
            if suit:
                suitable_V.append(new_v)
                # print 'suitable'
            else:
                unsuitable_V.append(new_v)
                # print 'unsuitable'
                
    #----------------------                
    # add vA
    new_v = vA[:]
    suit = True
    for RVO_BA in RVO_BA_all:                
        p_0 = RVO_BA[0]
        left = RVO_BA[1]
        right = RVO_BA[2]
        dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
        theta_dif = atan2(dif[1], dif[0])
        theta_right = atan2(right[1], right[0])
        theta_left = atan2(left[1], left[0])
        if in_between(theta_right, theta_dif, theta_left):
            suit = False
            break
    if suit:
        suitable_V.append(new_v)
    else:
        unsuitable_V.append(new_v)
    #----------------------        
    #----------------------
    if suitable_V:
        #print 'Suitable found'
        vA_post = min(suitable_V, key = lambda v: distance(v, vA))
        # #----------------------------------------
        # for v in suitable_V:
        #     print 'v, distance(v, VA)', (v, distance(v, vA))
        # #----------------------------------------
        new_v = vA_post[:]
        for RVO_BA in RVO_BA_all:
            # print 'RVO_BA', RVO_BA
            p_0 = RVO_BA[0]
            left = RVO_BA[1]
            right = RVO_BA[2]
            dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
            theta_dif = atan2(dif[1], dif[0])
            theta_right = atan2(right[1], right[0])
            theta_left = atan2(left[1], left[0])
            # print 'thetas', (theta_right, theta_dif, theta_left)
    else:
        # print 'Suitable not found'
        tc_V = dict()
        for unsuit_v in unsuitable_V:
            tc_V[tuple(unsuit_v)] = 0
            tc = []
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
                dist = RVO_BA[3]
                rad = RVO_BA[4]
                dif = [unsuit_v[0]+pA[0]-p_0[0], unsuit_v[1]+pA[1]-p_0[1]]
                theta_dif = atan2(dif[1], dif[0])
                theta_right = atan2(right[1], right[0])
                theta_left = atan2(left[1], left[0])
                if in_between(theta_right, theta_dif, theta_left):
                    small_theta = abs(theta_dif-0.5*(theta_left+theta_right))
                    # print '----------------------------------------'
                    # print 'RVO_BA', RVO_BA
                    # print 'dist*sin(small_theta), rad : %s, %s' %(str(dist*sin(small_theta)), str(rad))
                    # print '----------------------------------------'
                    if abs(dist*sin(small_theta)) >= rad:
                        rad = abs(dist*sin(small_theta))
                    big_theta = asin(abs(dist*sin(small_theta))/rad)
                    dist_tg = abs(dist*cos(small_theta))-abs(rad*cos(big_theta))
                    if dist_tg < 0:
                        dist_tg = 0                    
                    tc_v = dist_tg/distance(dif, [0,0])
                    tc.append(tc_v)
            tc_V[tuple(unsuit_v)] = min(tc)+0.001
        WT = 0.2
        vA_post = min(unsuitable_V, key = lambda v: ((WT/tc_V[tuple(v)])+distance(v, vA)))
    #     print 'tc_V[vA_post]', tc_V[tuple(vA_post)]
    #     print 'tc_V[vA]', tc_V[tuple(vA)]
    # print 'vA', vA
    # print 'vA_post', vA_post
    return vA_post 

def in_between(theta_right, theta_dif, theta_left):
    if abs(theta_right - theta_left) <= PI:
        if theta_right <= theta_dif <= theta_left:
            return True
        else:
            return False
    else:
        if (theta_left <0) and (theta_right >0):
            theta_left += 2*PI
            if theta_dif < 0:
                theta_dif += 2*PI
            if theta_right <= theta_dif <= theta_left:
                return True
            else:
                return False
        if (theta_left >0) and (theta_right <0):
            theta_right += 2*PI
            if theta_dif < 0:
                theta_dif += 2*PI
            if theta_left <= theta_dif <= theta_right:
                return True
            else:
                return False

def compute_V_des(X, goal, V_max):
    V_des = []
    for i in xrange(len(X)):
        dif_x = [goal[i][k]-X[i][k] for k in xrange(2)]
        norm = distance(dif_x, [0, 0])
        norm_dif_x = [dif_x[k]*V_max[k]/norm for k in xrange(2)]
        V_des.append(norm_dif_x[:])
    return V_des
            
def reach(p1, p2, bound=0.5):
    if distance(p1,p2)< bound:
        return True
    else:
        return False
    
    
