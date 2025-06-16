#!/usr/bin/env python3

import numpy as np
from scipy import interpolate
import trajectory_planning_helpers as tph
from scipy.signal import savgol_filter
from skimage.morphology import skeletonize
from skimage.segmentation import watershed
# from visualization_msgs.msg import *
# from geometry_msgs.msg import *
        # # interpolate centerline to 0.1m stepsize: less computation needed later for distance to track bounds
        # centerline_meter = np.column_stack((centerline_meter, np.zeros((centerline_meter.shape[0], 2))))

        # centerline_meter_int = helper_funcs_glob.src.interp_track.interp_track(reftrack=centerline_meter,
        #                                                                        stepsize_approx=0.1)[:, :2]
        # centerline_coords = np.array([
        #     [coord.x_m, coord.y_m] for coord in centerline_waypoints.wpnts
        # ])

        # psi_centerline, kappa_centerline = tph.calc_head_curv_num.\
        #     calc_head_curv_num(
        #         path=centerline_coords,
        #         el_lengths=0.1*np.ones(len(centerline_coords)-1),
        #         is_closed=False
        #     )
def is_pivots_nan(pivot1,pivot2):
    if (1-np.isnan(pivot1))*(1-np.isnan(pivot2)):
        return False
    else:
        return True

def fined_wall(wall, min_dis):
    new_list=[]
    ref=0
    for i in range(len(wall)):
        j = (i+1)%len(wall)
        dis_x= wall[j,0]-wall[i,0]
        dis_y= wall[j,0]-wall[i,0]
        dis=np.sqrt(dis_x**2+dis_y**2)
        if dis >min_dis:
            for k in range (int(dis//min_dis)):
                new_point=[wall[i,0]+dis_x*(k+1)/(dis//min_dis+1),wall[i,1]+dis_y*(k+1)/(dis//min_dis+1)]
                new_index = i+k+ref+1
                new_list.append([new_point,new_index])
                # new_list.append([[wall[i,0]+dis_x*(k+1)/(dis//min_dis+1),wall[i,1]+dis_y*(k+1)/(dis//min_dis+1)],i+k+ref+1])
            ref+=int(dis//min_dis)

    refined_wall=insert_new_points(wall,new_list)

    return refined_wall

def insert_new_points(wall , new_list):
    # print(len(wall))
    for i in range(len(new_list)):
        wall = np.insert(wall, new_list[i][1] ,new_list[i][0],axis=0)
    return wall

def calc_curv(traj):
    traj=np.array(traj)
    # print(len(traj[:,0]))
    dx = np.gradient(traj[:,0])
    dy = np.gradient(traj[:,1])    
    # print(len(dx))
    d2x = np.gradient(dx)
    d2y = np.gradient(dy)
    # print(len(d2x))
    curvature = (dx * d2y - d2x * dy) / (dx * dx + dy * dy)**1.5
    # print(curvature[0])
    return curvature

def calculate_ey(pub_track, position, yaw, is_inner, is_counterclock,threshold):
    x=position.x
    y=position.y
    sign=(2*is_inner-1)*(2*is_counterclock-1)
    rot_yaw=yaw+sign*np.pi/2
    ey=1000.0
    if is_inner:
        wall=pub_track.innerwall
        for i in range(len(wall)):
            dot=np.cos(rot_yaw)*(wall[i,0]-x)+np.sin(rot_yaw)*(wall[i,1]-y)
            ver_dis = np.abs(np.cos(rot_yaw)*(wall[i,1]-y)+np.sin(rot_yaw)*(wall[i,0]-x))
            # angle=np.arccos(dot/np.sqrt((wall[i,0]-x)**2+(wall[i,1]-y)**2))
            if (ver_dis<threshold and ey>np.abs(dot)):
                ey=np.abs(dot)
    else:
        wall=pub_track.outerwall
        for i in range(len(wall)):
            dot=np.cos(rot_yaw)*(wall[i,0]-x)+np.sin(rot_yaw)*(wall[i,1]-y)
            ver_dis = np.abs(np.cos(yaw)*(wall[i,0]-x)+np.sin(yaw)*(wall[i,1]-y))
            # angle=np.arccos(dot/np.sqrt((wall[i,0]-x)**2+(wall[i,1]-y)**2))
            if (dot>0 and ver_dis<threshold and ey>np.abs(dot)):
                ey=np.abs(dot)
    # min_angle=np.pi
    # for i in range(len(wall)):
    #     dot=np.cos(rot_yaw)*(wall[i,0]-x)+np.sin(rot_yaw)*(wall[i,1]-y)
    #     angle=np.arccos(dot/np.sqrt((wall[i,0]-x)**2+(wall[i,1]-y)**2))
    #     if angle<min_angle:
    #         min_angle=angle
    #         ey=np.abs(dot)
    return ey




def path_contain_zero_wp(m1_id, m2_id ,max_id):
    diff = abs(m1_id - m2_id)
    if diff > 0.5*max_id:
        return True
    else:
        return False
    
def straighten_2d(anchor1, anchor2, data_2d):
    if anchor1 is None or anchor2 is None:
        return data_2d 
    
    start_idx = None
    num_idx = None
    start_data = None
    diff_data = None
    
    if anchor1[0] == anchor2[0]:
        return data_2d
    elif anchor1[0] > anchor2[0]:
        if (anchor1[0] - anchor2[0]) > len(data_2d)/2:
            start_idx = anchor1[0]
            num_idx = len(data_2d) - (anchor1[0] - anchor2[0])
            start_data = anchor1[1:3]
            diff_data = [anchor2[1] - anchor1[1], anchor2[2] - anchor1[2]]
        else:
            start_idx = anchor2[0]
            num_idx = anchor1[0] - anchor2[0]
            start_data = anchor2[1:3]
            diff_data = [anchor1[1] - anchor2[1], anchor1[2] - anchor2[2]]
    else:
        if (anchor2[0] - anchor1[0]) > len(data_2d)/2:
            start_idx = anchor2[0]
            num_idx = len(data_2d) - (anchor2[0] - anchor1[0])
            start_data = anchor2[1:3]
            diff_data = [anchor1[1] - anchor2[1], anchor1[2] - anchor2[2]]
        else:
            start_idx = anchor1[0]
            num_idx = anchor2[0] - anchor1[0]
            start_data = anchor1[1:3]
            diff_data = [anchor2[1] - anchor1[1], anchor2[2] - anchor1[2]]
        
    for i in range(num_idx+1):
        idx = (start_idx+i)%len(data_2d)
        data_2d[idx][0] = start_data[0] + i * diff_data[0] / num_idx
        data_2d[idx][1] = start_data[1] + i * diff_data[1] / num_idx

    return data_2d

def straighten_1d(anchor1, anchor2, data_1d):
    if anchor1 is None or anchor2 is None:
        return data_1d 
    
    start_idx = None
    num_idx = None
    start_data = None
    diff_data = None
    
    if anchor1[0] == anchor2[0]:
        return data_1d
    elif anchor1[0] > anchor2[0]:
        if (anchor1[0] - anchor2[0]) > len(data_1d)/2:
            start_idx = anchor1[0]
            num_idx = len(data_1d) - (anchor1[0] - anchor2[0])
            start_data = anchor1[3]
            diff_data = anchor2[3] - anchor1[3]
        else:
            start_idx = anchor2[0]
            num_idx = anchor1[0] - anchor2[0]
            start_data = anchor2[3]
            diff_data = anchor1[3] - anchor2[3]
    else:
        if (anchor2[0] - anchor1[0]) > len(data_1d)/2:
            start_idx = anchor2[0]
            num_idx = len(data_1d) - (anchor2[0] - anchor1[0])
            start_data = anchor2[3]
            diff_data = anchor1[3] - anchor2[3]
        else:
            start_idx = anchor1[0]
            num_idx = anchor2[0] - anchor1[0]
            start_data = anchor1[3]
            diff_data = anchor2[3] - anchor1[3]             
        
    for i in range(num_idx+1):
        idx = (start_idx+i)%len(data_1d)
        data_1d[idx] = start_data + i * diff_data / num_idx
    
    return data_1d

def Vel_Offset(pub_track, marker_name, user_input):
    ori_pose = pub_track.server.get(marker_name)
    ori_pose.pose.position.z = ori_pose.pose.position.z + user_input     
    pub_track.menu_handler.reApply( pub_track.server )
    pub_track.server.applyChanges()
    return

def Vel_Set(anchor1, anchor2, vel, data_1d): 
    if anchor1 is None or anchor2 is None:
        return data_1d 
    
    start_idx = None
    num_idx = None

    if anchor1[0] == anchor2[0]:
        return data_1d
    elif anchor1[0] > anchor2[0]:
        if (anchor1[0] - anchor2[0]) > len(data_1d)/2:
            start_idx = anchor1[0]
            num_idx = len(data_1d) - (anchor1[0] - anchor2[0])
        else:
            start_idx = anchor2[0]
            num_idx = anchor1[0] - anchor2[0]
    else:
        if (anchor2[0] - anchor1[0]) > len(data_1d)/2:
            start_idx = anchor2[0]
            num_idx = len(data_1d) - (anchor2[0] - anchor1[0])
        else:
            start_idx = anchor1[0]
            num_idx = anchor2[0] - anchor1[0]         
        
    for i in range(num_idx+1):
        idx = (start_idx+i)%len(data_1d)
        data_1d[idx] = vel
    
    return data_1d

def Vel_Weight(pub_track, marker1_name, marker2_name, user_input):    
    m1_id = int(marker1_name[2:])
    m2_id = int(marker2_name[2:])
    if path_contain_zero_wp(m1_id, m2_id,pub_track.max_id):
        if m1_id < m2_id:
            tmp1 = range(m1_id, -1,-1)
            tmp2 = range(pub_track.max_id,m2_id-1,-1)
            smooth_range = list(tmp1)+list(tmp2)
        else:
            tmp1 = range(m2_id,-1,-1)
            tmp2 = range(pub_track.max_id,m1_id-1,-1)
            smooth_range = list(tmp1)+list(tmp2)
    else:
        if m1_id < m2_id:
            smooth_range = range(m1_id, m2_id+1,1)
        else:
            smooth_range = range(m1_id, m2_id-1,-1)

    for i in smooth_range:
        marker_name = "wp"+str(i)
        int_marker = pub_track.server.get(marker_name)
        int_marker.pose.position.z = int_marker.pose.position.z*np.float64(user_input)

    pub_track.menu_handler.reApply( pub_track.server )
    pub_track.server.applyChanges()
    return

def cal_unit_vec(yaw):
    return np.array([np.cos(yaw), np.sin(yaw)])

def cal_yaw(unit_vector):
    yaw = np.arctan2(unit_vector[1], unit_vector[0])
    return yaw

def sampleCubicSplinesWithDerivative(reference, data, resolution, target, scale):
    '''
    Compute and sample the cubic splines for a set of input points with
    optional information about the tangent (direction AND magnitude). The 
    splines are parametrized along the traverse line (piecewise linear), with
    the resolution being the step size of the parametrization parameter.
    The resulting samples have NOT an equidistant spacing.

    Arguments:      points: a list of n-dimensional points
                    tangents: a list of tangents
                    resolution: parametrization step size
    Returns:        samples

    Notes: Lists points and tangents must have equal length. In case a tangent
        is not specified for a point, just pass None. For example:
                    points = [[0,0], [1,1], [2,0]]
                    tangents = [[1,1], None, [1,-1]]

    '''    # print(self.server.get(marker_name))
    ref=reference[1:]
    ref_back=data[(reference[0]-resolution)%len(data)]
    ref_forw=data[(reference[0]+resolution)%len(data)]
    # tan=self.cal_unit_vec(self.server.get(marker_name).controls[0].markers[0].color.r)
    # tan1=self.cal_unit_vec(self.server.get('wp'+str((reference[0]-resolution)%self.track_len)).controls[0].markers[0].color.r)
    # tan2=self.cal_unit_vec(self.server.get('wp'+str((reference[0]+resolution)%self.track_len)).controls[0].markers[0].color.r)
    
    

    points=[]
    tangents=[]
    if target == "Pose":
        points.append(ref_back[0:2]) ; tangents.append(cal_unit_vec(ref_back[2]))
        points.append(ref[0:2])           ; tangents.append(cal_unit_vec(data[reference[0]][2]))
        points.append(ref_forw[0:2]) ; tangents.append(cal_unit_vec(ref_forw[2]))

    elif target == "Vel":
        vec1=np.array([1, (data[(reference[0]-resolution + 1)%len(data)] -data[(reference[0]-resolution -1)%len(data)])/2])
        vec=np.array([1, (data[(reference[0] + 1)%len(data)] -data[(reference[0] -1)%len(data)])/2])
        vec2=np.array([1, (data[(reference[0]+resolution + 1)%len(data)] -data[(reference[0]+resolution -1)%len(data)])/2])
        
        points.append([0,ref_back])            ; tangents.append(vec1/np.linalg.norm(vec1))
        points.append([resolution,ref[2]])     ; tangents.append(vec/np.linalg.norm(vec))
        points.append([resolution*2,ref_forw]) ; tangents.append(vec2/np.linalg.norm(vec2))

    tangents = np.dot(tangents, scale*np.eye(2))
    points = np.asarray(points)
    nPoints, dim = points.shape

    # Parametrization parameter s.
    dp = np.diff(points, axis=0)                 # difference between points
    dp = np.linalg.norm(dp, axis=1)              # distance between points
    d = np.cumsum(dp)                            # cumsum along the segments
    d = np.hstack([[0],d])                       # add distance from first point
    l = d[-1]                                    # length of point sequence
    nSamples = resolution*2+1                 # number of samples 
    s,r = np.linspace(0,l,nSamples,retstep=True) # sample parameter and step

    # Bring points and (optional) tangent information into correct format.
    assert(len(points) == len(tangents))
    spline_result = np.empty([nPoints, dim], dtype=object)
    for i,ref in enumerate(points):
        t = tangents[i]
        # Either tangent is None or has the same
        # number of dimensions as the point ref.
        assert(t is None or len(t)==dim)
        fuse = list(zip(ref,t) if t is not None else zip(ref,))
        spline_result[i,:] = fuse

    # Compute splines per dimension separately.
    samples = np.zeros([nSamples, dim])
    for i in range(dim):
        poly = interpolate.BPoly.from_derivatives(d, spline_result[:,i])
        samples[:,i] = poly(s)

    for i in range(resolution*2+1):
        if target=="Pose":
            data[(reference[0]-resolution+i)%len(data)][0] = samples[i][0]
            data[(reference[0]-resolution+i)%len(data)][1] = samples[i][1]
        elif target=="Vel":
            data[(reference[0]-resolution+i)%len(data)] = samples[i][1]  
    
    return data

def entire_traj_translation(reference, data_2d):
    diff = np.array([reference[1] - data_2d[reference[0]][0], reference[2] - data_2d[reference[0]][1]] )
    data_2d += diff
    
    # for i in range(len(data_2d)):
    #     data_2d
    #     idx = (start_idx+i)%len(data_1d)
    #     data_1d[idx] = vel
    
    return data_2d
    
def rotate_point(point, angle, center):
    # Calculate the angle in radians
    
    # Translate the point to the origin
    translated_x = point[0] - center[0]
    translated_y = point[1] - center[1]
    
    # Perform the rotation
    rotated_x = translated_x * np.cos(angle) - translated_y * np.sin(angle)
    rotated_y = translated_x * np.sin(angle) + translated_y * np.cos(angle)
    
    # Translate the point back to its original position
    result_x = rotated_x + center[0]
    result_y = rotated_y + center[1]
    
    return result_x, result_y
        
def entire_traj_rotation(anchor1, anchor2, data_2d):
    dis_sqr_1=(data_2d[anchor1[0]][0] - anchor1[1])**2+(data_2d[anchor1[0]][1] - anchor1[2])**2
    dis_sqr_2=(data_2d[anchor2[0]][0] - anchor2[1])**2+(data_2d[anchor2[0]][1] - anchor2[2])**2
    if dis_sqr_1<dis_sqr_2:
        rot_center=[anchor1[1], anchor1[2]]
        ref1_point=[data_2d[anchor2[0]][0], data_2d[anchor2[0]][1]]
        ref2_point=[anchor2[1], anchor2[2]]
    else:
        rot_center=[anchor2[1], anchor2[2]]
        ref1_point=[data_2d[anchor1[0]][0], data_2d[anchor1[0]][1]]
        ref2_point=[anchor1[1], anchor1[2]]
        
    ref1_vec=[ref1_point[0] - rot_center[0], ref1_point[1] - rot_center[1]]
    ref2_vec=[ref2_point[0] - rot_center[0], ref2_point[1] - rot_center[1]]
    dot=ref1_vec[0]*ref2_vec[0]+ref1_vec[1]*ref2_vec[1]
    cro=ref1_vec[0]*ref2_vec[1]-ref1_vec[1]*ref2_vec[0]
    mag_1=np.sqrt(ref1_vec[0]**2+ref1_vec[1]**2)
    mag_2=np.sqrt(ref2_vec[0]**2+ref2_vec[1]**2)

    # Calculate the rotation angle
    if cro>0:
        angle = np.arccos(dot / (mag_1*mag_2))
    else:
        angle = -np.arccos(dot / (mag_1*mag_2))
    # Use one of the points as the center of rotation (e.g., m1_int_pos)
    
    for i in range(len(data_2d)):
        # marker_name = "wp" + str(i)
        # if i == m1_id or i==m2_id:
        #     ori_pose=pub_track.track.markers[i].pose
        # else:
        #     ori_pose = pub_track.server.get(marker_name).pose
        data_2d[i][0], data_2d[i][1] = rotate_point(data_2d[i], angle, rot_center)
    return data_2d


def set_lookahead(pub_track, marker1_name, marker2_name, input_value):
    print(marker1_name[2:])
    m1_id = int(marker1_name[2:])
    m2_id = int(marker2_name[2:])
    if path_contain_zero_wp(m1_id, m2_id,pub_track.max_id):
        if m1_id < m2_id:
            tmp1 = range(m1_id, -1,-1)
            tmp2 = range(pub_track.max_id,m2_id-1,-1)
            smooth_range = list(tmp1)+list(tmp2)
        else:
            tmp1 = range(m2_id,-1,-1)
            tmp2 = range(pub_track.max_id,m1_id-1,-1)
            smooth_range = list(tmp1)+list(tmp2)
    else:
        if m1_id < m2_id:
            smooth_range = range(m1_id, m2_id+1,1)
        else:
            smooth_range = range(m1_id, m2_id-1,-1)

    for i in smooth_range:
        marker_name = "wp"+str(i)
        int_marker = pub_track.server.get(marker_name)
        int_marker.controls[0].markers[0].color.r = np.float32(input_value)
        
    pub_track.menu_handler.reApply( pub_track.server )
    pub_track.server.applyChanges()
    return

def OnOff(pub_track,marker1_name, marker2_name, input_value,Target):
    # print(marker1_name[2:]) 
    m1_id = int(marker1_name[2:])
    m2_id = int(marker2_name[2:])
    if path_contain_zero_wp(m1_id, m2_id,pub_track.max_id):
        if m1_id < m2_id:
            tmp1 = range(m1_id, -1,-1)
            tmp2 = range(pub_track.max_id,m2_id-1,-1)
            smooth_range = list(tmp1)+list(tmp2)
        else:
            tmp1 = range(m2_id,-1,-1)
            tmp2 = range(pub_track.max_id,m1_id-1,-1)
            smooth_range = list(tmp1)+list(tmp2)
    else:
        if m1_id < m2_id:
            smooth_range = range(m1_id, m2_id+1,1)
        else:
            smooth_range = range(m1_id, m2_id-1,-1)
    
    for i in smooth_range:
        marker_name = "wp"+str(i)
        int_marker = pub_track.server.get(marker_name)
        if Target=="Lookahead":
            int_marker.controls[0].markers[0].color.g = input_value
            int_marker.controls[0].markers[0].color.r = pub_track.track.markers[i].color.r
            int_marker.controls[0].markers[0].color.b = pub_track.track.markers[i].color.b
        if Target=="Velocity":
            int_marker.controls[0].markers[0].color.b = input_value
            int_marker.controls[0].markers[0].color.r = pub_track.track.markers[i].color.r
            int_marker.controls[0].markers[0].color.g = pub_track.track.markers[i].color.g
    pub_track.menu_handler.reApply( pub_track.server )
    pub_track.server.applyChanges()
    return



def distance_between_int_markers(int_marker1, int_marker2):
    if int_marker2 == None:
        return 999
    m1_p = np.array([int_marker1.pose.position.x, int_marker1.pose.position.y, int_marker1.pose.position.z])
    m2_p = np.array([int_marker2.pose.position.x, int_marker2.pose.position.y, int_marker2.pose.position.z])
    distance = np.linalg.norm(m1_p-m2_p)
    # print(distance)
    return distance

def cal_slope(prev_marker, cur_marker, next_marker):
    p_x=prev_marker.pose.position.x
    p_y=prev_marker.pose.position.y
    c_x=cur_marker.pose.position.x
    c_y=cur_marker.pose.position.y
    n_x=next_marker.pose.position.x
    n_y=next_marker.pose.position.y
    vec_1=np.array([c_x-p_x,c_y-p_y])
    vec_2=np.array([n_x-c_x,n_y-c_y])
    # print(vec_1)
    norm_vec_1=vec_1/np.linalg.norm(vec_1)
    norm_vec_2=vec_2/np.linalg.norm(vec_2)
    # print(norm_vec_1)
    return (vec_1+vec_2)/np.linalg.norm(norm_vec_1+norm_vec_2)