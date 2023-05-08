import numpy as np

MOUNT_ANGLE = 30
wall_slope = np.tan(30/180*np.pi)
wall_distance = 3

def remove_recede_obj(obj_list_in) : 
    obj_list_out = []
    for ii in range(len(obj_list_in)) :
        x = obj_list_in[ii].x
        dx = obj_list_in[ii].xd
        if (x > 20) :
            if (dx < 0) :
                obj_list_out.append(obj_list_in[ii])
        else :
            obj_list_out.append(obj_list_in[ii])
        

    return obj_list_out

def remove_far_obj(obj_list_in) :
    obj_list_out = []

    for ii in range(len(obj_list_in)) :
        if (obj_list_in[ii].x <= 9) & (obj_list_in[ii].y >= -30): # 34 * cosd(30) : distance : 30, margin : 4m   (backward)
            obj_list_out.append(obj_list_in[ii])
            continue
    return obj_list_out


# def remove_wall_obj(obj_list_in) :
#     global wall_distance, wall_slope
#     obj_list_out = []
#     margin = 0.5
#     mean_x = 0
#     mean_y = 0
#     num_obj = len(obj_list_in)
#     if num_obj == 0 :
#         obj_list_out = obj_list_in
#         return obj_list_out
#     for ii in range(num_obj) :
#         x= obj_list_in[ii].x
#         y= obj_list_in[ii].y
#         mean_x += x
#         mean_y += y
#     mean_x = mean_x / num_obj
#     mean_y = mean_y / num_obj
#     bunja = 0
#     bunmo = 0
#     for ii in range(num_obj) :
#         x= obj_list_in[ii].x
#         y= obj_list_in[ii].y
#         if ((x-mean_x) <= margin) & ((y-mean_y) <= margin) :
#             # bunja += (x-mean_x)*(y-mean_y)
#             # bunmo += (x-mean_x)**2
#             bunja += (x*y)-(mean_y)
#             bunmo += (x-mean_x)**2
#     if bunmo == 0 :
#         obj_list_out = obj_list_in
#         return obj_list_out
#     a = bunja/bunmo
#     b = mean_y - a * mean_x
#     # if abs(a - wall_slope) < 0.1:
#     for ii in range(num_obj) :
#         x= obj_list_in[ii].x
#         y= obj_list_in[ii].y
#         dist = a*x + b
#         if abs(y - dist) > margin :
#             obj_list_out.append(obj_list_in[ii])
#     else : 
#         obj_list_out = obj_list_in
#     # wall_slope = wall_slope*0.95 + a*0.05
#     # wall_distance = wall_distance*0.95 + b*0.05
#     print (a, b, wall_slope, np.tan(30/180*np.pi))
#     return obj_list_out

def remove_wall_obj(obj_list_in) :
    global wall_distance, wall_slope
    obj_list_out = obj_list_in

    return obj_list_out



# def remove_far_obj(obj_list_in) :
#     obj_list_out = []
#     for ii in range(len(obj_list_in)) :

#     return obj_list_out