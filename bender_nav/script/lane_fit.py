import numpy as np
import math
from scipy.optimize import curve_fit 
     
#These functions identify the two lanes from the cost array and the value function and fit a polynomial on each lane

def sliding_window_col(vf, i, j, win_size):
    half_step = math.floor(win_size/2)
    return vf[i-half_step : i+half_step, j : j+win_size]
    
def sliding_window_row(vf, i, j, win_size):
    half_step = math.floor(win_size/2)
    return vf[i : i+win_size, j-half_step : j+half_step]

def identify_lanes(vf, cost_arr):
    #given agent heading, this functions finds the two lanes. To do so, we first create histogram of all the columns and row
    #to find the base of each lane. Then we use a 5x5 sliding window to move from the base of the lane to the top edge.
    #We save and return each lane separately

    #NOTE: vf is populated as a grid where the x position is the column and the y position is the row.
    #so to build histogram of the lanes, we average the columns (i.e iterate over x positions) and average the row (i.e iterate over y positions) 
    ##############################
    lane_thickness = 40
    ####################################
    
    collapsed_row = cost_arr[0,:]
    for i in range(0, math.floor(np.size(cost_arr, 0))-1, 1): #check half of the costmap grid (around curves histogram will be shifted at base)
        collapsed_row = collapsed_row + 1.0/1.0*cost_arr[i+1,:]

    collapsed_col = cost_arr[:,0]
    for j in range(0, math.floor(np.size(cost_arr, 1))-1): #check half of the costmap grid (around curves histogram will be shifted at base)
        collapsed_col = collapsed_col + 1.0/1.0*cost_arr[:,j+1]

    #sort and find the corresponding indices
    sorted_col_ind = np.argsort(collapsed_col)
    sorted_row_ind = np.argsort(collapsed_row)

    #detect index jump
    max_col_ind = [sorted_col_ind[-1],0]
    for i in range(len(sorted_col_ind)-1, 0, -1):
        if np.abs(sorted_col_ind[-1] - sorted_col_ind[i-1]) > lane_thickness:
            max_col_ind[1] =  sorted_col_ind[i-1]
            break
        
    print("sorted_row_ind = ", sorted_row_ind[-5:])
    max_row_ind = [sorted_row_ind[-1],0]
    for i in range(len(sorted_row_ind)-1, 0, -1):
        if np.abs(sorted_row_ind[-1] - sorted_row_ind[i-1]) > lane_thickness:
            max_row_ind[1] =  sorted_row_ind[i-1]
            break

    base_lanes_col_ind = max_col_ind
    base_lanes_row_ind = max_row_ind
    print("base lane options = ", max_col_ind)

    #find the maximum vf on the collapsed col and row
    base_lanes_col = collapsed_col[sorted_col_ind[-1]]
    base_lanes_row = collapsed_row[sorted_row_ind[-1]]

    print("col base cost = ", base_lanes_col)
    print("row base cost = ", base_lanes_row)

    print("row base = ", base_lanes_row_ind)
    print("col base = ", base_lanes_col_ind)

    win_size = 4
    half_step = math.floor(win_size/2)
    #pick row or column with highest vf; may pick one of each if the agent is on a curve
    # base        = base_lanes_col_ind
    # search_ind = range(win_size, np.size(vf, 0)-win_size, win_size)
    # search_dirc = "col"
        
    if np.sum(base_lanes_col) >= np.sum(base_lanes_row):
        base        = base_lanes_col_ind
        search_ind = range(win_size, np.size(vf, 1)-win_size, win_size)
        search_dirc = "col"
        
    elif np.sum(base_lanes_col) < np.sum(base_lanes_row):
        base        = base_lanes_row_ind
        search_ind = range(win_size, np.size(vf, 0)-win_size, win_size)
        search_dirc = "row"

    #create the sliding window
    #lane1 and lane2 are simply populated as [row, col]
    lane1 = []
    lane2 = []
    overwrite_rate = 1

    #slide along the window, pick index of maximum vf, keep sliding
    if search_dirc == "col": 

        lane1.append([base[0], 0])
        lane2.append([base[1], 0])
        overwrite_col_rate1 = 0
        overwrite_col_rate2 = 0

        while ((lane1[-1][1] < 300) & (lane2[-1][1] < 300)):
            # print(i)
            # print(lane2[i-1])
            sliding_win1 = sliding_window_col(vf, lane1[-1][0], lane1[-1][1], win_size)
            sliding_win2 = sliding_window_col(vf, lane2[-1][0], lane2[-1][1], win_size)
            # print("sliding window = ", list(np.unravel_index(np.argmax(sliding_win1), sliding_win1.shape)))
            newlane1 = np.add(lane1[-1], list(np.unravel_index(np.argmax(sliding_win1), sliding_win1.shape)))
            newlane2 = np.add(lane2[-1] , list(np.unravel_index(np.argmax(sliding_win2), sliding_win2.shape)))

            if (lane1[-1] == newlane1).all() :

                if (np.max(sliding_win1[:,-1]) == sliding_win1[half_step,-1]):
        #if the maximum vf in the sliding window is the previous maximum location, 
        #then select the largest vf on the last column. If all the values on the last column
        #are equal, pick the center index.
                    next_col = [0, np.size(sliding_win1, 1)] 
                else: 
                    next_col = [np.argmax(sliding_win1[:,-1])-half_step, np.size(sliding_win1, 1)]
        #overwrite the lane if it keeps picking the same index. It means it has not reached the lane pixels yet
                # if (overwrite_col_rate1 < overwrite_rate):
                #     lane1[-1] = np.add(lane1[-1] , next_col)
                #     overwrite_col_rate1 += 1
                # else:
                lane1.append(np.add(lane1[-1] , next_col))
                #     overwrite_col_rate1 = 0
                    
            else :
        #has reached lane indices
                lane1.append(newlane1)
                # lane1[-1] = newlane1

        ##repeat for lane 2
            if (lane2[-1] == newlane2).all() :

                if (np.max(sliding_win2[:,-1]) == sliding_win2[half_step,-1]):
            
                    next_col = [0, np.size(sliding_win2, 1)] 
                else: 
                    next_col = [np.argmax(sliding_win2[:,-1])-half_step, np.size(sliding_win2, 1)]

                # if (overwrite_col_rate2 < overwrite_rate):
                #     lane2[-1] = np.add(lane2[-1] , next_col )
                #     overwrite_col_rate2 += 1
                # else:
                lane2.append(np.add(lane2[-1] , next_col ))
                #     overwrite_col_rate2 = 0

            else:
                lane2.append(newlane2)
                # lane2[-1] = newlane2

    elif search_dirc == "row":

        lane1.append([0, base[0]])
        lane2.append([0, base[1]])

        overwrite_row_rate1 = 0
        overwrite_row_rate2 = 0

        while ((lane1[-1][0] < 300) & (lane2[-1][0] < 300)):
            #check moving base plus 5x5 sliding window; pick one index for every 5x5 window to curve fit
            sliding_win1 = sliding_window_row(vf, lane1[-1][0], lane1[-1][1], win_size)
            sliding_win2 = sliding_window_row(vf, lane2[-1][0], lane2[-1][1], win_size)

            newlane1 = np.add(lane1[-1] , list(np.unravel_index(np.argmax(sliding_win1), sliding_win1.shape)))
            newlane2 = np.add(lane2[-1] , list(np.unravel_index(np.argmax(sliding_win2), sliding_win2.shape)))

            if (lane1[-1] == newlane1).all() :
                if (np.max(sliding_win1[-1,:]) == sliding_win1[-1, half_step]):
        #if maximum of the sliding window does not change, pick maximum from the last row.
        #if all the vf's on the last row have the same value, select the center index
                    next_row = [np.size(sliding_win1, 0), 0] 
                else: 
                    next_row = [np.size(sliding_win1, 0), np.argmax(sliding_win1[-1,:])-half_step]

                # if (overwrite_row_rate1 < overwrite_rate):
                #     lane1[-1] = np.add(lane1[-1] , next_row)
                #     overwrite_row_rate1 += 1
                # else:
                lane1.append(np.add(lane1[-1] , next_row))
                #     overwrite_row_rate1 = 0

            else :
                lane1.append(newlane1)
                # lane1[-1] = newlane1

                
            if (lane2[-1] == newlane2).all() :

                if (np.max(sliding_win2[-1,:]) == sliding_win2[-1, half_step]):
                    next_row = [np.size(sliding_win2, 0), 0] 
                else: 
                    next_row = [np.size(sliding_win2, 0), np.argmax(sliding_win2[-1,:])-half_step]
                
                # if (overwrite_row_rate2 < overwrite_rate):
                #     lane2[-1] = np.add(lane2[-1] , next_row)
                #     overwrite_row_rate2 += 1
                # else:
                lane2.append(np.add(lane2[-1] , next_row))
                #     overwrite_row_rate2 = 0

            else:
                lane2.append(newlane2)
                # lane2[-1] = newlane2

    # print("lane1 = ", lane1)
    return lane1, lane2

def lane_poly(x, a0, a1, a2, a3):
    return a0 + a1*x + a2*x**2 + a3*x**3  

def lane_fit(x, a0, a1, a2, a3):

    lane_approximation_thickness = 0.01
    y1  =  lane_poly(x, a0, a1, a2, a3)
    lane_ind1 = [[math.floor(z), x1] for (z, x1) in zip(y1, x)]  
    # y2  =  lane_poly(x, a0*(1.0+lane_approximation_thickness), a1, a2, a3)
    # lane_ind2 = [[math.floor(z), x1] for (z, x1) in zip(y2, x)] 
    # y3  =  lane_poly(x, a0*(1.0-lane_approximation_thickness), a1, a2, a3)
    # lane_ind3 = [[math.floor(z), x1] for (z, x1) in zip(y3, x)]     

    # print("lane curve fit ind = ", lane_ind)
    # return np.concatenate((lane_ind1, lane_ind2, lane_ind3))
    return lane_ind1

def approximate_poly_lane(vf, cost_arr):
    #approximate polynomial fit for each lane
    lane1, lane2 = identify_lanes(vf, cost_arr)
    x1 = [l[1] for l in lane1]
    y1 = [l[0] for l in lane1]
    x2 = [l[1] for l in lane2]
    y2 = [l[0] for l in lane2]

    #TODO: draw circle in rviz
    #TODO: if the lanes are not visible (not enough data points for optimizer), handle exception

    popt1, cov1 = curve_fit(lane_poly, x1, y1)
    a01, a11, a21, a31 = popt1 
    popt2, cov2 = curve_fit(lane_poly, x2, y2)
    a02, a12, a22, a32 = popt2 
    return  a01, a11, a21, a31, a02, a12, a22, a32

