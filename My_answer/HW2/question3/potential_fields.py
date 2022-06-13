import numpy as np

class potential_fields:
    
    # Please complete these functions for question2, the arguments such as coefficient can be changed by your need. The return value should be a 2*1 matrix for robot to perform

    def uniform(self, vector=np.array([ [1], [0] ]), coefficient=1):
        return coefficient*vector

    def perpendicular(self, line_obstacle, car_position, coefficient1, coefficient2, state=False):
        if state:
            return np.array([ [0.0], [0.0] ])
        x1=line_obstacle[0]
        y1=line_obstacle[1]
        x2=line_obstacle[2]
        y2=line_obstacle[3]
        x3=car_position[0]
        y3=car_position[1]
        #calculate the direction vector and its length
        n = np.array([ [y2-y1],[x1-x2]])
        l = n.T @ n
        P = np.array([car_position[0],car_position[1]])
        # print(P)
        A = np.array([[x1],[y1]])
        B = np.array([[x2],[y2]])
        #determine the direction
        if (y2-y1)*(x3-x2)==(y3-y2)*(x2-x1):
            t = 0
        elif (P-A).T @n >0:
            t = 1*coefficient1
        else:
            t =-1*coefficient2
        distance = ((P-A).T @ n)/l
        if distance==0:
            return np.array([ [0], [0] ])
        return n * t/(l* distance**2)
    # def perpendicular(self, line_obstacle, car_position, coefficient=1):
    #     v = np.array( [[line_obstacle[0]], [line_obstacle[1]] ])
    #     w = np.array( [[line_obstacle[2]], [line_obstacle[3]] ])
    #     p = np.array( [car_position[0] ,  car_position[1] ])
    #     sdp = potential_fields.shortest_distance_point(self, v, w, p)
    #     distance = sdp[0]
    #     project = sdp[1]
    #     if (p-v).T @(v-w) >0:
    #         t = 1
    #     else:
    #         t =-1
    #     return  0.1*(car_position-project) * t/(distance** 3) 
    
    def attractive(self, goal_point, car_position, coefficient=0.1, state=False):
        if state:
            return np.array([ [0.0], [0.0] ])
        goal = np.array([ goal_point[0]  ,  goal_point[1]])
        car  = np.array([ car_position[0],car_position[1]])
        attract = goal - car
        l = attract.T @ attract
        # print(attract)
        return coefficient*attract*l

    def repulsive(self, obstacle_point, car_position, coefficient=20):
        obstacle = np.array([ obstacle_point[0]  ,  obstacle_point[1]])
        car  = np.array([ car_position[0],car_position[1]])
        repulse = car - obstacle
        l = repulse.T @ repulse
        return coefficient*repulse/(l*l*l)

    def tangential(self, point, car_position, coefficient=0.5):
        target = np.array([ point[0]  ,  point[1]])
        car  = np.array([ car_position[0],car_position[1]])
        # target = [ point[0]  ,  point[1], 0]
        # car  = [ car_position[0],car_position[1],0]
        a = np.double(point[0]-car_position[0])
        b = np.double(point[1]-car_position[1])
        dist = [a,b,0]
        z =target -car
        l = z.T @ z
        w = [0,0,-1]
        result = np.cross(w,dist)
        # print(l)
        return coefficient*np.array([ [result[0]], [result[1]] ])

    def shortest_distance_point(self, v, w, p):
        # the minimum distance between line segment vw, and point p
        # v, w, p all are 2*1 matrix
        l2 = (w - v).T @ (w - v)
        if l2 == 0:
            return np.linalg.norm( p-v )

        t = max(0, min(1, (p - v).T @ (w - v) / l2 ))
        proj_point = v + t * (w-v)
        min_distance = np.linalg.norm( p-proj_point )
        
        return min_distance, proj_point, t