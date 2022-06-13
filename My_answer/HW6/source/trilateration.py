from dis import dis
from locale import DAY_2
from scipy.optimize import minimize
import numpy as np

def trilateration(distances_to_APs, STA_coordinates, target_position=None):
    # you should complete the function for question1
    # related to the distances between STA and three APs
    x1 = STA_coordinates[0][0]
    y1 = STA_coordinates[0][1]
    x2 = STA_coordinates[1][0]
    y2 = STA_coordinates[1][1]
    x3 = STA_coordinates[2][0]
    y3 = STA_coordinates[2][1]
    d1 = distances_to_APs[0]
    d2 = distances_to_APs[1]
    d3 = distances_to_APs[2]
    A = np.matrix([[2*(x1-x3),2*(y1-y3)],[2*(x2-x3),2*(y2-y3)]])
    B = np.matrix([[x1**2-x3**2+y1**2-y3**2+d3**2-d1**2],[x2**2-x3**2+y2**2-y3**2+d3**2-d2**2]])
    target_position = np.matmul(np.linalg.inv(np.matmul(A.T, A)),np.matmul(A.T, B))

    return target_position

if __name__ == "__main__":
	stations = list(np.array([[1,1], [0,1], [1,0]]))
	distances_to_station = [0.1, 0.5, 0.5]
	print(trilateration(distances_to_station, stations))
