import pyransac3d as pyrsc
import numpy as np
import matplotlib.pyplot as plt
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from simulation.terrain_generator import generate_terrain

THRESHOLD = 0.05 #c.a.d distance au plan qui considere pt comme inlier

#Génerer nuage de points random:
points = generate_terrain(100,50)

def ransac(points, threshold):
    # Points = point cloud as a numpy array (N, 3)

    #RANSAC:
    plane1 = pyrsc.Plane()
    best_eq, best_inliers = plane1.fit(points, threshold) 
    #best_eq = np array (1,4) Avec Ax + By+ Cx + D comme [A,B,C,D]

    #Outliers:
    x_inlier = []
    y_inlier = []
    z_inlier = []
    outliers_index = [i for i in range(points.shape[0]) if i not in best_inliers]
    for i in outliers_index:
        x_inlier.append(points[i,0])
        y_inlier.append(points[i,1])
        z_inlier.append(points[i,2])
    outliers = np.column_stack((x_inlier, y_inlier, z_inlier)) #un np.array avec 3 colonnes qui représente la coordonnée en x, y, z du pt
    return outliers 

#--------------------------------------PLOT----------------------------------------
#Plot nuages de points
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(points[:,0], points[:,1], points[:,2], zorder =1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

#Plot outliers (=obstacles):
outliers = ransac(points, THRESHOLD)
ax.scatter(outliers[:,0], outliers[:,1], outliers[:,2], color = "red", zorder = 2)

# #plot plan
# x = np.linspace(-100,100,100)
# y = np.linspace(-100,100,100)
# x, y = np.meshgrid(x, y)
# z = (- best_eq[0]*x - best_eq[1]*y - best_eq[3])/best_eq[2] #isole z de l'equation
# ax.plot_surface(x, y, z)
plt.show()