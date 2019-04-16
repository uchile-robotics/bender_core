from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import csv

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

#Define all the lists we're gonna use:
ws = [] #Cantains all the data
xp = [] #X coordinates for Possible solutions
yp = [] #Y coordinates for Possible solutions
zp = [] #Z coordinates for Possible solutions
xi = [] #X coordinates for Impossible solutions
yi = [] #Y coordinates for Impossible solutions
zi = [] #Z coordinates for Impossible solutions
mini = -0.55
maxi = 1.4
x = [-0.4, -0.4, -0.4, -0.4, 0.8, 0.8, 0.8, 0.8] 
y = [-0.55, -0.55, 0.65, 0.65, -0.55, -0.55, 0.65, 0.65]
z = [0.2, 1.4, 0.2, 1.4, 0.2, 1.4, 0.2, 1.4]

#Use thw 'ws' list to contain all the data from 'workspace.csv'
with open('/home/robotica/uchile_ws/pkgs/base_ws/bender_core/bender_test_config_v38/scripts/workspace.csv', 'r') as f:
	reader = csv.reader(f)
	ws = list(reader)

#Assing the coordinates for possible and impossible solutions
for i in range(0,(len(ws)-1)):
	if float(ws[i][0]) > 0: #Possible
		xp.append(float(ws[i][1]))
		yp.append(float(ws[i][2]))
		zp.append(float(ws[i][3]))
	elif float(ws[i][0]) < 0: #Impossible
		xi.append(float(ws[i][1]))
		yi.append(float(ws[i][2]))
		zi.append(float(ws[i][3]))


#Plot all point with their respective color (Green for Possible and Red for Impossible)
ax.scatter(xp, yp, zp, c='g', marker='o')
ax.scatter(x, y ,z, c=[1.0, 0.0, 0.0, 0.0], marker='.')

ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')

plt.show()