import matplotlib.pyplot as plt
import csv
import sys
import numpy as np

data = {'px_true':[],'py_true':[],'vx_true':[],'vy_true':[],'yaw_true':[],\
		'px_estimate':[],'py_estimate':[],'vx_estimate':[],'vy_estimate':[],\
		'yaw_estimate':[],'Sensor Type':[],'NIS':[]}

L_NIS = []
R_NIS = []

# type in the output .csv file name
with open('build/'+sys.argv[1],'r') as f:
	reader = csv.reader(f, delimiter=',')
	headers = next(reader)

	for row in reader:
		if row[-2] == 'R':
			R_NIS.append(float(row[-1]))
		else:
			L_NIS.append(float(row[-1]))
		for i,header in enumerate(headers):
			try:
				data[header].append(float(row[i]))
			except:
				pass

f.close()


plt.plot(R_NIS,label='NIS_Radar')
plt.plot(np.ones(len(R_NIS))*7.815,label='95% Line')
plt.ylabel('NIS')
plt.xlabel('Frame Index')
plt.grid()
plt.legend()
plt.show()

plt.plot(L_NIS,label='NIS_Laser')
plt.plot(np.ones(len(L_NIS))*5.991,label='95% Line')
plt.ylabel('NIS')
plt.xlabel('Frame Index')
plt.grid()
plt.legend()
plt.show()

plt.plot(data['px_true'],label='px_true',marker='*')
plt.plot(data['px_estimate'],label='px_estimate')
plt.ylabel('x position [m]')
plt.xlabel('Frame Index')
plt.grid()
plt.legend()
plt.show()

plt.plot(data['py_true'],label='py_true',marker='x')
plt.plot(data['py_estimate'],label='py_estimate')
plt.ylabel('y position [m]')
plt.xlabel('Frame Index')
plt.grid()
plt.legend()
plt.show()

plt.plot(data['vx_true'],label='vx_true')
plt.plot(data['vx_estimate'],label='vx_estimate')
plt.ylabel('x velocity [m/s]')
plt.xlabel('Frame Index')
plt.grid()
plt.legend()
plt.show()

plt.plot(data['vy_true'],label='vy_true')
plt.plot(data['vy_estimate'],label='vy_estimate')
plt.ylabel('y velocity [m/s]')
plt.xlabel('Frame Index')
plt.grid()
plt.legend()
plt.show()

plt.plot(data['yaw_true'],label='yaw_true')
plt.plot(data['yaw_estimate'],label='yaw_estimate')
plt.ylabel('yaw angle [rad]')
plt.xlabel('Frame Index')
plt.grid()
plt.legend()
plt.show()