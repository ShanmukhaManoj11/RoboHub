import numpy as np
import mayavi.mlab
import struct

def save_pcd(points, out_file='./points.bin'):
	# points is of shape [nx3]
	with open(out_file, 'wb') as f:
		for point in points:
			f.write(struct.pack('fff', point[0], point[1], point[2]))
	print('points saved - {0}'.format(out_file))

def show_points(points):
	# points is np array of shape [3xn]
	fig = mayavi.mlab.figure(bgcolor=(0, 0, 0), size=(640, 500))
	mayavi.mlab.points3d(points[0,:], points[1,:], points[2,:],
							points[0,:],          # Values used for Color
							mode="point",
							# colormap='spectral', # 'bone', 'copper', 'gnuplot'
							color=(0, 1, 0),   # Used a fixed (r,g,b) instead
							figure=fig
						)
	mayavi.mlab.plot3d(np.linspace(0,1,10), np.linspace(0,0,10), np.linspace(0,0,10), tube_radius=0.025, color=(1,0,0), figure=fig)
	mayavi.mlab.plot3d(np.linspace(0,0,10), np.linspace(0,1,10), np.linspace(0,0,10), tube_radius=0.025, color=(0,1,0), figure=fig)
	mayavi.mlab.plot3d(np.linspace(0,0,10), np.linspace(0,0,10), np.linspace(0,1,10), tube_radius=0.025, color=(0,0,1), figure=fig)
	mayavi.mlab.show()

def generate_plane(_normal=[0,0,1], _offset=[0,0,0], _xrange=10, _yrange=10, _npoints=100, _noise=0.1):
	points = np.random.rand(3, _npoints)
	points[0,:] = points[0,:]*_xrange - _xrange*0.5
	points[1,:] = points[1,:]*_yrange - _yrange*0.5
	points[2,:] = points[2,:]*_noise - _noise*0.5

	if _normal!=[0,0,1]:
		_n = np.array(_normal).reshape([3,1])
		_n = _n/np.sqrt(np.sum(_n**2))
		# rotated x axis is cross product of new z axis i.e _n and original z axis i.e [0, 0, 1]
		_xr = np.array([_n[1,0], -_n[0,0], 0]).reshape([3,1])
		_xr = _xr/np.sqrt(np.sum(_xr**2))
		# rotated y axis is cross product of new z axis i.e _n and rotated x axis
		_yr = np.array([_n[1,0]*_xr[2,0]-_n[2,0]*_xr[1,0], -_n[0,0]*_xr[2,0]+_n[2,0]*_xr[0,0], _n[0,0]*_xr[1,0]-_n[1,0]*_xr[0,0]]).reshape([3,1])
		_yr = _yr/np.sqrt(np.sum(_yr**2))
		# rotation matrix columns are the new axes
		R = np.concatenate([_xr, _yr, _n], axis=1)
		points = np.matmul(R, points) + np.array(_offset).reshape([3,1])

	show_points(points)

	save_pcd(points.T, out_file='/home/mano/work/robotics_ws/data/plane.bin')

def generate_sphere(_r=1.0, _rnoise=0.1, _th_noise=0.1, _npoints=100):
	r = _r + np.random.rand(1, _npoints*_npoints)*_rnoise
	th, alpha = np.meshgrid(np.linspace(-np.pi/2, np.pi/2, _npoints), np.linspace(-np.pi/2, np.pi/2, _npoints))
	th = th.reshape(1, -1)
	alpha = alpha.reshape(1, -1)

	points = np.concatenate([r*np.cos(th)*np.cos(alpha), r*np.cos(th)*np.sin(alpha), r*np.sin(th)], axis=0)
	
	show_points(points)

	save_pcd(points.T, out_file='/home/mano/work/robotics_ws/data/sphere.bin')
		

if __name__=='__main__':
	generate_plane(_normal=[1,1,1], _offset=[-1,-1,-1], _xrange=2, _yrange=2, _npoints=1000, _noise=0.01)
	generate_sphere(_r=5.0, _rnoise=0.01, _th_noise=0.01, _npoints=100)