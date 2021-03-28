import numpy as np
import struct
import mayavi.mlab

def show_input_pointcloud():
	pointcloud = np.fromfile('../data/0000000000.bin', dtype=np.float32, count=-1).reshape([-1,4])
	x = pointcloud[:, 0]  
	y = pointcloud[:, 1]  
	z = pointcloud[:, 2] 
	r = pointcloud[:, 3] 
	 
	fig = mayavi.mlab.figure(bgcolor=(0, 0, 0), size=(640, 500))
	mayavi.mlab.points3d(x, y, z,
							r,          # Values used for Color
							mode="point",
							colormap='spectral', # 'bone', 'copper', 'gnuplot'
							# color=(0, 1, 0),   # Used a fixed (r,g,b) instead
							figure=fig
						)

	mayavi.mlab.plot3d(np.linspace(0,1,10), np.linspace(0,0,10), np.linspace(0,0,10), tube_radius=0.025, color=(1,0,0), figure=fig)
	mayavi.mlab.plot3d(np.linspace(0,0,10), np.linspace(0,1,10), np.linspace(0,0,10), tube_radius=0.025, color=(0,1,0), figure=fig)
	mayavi.mlab.plot3d(np.linspace(0,0,10), np.linspace(0,0,10), np.linspace(0,1,10), tube_radius=0.025, color=(0,0,1), figure=fig)
	
	mayavi.mlab.show()

def check_plane_extraction():
	planecloud = np.fromfile('../data/0000000000_plane_points.bin', dtype=np.float32, count=-1).reshape([-1,3])
	x = planecloud[:, 0]  
	y = planecloud[:, 1]  
	z = planecloud[:, 2]  
	 
	fig = mayavi.mlab.figure(bgcolor=(0, 0, 0), size=(640, 500))
	mayavi.mlab.points3d(x, y, z,
							x,          # Values used for Color
							mode="point",
							# colormap='spectral', # 'bone', 'copper', 'gnuplot'
							color=(0, 1, 0),   # Used a fixed (r,g,b) instead
							figure=fig
						)

	othercloud = np.fromfile('../data/0000000000_other_points.bin', dtype=np.float32, count=-1).reshape([-1,3])
	x = othercloud[:, 0]  
	y = othercloud[:, 1]  
	z = othercloud[:, 2]  
	 
	mayavi.mlab.points3d(x, y, z,
							x,          # Values used for Color
							mode="point",
							# colormap='spectral', # 'bone', 'copper', 'gnuplot'
							color=(1, 0, 0),   # Used a fixed (r,g,b) instead
							figure=fig
						)

	mayavi.mlab.plot3d(np.linspace(0,1,10), np.linspace(0,0,10), np.linspace(0,0,10), tube_radius=0.025, color=(1,0,0), figure=fig)
	mayavi.mlab.plot3d(np.linspace(0,0,10), np.linspace(0,1,10), np.linspace(0,0,10), tube_radius=0.025, color=(0,1,0), figure=fig)
	mayavi.mlab.plot3d(np.linspace(0,0,10), np.linspace(0,0,10), np.linspace(0,1,10), tube_radius=0.025, color=(0,0,1), figure=fig)
	
	mayavi.mlab.show()

if __name__=='__main__':
	show_input_pointcloud()
	check_plane_extraction()