import numpy as np
import struct
import mayavi.mlab

def check_plane():
	pointcloud = np.fromfile('/workspace/data/plane.bin', dtype=np.float32, count=-1).reshape([-1,3])
	x = pointcloud[:, 0]  
	y = pointcloud[:, 1]  
	z = pointcloud[:, 2]  
	 
	fig = mayavi.mlab.figure(bgcolor=(0, 0, 0), size=(640, 500))
	mayavi.mlab.points3d(x, y, z,
							x,          # Values used for Color
							mode="point",
							# colormap='spectral', # 'bone', 'copper', 'gnuplot'
							color=(0, 1, 0),   # Used a fixed (r,g,b) instead
							figure=fig
						)

	normals = np.fromfile('/workspace/data/plane_normals.bin', dtype=np.float32, count=-1).reshape([-1,3])
	mayavi.mlab.quiver3d(x, y, z, normals[:,0], normals[:,1], normals[:,2], line_width=0.2, scale_factor=0.2, figure=fig)

	mayavi.mlab.plot3d(np.linspace(0,1,10), np.linspace(0,0,10), np.linspace(0,0,10), tube_radius=0.025, color=(1,0,0), figure=fig)
	mayavi.mlab.plot3d(np.linspace(0,0,10), np.linspace(0,1,10), np.linspace(0,0,10), tube_radius=0.025, color=(0,1,0), figure=fig)
	mayavi.mlab.plot3d(np.linspace(0,0,10), np.linspace(0,0,10), np.linspace(0,1,10), tube_radius=0.025, color=(0,0,1), figure=fig)
	
	mayavi.mlab.show()

def check_sphere():
	pointcloud = np.fromfile('/workspace/data/sphere.bin', dtype=np.float32, count=-1).reshape([-1,3])
	x = pointcloud[:, 0]  
	y = pointcloud[:, 1]  
	z = pointcloud[:, 2]  
	 
	fig = mayavi.mlab.figure(bgcolor=(0, 0, 0), size=(640, 500))
	mayavi.mlab.points3d(x, y, z,
							x,          # Values used for Color
							mode="point",
							# colormap='spectral', # 'bone', 'copper', 'gnuplot'
							color=(0, 1, 0),   # Used a fixed (r,g,b) instead
							figure=fig
						)

	normals = np.fromfile('/workspace/data/sphere_normals.bin', dtype=np.float32, count=-1).reshape([-1,3])
	mayavi.mlab.quiver3d(x, y, z, normals[:,0], normals[:,1], normals[:,2], line_width=0.2, scale_factor=0.2, figure=fig)

	mayavi.mlab.plot3d(np.linspace(0,1,10), np.linspace(0,0,10), np.linspace(0,0,10), tube_radius=0.025, color=(1,0,0), figure=fig)
	mayavi.mlab.plot3d(np.linspace(0,0,10), np.linspace(0,1,10), np.linspace(0,0,10), tube_radius=0.025, color=(0,1,0), figure=fig)
	mayavi.mlab.plot3d(np.linspace(0,0,10), np.linspace(0,0,10), np.linspace(0,1,10), tube_radius=0.025, color=(0,0,1), figure=fig)
	
	mayavi.mlab.show()


if __name__=='__main__':
	check_plane()
	check_sphere()