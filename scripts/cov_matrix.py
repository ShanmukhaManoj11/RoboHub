import numpy as np

if __name__=='__main__':
	_xrange, _yrange, _zrange = 2, 2, 2
	points = np.random.rand(3, 5)
	points[0,:] = points[0,:]*_xrange - _xrange*0.5
	points[1,:] = points[1,:]*_yrange - _yrange*0.5
	points[2,:] = points[2,:]*_zrange - _zrange*0.5
	print(points)

	mean = np.mean(points, axis=1, keepdims=True)

	points = points - mean
	cov = np.matmul(points, points.T)/points.shape[1]
	print(cov)