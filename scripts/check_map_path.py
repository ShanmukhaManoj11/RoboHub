import matplotlib.pyplot as plt
import numpy as np

##
# Read pgm to plot the map
#
# @param pgm file path
# @return (map width, map height, map raster)
def read_pgm(pgmfile):
    with open(pgmfile, 'rb') as pgmf:
        pgmf.readline() # first line - version P5
        pgmf.readline() # second line - comment
        (width, height) = [int(i) for i in pgmf.readline().split()] # read width and height
        depth = int(pgmf.readline()) # read max pixel value
        assert depth <= 255

        raster = []
        for y in range(height):
            row = []
            for x in range(width):
                row.append(ord(pgmf.read(1)))
            raster.append(row)
        return width, height, raster

##
# Read path from bin 
#
# @param binfile with path data as array of bytes
def read_path_from_bin(binfile):
    path = np.fromfile(binfile, dtype=np.float32, count=-1).reshape([-1,2])
    return path

if __name__ == '__main__':
    path = read_path_from_bin('../data/map_path.bin')
    print(path.shape)

    width, height, raster_map = read_pgm('../data/map.pgm')
    origin = [-33.0, -39.4] # origin of the map
    resolution = 0.02; # resolution of the map
    path_i = []
    for path_point in path:
        i = int(np.floor((path_point[0]-origin[0])/resolution))
        j = int(np.floor((path_point[1]-origin[1])/resolution))
        # raster_map[i][j] = 0
        path_i.append([i,j])
    path_i = np.array(path_i)

    # show map with path
    plt.imshow(raster_map, cmap='gray')
    plt.plot(path_i[:,1], path_i[:,0], 'b-', linewidth=0.1)
    plt.show()