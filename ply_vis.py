import numpy as np
from open3d import *    
import sys

def main(input_file):
    ply = read_point_cloud(input_file) # Read the point cloud
    draw_geometries([ply]) # Visualize the point cloud     

if __name__ == "__main__":
    main(sys.argv[1])
