import numpy as np
from ransac import *

def augment(xyzs):
    axyz = np.ones((len(xyzs), 4))
    axyz[:, :3] = xyzs
    return axyz

def estimate(xyzs):
    axyz = augment(xyzs[:3])
    return np.linalg.svd(axyz)[-1][-1, :]

def is_inlier(coeffs, xyz, threshold):
    return np.abs(coeffs.dot(augment([xyz]).T)) < threshold

if __name__ == '__main__':
    import matplotlib
    import matplotlib.pyplot as plt
    from mpl_toolkits import mplot3d
    import plyfile
    fig = plt.figure()
    ax = mplot3d.Axes3D(fig)

    def plot_plane(a, b, c, d):
        xx, yy = np.mgrid[0:1:0.1, 0:1:0.1]
        return xx, yy, (-d - a * xx - b * yy) / c

    def reduce_data(xyzs):
        # del_seq = np.array([i for i in range(len(xyzs)) if i%2==0])
        # del_seq = np.s_[::2]
        # print(del_seq)
        xyzs = np.delete(xyzs, np.s_[::2], axis=0)
        return xyzs

    n = 100000
    ip = 0
    op = 100000
    max_iterations = 100
    goal_inliers = n * 0.2

    # test data
    plydata = plyfile.PlyData.read("data/1.ply")
    xi = plydata.elements[0].data['x']
    yi = plydata.elements[0].data['y']
    zi= plydata.elements[0].data['z']

    xyzs = np.vstack((xi.T, yi.T, zi.T)).T
    xyzs = reduce_data(xyzs)
    xyzs = reduce_data(xyzs)
    xyzs = reduce_data(xyzs)
    xyzs = reduce_data(xyzs)

    ax.scatter3D(xyzs.T[0][ip:op], xyzs.T[1][ip:op], xyzs.T[2][ip:op], s=0.1)

    # RANSAC
    m, b = run_ransac(xyzs[ip:op], estimate, lambda x, y: is_inlier(x, y, 0.01), 3, goal_inliers, max_iterations)
    print("M values = {}".format(m))
    a, b, c, d = m
    xx, yy, zz = plot_plane(a, b, c, d)
    # print("plane values= x:{},\ny:{},\nz{}".format(xx,yy,zz))
    ax.plot_surface(xx, yy, zz, color=(0, 1, 0, 0.5))

    plt.show()
