import numpy as np
from scipy.spatial.distance import cdist
import skfuzzy as fuzz


class GK(object):
    """docstring for GK

    data: n_features X n_dims -> numpy array of (no of features, dimensions) of the dataset.
    n_clusters: No of clusters required. (int)
    m: "Fuzziness" parameter (float)
    gamma: weighting the covariance. (int/float)
    eps: Parameter that controls when the iteration has to stop. If change from previous to current \
            is less than eps, the iteration is stopped (float/int)

    beta: Parameter, usually set to high value, controls the condition number of the matrix. (float/int)
    dims: Optional parameter.

    Returns:
                    fuzzy_matrix: The association matrix which has weights of each point for each cluster.
                    cost_function: List containing history of the cost function in each iteration.
                    cluster_assignments: n_features X 1 array which contains cluster assignments for each point. 


    Use case:
                    data = np.random.random((1000,3))
                    g = GK(data,n_clusters=4,m=1.5,eps=1e-3,gamma=0.1,beta=1e10)
                    fuzzy_matrix ,cost_function,cluster_assignments = g.predict()

    """

    def __init__(self, data, n_clusters=2, m=2.0, eps=1e-4, gamma=0, beta=1e15, dims=None):
        super(GK, self).__init__()
        self.n_clusters = n_clusters
        self.data = data
        self.m = m
        self.eps = eps
        self.gamma = gamma
        self.beta = beta

        if dims is None:
            self.features = self.data.shape[0]
            self.dims = self.data.shape[1]
        else:
            self.features = self.data.shape[0]
            self.dims = dims

    def get_fuzzy_partition_matrix(self, data, n_clusters, m):
        mm = np.mean(self.data, 0)  # Columnwise mean
        mm = np.expand_dims(mm, 1)

        aa = np.max(
            np.abs(self.data - np.ones((self.features, 1)).dot(mm.T)), 0)
        aa = np.expand_dims(aa, 1).T

        v = 2*np.ones((self.n_clusters, 1)).dot(aa)
        v = v*np.random.random((self.n_clusters, self.dims)) - 0.5
        v = v + np.ones((self.n_clusters, 1)).dot(mm.T)

        d = []
        X1 = np.ones((self.features, 1))

        for j in range(self.n_clusters):
            xv = self.data - X1.dot(np.expand_dims(v[j], 0))
            d.append(np.sum(np.power(xv, 2), axis=1))

        d = np.array(d).T

        d = (d + 1e-10) ** (-1/(m-1))
        f0 = d / (np.expand_dims(np.sum(d, 1),
                                 1).dot(np.ones((1, self.n_clusters))))

        return f0, d

    def get_cmeans_partition(self, data, n_clusters, m):
        cntr, u, u0, d, jm, p, fpc = fuzz.cluster.cmeans(
            data.T, n_clusters, m, self.eps, maxiter=3000, init=None)
        return u.T, d.T

    def get_criterion(self, max_iterations, iteration, f0, f):
        if max_iterations is not None:
            if iteration < max_iterations:
                return True
        else:
            if np.max(f0-f) > self.eps:
                return True
        return False

    def predict(self, max_iterations=None):

        # f0,d = self.get_fuzzy_partition_matrix(self.data,self.n_clusters,self.m)
        f0, d = self.get_cmeans_partition(self.data, self.n_clusters, self.m)
        # print (f0.shape, d.shape)
        f = np.zeros((self.features, self.n_clusters))

        rho = np.ones((1, self.n_clusters))

        # NUMPY COV EXPECTS TRANSPOSE.
        A0 = np.eye(self.dims) * \
            np.linalg.det(np.cov(self.data.T))**(1/self.dims)

        iteration = 0
        cost_function = []
        while self.get_criterion(max_iterations, iteration, f0, f):
            iteration += 1
            f = f0

            # Calculate centers

            fm = f**self.m
            sumf = np.sum(fm, 0)
            sumf = np.expand_dims(sumf, 0)

            v = fm.T.dot(self.data) / sumf.T.dot(np.ones((1, self.dims)))

            X1 = np.ones((self.features, 1))

            for j in range(self.n_clusters):
                xv = self.data - X1.dot(np.expand_dims(v[j], 0))

                # Covariance matrix for each cluster
                A = np.ones((self.dims, 1)).dot(
                    np.expand_dims(fm[:, 0], 1).T) * xv.T
                A = A.dot(xv)/sumf[:, j]
                A = (1-self.gamma) * A + self.gamma*(A0**(1/self.dims))

                if np.linalg.cond(A) > self.beta:
                    ev, ed = np.linalg.eig(A)
                    edmax = max(np.diag(ed))
                    ev = np.expand_dims(ev, 1)
                    z = np.diag(np.expand_dims(np.diag(ed), 1))
                    z = np.expand_dims(z, 1)
                    A = ev.dot(z).dot(np.linalg.inv(ev))

                # Distance calculations
                M = (1/np.linalg.det(np.linalg.pinv(A)) /
                     rho[:, j])**(1/self.dims)*np.linalg.pinv(A)
                d[:, j] = np.sum((xv.dot(M)*xv), 1)

            distout = np.sqrt(d)
            cost_function.append(np.sum(np.sum(f0*d, 0), 0))

            # Update fuzzy matrix f0
            d = (d + 1e-10) ** (-1/(self.m-1))
            f0 = d / (np.expand_dims(np.sum(d, 1),
                                     1).dot(np.ones((1, self.n_clusters))))

            cluster_assignments = np.apply_along_axis(np.argmax, 1, f0)

            return f0, cost_function, cluster_assignments
