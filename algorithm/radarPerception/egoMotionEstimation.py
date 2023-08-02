from sklearn.linear_model import RANSACRegressor, LinearRegression

import math
import numpy as np

classifier2 = lambda x : (-1 if x == False else 1) # classificate with LOF score

def _kernelTrick(X):
    X = X * np.pi /180
    X = np.concatenate([(-np.sin(X)).reshape(-1,1), (-np.cos(X)).reshape(-1,1)], axis=1)

    return X

def _kernelTrick_3D(theta, phi):

    theta = theta * np.pi / 180
    phi = phi * np.pi / 180
    X = np.concatenate([(-np.sin(theta)*np.cos(phi)).reshape(-1,1), (-np.cos(theta)*np.cos(phi)).reshape(-1,1)], axis=1)

    return X


class egoMotionEst():
    def __init__(self):
        self.clf = RANSACRegressor(estimator=LinearRegression(fit_intercept=False, n_jobs=-1), residual_threshold = 0.20, max_trials = 15)
        self.xDomian = np.arange(-90,90,5)[:, np.newaxis]

    def __call__(self, objs):
        self.clf.fit(_kernelTrick(objs[:,0].reshape(-1,1)), objs[:,1])
        ret = [classifier2(x) for x in self.clf.inlier_mask_]
        line_y = self.clf.predict(_kernelTrick(self.xDomian))

        return ret, self.xDomian, line_y, self.clf.estimator_.coef_[0], self.clf.estimator_.coef_[1]
    """ return flags, line_x, line_y, vx of ego, vy of ego """
    
class egoMotionEst_3D():
    def __init__(self):
        self.clf = RANSACRegressor(estimator=LinearRegression(fit_intercept=False, n_jobs=-1), residual_threshold = 0.20, max_trials = 15)
        self.xDomian = np.arange(-90,90,5)[:, np.newaxis]

    def __call__(self, objs):
        self.clf.fit(_kernelTrick_3D(objs[:,0].reshape(-1,1), objs[:,0].reshape(-1,1)), objs[:,1])
        ret = [classifier2(x) for x in self.clf.inlier_mask_]
        line_y = self.clf.predict(_kernelTrick(self.xDomian))

        return ret, self.xDomian, line_y, self.clf.estimator_.coef_[0], self.clf.estimator_.coef_[1]
    """ return flags, line_x, line_y, vx of ego, vy of ego """
    