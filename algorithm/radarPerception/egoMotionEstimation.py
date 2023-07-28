from sklearn.linear_model import RANSACRegressor, LinearRegression

import math
import numpy as np

classifier2 = lambda x : (-1 if x == False else 1) # classificate with LOF score

def _kernelTrick(X):
    X = X * np.pi /180
    X = np.concatenate([(-np.sin(X)).reshape(-1,1), (-np.cos(X)).reshape(-1,1)], axis=1)

    return X

class egoMotionEst():
    def __init__(self):
        self.clf = RANSACRegressor(estimator=LinearRegression(fit_intercept=False, n_jobs=-1), residual_threshold = 0.13, max_trials = 50)
        self.xDomian = np.arange(-90,90,5)[:, np.newaxis]

    def process(self, objs):
        idxs = objs[:,0:2]
        self.clf.fit(_kernelTrick(idxs[:,0].reshape(-1,1)), idxs[:,1])
        ret = [classifier2(x) for x in self.clf.inlier_mask_]
        line_y = self.clf.predict(_kernelTrick(self.xDomian))

        return ret, self.xDomian, line_y, self.clf.estimator_.coef_[0], self.clf.estimator_.coef_[1]
    """ return flags, line_x, line_y, vx of ego, vy of ego """
    
    