from sklearn.neighbors import LocalOutlierFactor # pip install scikit-learn
from sklearn.svm import OneClassSVM
from sklearn.ensemble import IsolationForest
from sklearn.linear_model import RANSACRegressor, LinearRegression
import pdb
import math
import numpy as np

classifier2 = lambda x : (-1 if x == False else 1) # classificate with LOF score

def add_square_feature(X):
    X = np.concatenate([(X**2).reshape(-1,1), X], axis=1)
    # print(X.shape)
    return X

def kernelTrick(X):
    X = X * np.pi /180
    X = np.concatenate([(-np.sin(X)).reshape(-1,1), (-np.cos(X)).reshape(-1,1)], axis=1)

    return X

def ransac_cos(objs):
    xDomian = np.arange(-90,90,5)[:, np.newaxis]
    numNeighbor = math.ceil(len(objs) /2)

    idxs = objs[:,0:2]


    clf = RANSACRegressor(estimator=LinearRegression(fit_intercept=False, n_jobs=-1), residual_threshold = 0.13, max_trials = 50)
    #if fit_intercept= False then fitting model is v(\theta) = - vx*sin(\theta) -vy*cos(\theta). if not the fitting model is v(\theta) = - vx*sin(\theta) -vy*cos(\theta) + c.
    # print(clf.estimator_)
    # clf.estimator_.get_params()
    clf.fit(kernelTrick(idxs[:,0].reshape(-1,1)), idxs[:,1])

    # ret = clf.score(idxs[:,0].reshape(-1,1),idxs[:,1])
    # ret = 

    ret = [classifier2(x) for x in clf.inlier_mask_]
    line_y = clf.predict(kernelTrick(xDomian))
    v_y = -1 * clf.predict(kernelTrick(np.array([[0]])))
    v_x = -1 * clf.predict(kernelTrick(np.array([[90]])))
    # print("n_feature : {}".format(clf.n_features_in_))
    # print("vx : {:.2f}m/s, vy : {:.2f}m/s".format(clf.estimator_.coef_[0], clf.estimator_.coef_[1]))
    
    return ret, xDomian, line_y, clf.estimator_.coef_[0], clf.estimator_.coef_[1]

def ransac(objs):
    xDomian = np.arange(-90,90,5)[:, np.newaxis]
    numNeighbor = math.ceil(len(objs) /2)

    idxs = objs[:,0:2]

    clf = RANSACRegressor(estimator=LinearRegression(n_jobs=-1), residual_threshold = 0.16, max_trials = 25)
    clf.fit(add_square_feature(idxs[:,0].reshape(-1,1)), idxs[:,1])
    # ret = clf.score(idxs[:,0].reshape(-1,1),idxs[:,1])
    # ret = 

    ret = [classifier2(x) for x in clf.inlier_mask_]
    line_y = clf.predict(add_square_feature(xDomian))

    return ret, xDomian, line_y

def outlierDetection(objs, idx = 2, LOF = -1.0):

    classifier = lambda x : (-1 if x < LOF else 1) # classificate with LOF score

    # print(objs)
    if 1:
        numNeighbor = math.ceil(len(objs) /2)
    else:
        numNeighbor = 15

    idxs = objs[:,idx]
    # print(idxs)
    for i, idx in enumerate(idxs):
        # print(idx)
        if idx >= 32:
            # pdb.set_trace()
            idxs[i] = idx - 64
    # print("Input : ",idxs)
    idxs = idxs.reshape(-1,1)

    clf = LocalOutlierFactor(n_neighbors=numNeighbor, algorithm = 'ball_tree', p =2, metric = 'minkowski')
    ret = clf.fit_predict(idxs)
    # print("n_feature : {}".format(clf.n_features_in_))
    # print("n_neighbors_ :", clf.n_neighbors_)
    # print("negative_outlier_factor : ", clf.negative_outlier_factor_)
    # print("offset : ", clf.offset_)
    # print("out :", ret)
    ret = [classifier(x) for x in clf.negative_outlier_factor_]
        # print(ret)
    # elif 0:
    #     clf = OneClassSVM(kernel='linear', gamma='scale').fit(idxs)
    #     ret = clf.predict(idxs)
    # else:
    #     clf = IsolationForest(random_state=0).fit(idxs)
    #     ret = clf.predict(idxs)
    # print(clf.negative_outlier_factor_)
   
    return ret