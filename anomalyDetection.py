from sklearn.neighbors import LocalOutlierFactor # pip install scikit-learn
from sklearn.svm import OneClassSVM
from sklearn.ensemble import IsolationForest
import pdb
import math

classifier = lambda x : (-1 if x < -1.0 else 1)

def outlierDetection(objs):
    # print(objs)
    if 1:
        numNeighbor = math.ceil(len(objs) /2)
    else:
        numNeighbor = 15
    if 0:
    
        idxs = objs[:,0:3:2]
        # print(idxs)
        for i, idx in enumerate(idxs):
            # print(idx)
            if idx[1] >= 32:
                # pdb.set_trace()
                idxs[i][1] = idx[1] -64
        # print(idxs)
    else:

        idxs = objs[:,2]
        # print(idxs)
        for i, idx in enumerate(idxs):
            # print(idx)
            if idx >= 32:
                # pdb.set_trace()
                idxs[i] = idx - 64
        # print("Input : ",idxs)
        idxs = idxs.reshape(-1,1)
    
    if 1:
        
        clf = LocalOutlierFactor(n_neighbors=numNeighbor, algorithm = 'ball_tree', p =2, metric = 'minkowski')
        ret = clf.fit_predict(idxs)
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