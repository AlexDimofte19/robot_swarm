import numpy as np
import pickle
from os import path
import os

class Classifier(object):
    def __init__(self):
        cwd = os.getcwd()
        model_dir = cwd+'/../model/model.svm'
        if path.exists(model_dir):
            self.model = pickle.load(open('../model/model.svm', 'rb'))
        else:
            self.model = None

    def predict(self, data):
        '''
        @Params
        `data` -> list [[girth, width, depth], ..]
    
        @Return
        `predictions` -> list with predictions

        @Description
        Predict label for each entry in `data`
        '''
        X = []
        for d in data:
            X.append(np.array(d))
        X = np.array(X)
        return self.model.predict(X)