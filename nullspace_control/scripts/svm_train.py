from random import shuffle
import numpy as np
from sklearn.svm import SVC
from sklearn.model_selection import train_test_split,  cross_val_score
import cPickle
import pickle
import os
from os import path

legs = np.load('../../../../../data/legs.npy', allow_pickle=True)
legs1 = np.load('../../../../../data/legs1.npy', allow_pickle=True)


#fixing a bug because leg1 have label 0
legss1 = legs1.transpose()
legss1[1] = [1 for _ in legs1.transpose()[1]]
legs1 = legss1.transpose()
legs_data = np.concatenate((legs, legs1), axis=0)

walls  = np.load('../../../../../data/walls.npy', allow_pickle=True)
walls1 = np.load('../../../../../data/walls1.npy', allow_pickle=True)
# walls2 = np.load('../../../../../data/walls3.npy', allow_pickle=True)

wall_data = np.concatenate((walls, walls1), axis=0)
# wall_data = np.concatenate((wall_data, walls2), axis=0)

data = np.concatenate((legs_data, wall_data), axis=0)

data = np.transpose(data)
X1, X = data[0], []
y1, y = data[1], []

for x in X1:
    X.append(np.array(x))
for y_ in y1:
    y.append(np.array(y_))

X = np.array(X)
y = np.array(y)

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.20, shuffle=True, random_state=42)
model = SVC(C=1000  , gamma='scale')
model.fit(X_train, y_train)
score = model.score(X_test, y_test)
print("Test data score", score)

cv_scores = cross_val_score(model, X, y, cv=10)
print("Cross validation score (10 folds):", np.mean(cv_scores), "STD:",  np.std(cv_scores))

cwd = os.getcwd()
model_dir = cwd+'/../model'
if not path.exists(model_dir):
    os.mkdir(model_dir)

if np.mean(cv_scores) > 0.89:
    with open('../model/model.svm', 'wb') as f:
        cPickle.dump(model, f)
        print('saved model')

