import numpy as np
import csv
import pandas
from sklearn import svm
from sklearn.ensemble import RandomForestClassifier
from sklearn.externals import joblib
import os
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.decomposition import PCA

## ESF Descriptors
folder_path_Descriptor = "/home/nithins/Desktop/ThreeDCV/Real Dataset Descriptors/SHOTC_Descriptors/shotc_descriptors_HIT.csv"
# folder_path_Weights = "/home/nithins/Desktop/ThreeDCV/Machine Learning/Weights/ESF Weights/ESF_RF.joblib"
folder_path_Weights = "/home/nithins/Desktop/ThreeDCV/Machine Learning/NN_SHOTC.joblib"

## SHOTA Descriptors
# folder_path_Descriptor = "/home/nithins/Desktop/ThreeDCV/Real Dataset Descriptors/SHOTA_Descriptors/shota_descriptors_FLIP.csv"
# folder_path_Weights = "/home/nithins/Desktop/ThreeDCV/Machine Learning/Weights/SHOTA Weights/SHOTA_SVM.joblib"


## SHOTC Descriptors
# folder_path_Descriptor = "/home/nithins/Desktop/ThreeDCV/Real Dataset Descriptors/SHOTC_Descriptors/shotc_descriptors_FLIP.csv"
# folder_path_Weights = "/home/nithins/Desktop/ThreeDCV/Machine Learning/Weights/SHOTC Weights/SHOTC_SVM.joblib"


data = pandas.read_csv(folder_path_Descriptor)
print(data)
X = data.drop(['Function','Name'],axis = 1)
X = X.values
n = 4
# Matrix = [[[0 for z in range(3)] for x in range(n)] for y in range(n)]
Rank = [[0 for k in range(4)]]
model = joblib.load(folder_path_Weights)
a = model.predict_proba(X)
print(a)
for i in range(n):
    for j in range(n):
        if i==j:
            ##Matrix[i][j]=0.0
            continue
        # Matrix[i][j] = [a[i][5] + a[j][4],i+1,j+1]
        Rank.append([a[i][3],a[j][6],i+1,j+1])
        print("Part Primary:%s"%i)
        print("Part Secondary:%s"%j)
        print("Probability of handle: %s" % a[j][6])
        print("Probability of spatula: %s"% a[i][3])

# print(Matrix[0][1][0])
# print(Matrix)
Rank.pop(0)
for i in range(len(Rank)-1,0,-1):
    for j in range(i):
        if Rank[j][0]<Rank[j+1][0]:
            temp = Rank[j]
            Rank[j] = Rank[j+1]
            Rank[j+1] = temp
print(Rank)










# y1 = [6,5,5,5]
# y2 = [5,6,5,5]
# y3 = [5,5,6,5]
# y4 = [5,5,5,6]
# print("Score:row1 %s" % model.score(X,y1))
# print("Score:row2 %s" % model.score(X,y1))
# print("Score:row3 %s" % model.score(X,y1))
# print("Score:row4 %s" % model.score(X,y1))
# y_pred_handle = model.predict(X)
# y_pred_spatula = model.predict(X)
# for i in range(len(X)):
# 	print("X=%s, Predicted=%s" % (X[i], y_pred_handle[i]))
# print("y_pred and y_true for handle:",y_pred_handle,y_handle)
# print("y_pred and y_true for spatula:",y_pred_spatula,y_spatula)
# for i in range(n):
#     a = X[i]
#     for j in range(n):
# ##        if i==j:
# ##            Matrix[i][j] = 0.0
# ##            continue
#         x_handle = X[j]
#         x_spatula = X[i]
#         y_handle = 5
#         y_spatula = 6
#         y_pred_handle = model.predict(x_handle)
#         y_pred_spatula = model.predict(x_spatula)
#         print("y_pred and y_true for handle:",y_pred_handle,y_handle)
#         print("y_pred and y_true for spatula:",y_pred_spatula,y_spatula)
        #score_handle = model.score(np.array(x_handle),np.array(y_handle))
        #score_spatula = model.score(np.array(x_spatula),np.array(y_spatula))
        #score_final = score_handle + score_spatula
        # print(score_final)



# 4 elements
# 2 parts
# each can take any of the parts - 1,2 1,3 1,4 2,1 2,3 2,4 3,1 3,2 3,4 4,1 4,2 4,3. This is 4P2
# for i in range(n):
#   a = X[i]
#   for j in range(n):
#   if i==j: continue
#   x_handle = X[j]
#   x_spatula = X[i]
#   y_handle = 5
#   y_spatula = 6




## Target Names:
## 1 - Contain Handle
## 2 - Contain Contain
## 3 - Cut Handle
## 4 - Cut Blade
## 5 - Flip Handle
## 6 - Flip Spatula
## 7 - Hit Handle
## 8 - Hit Head
## 9 - Poke Handle
## 10 - Poke Tip
## 11 - Scoop Handle
## 12 - Scoop Scoop

## Target Names:
## 0 - Contain Contain
## 1 - Cut Blade
## 2 - Flip Spatula
## 3 - Hit Head
## 4 - Poke Tip
## 5 - Scoop Scoop
## 6 - Handle
