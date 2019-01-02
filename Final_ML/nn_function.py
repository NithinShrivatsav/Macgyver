import numpy as np
import csv
import pandas
from sklearn.externals import joblib
import os
## X is the feature vector and n is the number of objects in the scene
def nn_func(X,n):
    # Path to the weights
    folder_path_Weights = "/home/nithins/Desktop/ThreeDCV/Machine Learning/NN_ESF.joblib"
    # folder_path_Weights = "/home/nithins/Desktop/ThreeDCV/Machine Learning/NN_SHOTC.joblib"

    # The model is loaded
    model = joblib.load(folder_path_Weights)

    # The list Rank is used to store the rank for all the possible part combinations
    Rank = [[0 for k in range(n)]]
    print("Probabilities for all the classes")
    a = model.predict_proba(X)
    for i in range(n):
        for j in range(n):
            if i==j:
                continue
            ## The numbers 3 and 6 needs to be changed according to the requirement
            ## Target Names:
            ## 0 - Contain Contain
            ## 1 - Cut Blade
            ## 2 - Flip Spatula
            ## 3 - Hit Head
            ## 4 - Poke Tip
            ## 5 - Scoop Scoop
            ## 6 - Handle
            Rank.append([a[i][3],a[j][6],i+1,j+1])
            print("Part Primary:%s"%i)
            print("Part Secondary:%s"%j)
            print("Probability of handle: %s" % a[j][6])
            print("Probability of primary part: %s"% a[i][3])
    Rank.pop(0)
    # Here is where the ranking of the probabilities takes place
    for i in range(len(Rank)-1,0,-1):
        for j in range(i):
            if Rank[j][0]<Rank[j+1][0]:
                temp = Rank[j]
                Rank[j] = Rank[j+1]
                Rank[j+1] = temp
    print(Rank)
