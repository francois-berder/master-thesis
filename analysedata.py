import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import math


if __name__ == '__main__':

    modelFile = open("results/error", "r")
    reader = csv.reader(modelFile, delimiter=' ')
    
    errorRANSAC = []
    errorLinearModel = []
    errorQuadraticModel = []
    errorMixedModel = []
    for row in reader:
        errorRANSAC.append(float(row[1]))
        errorLinearModel.append(float(row[3]))
        errorQuadraticModel.append(float(row[5]))
        errorMixedModel.append(float(row[7]))

    print(np.mean(errorRANSAC))
    print(np.mean(errorLinearModel))
    print(np.mean(errorQuadraticModel))
    print(np.mean(errorMixedModel))

    print(np.min(errorRANSAC))
    print(np.min(errorLinearModel))
    print(np.min(errorQuadraticModel))
    print(np.min(errorMixedModel))

    print(np.max(errorRANSAC))
    print(np.max(errorLinearModel))
    print(np.max(errorQuadraticModel))
    print(np.max(errorMixedModel))

    print(np.std(errorRANSAC))
    print(np.std(errorLinearModel))
    print(np.std(errorQuadraticModel))
    print(np.std(errorMixedModel))

