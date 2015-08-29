import csv
import numpy as np
import matplotlib.pyplot as plt

modelFile = open("data/modeldata", "r")
reader = csv.reader(modelFile, delimiter=' ')
data = []
for row in reader:
    data.append((-float(row[0]), float(row[1]), -float(row[2])))
tmp = list(zip(*data))

plt.figure()
plt.xlabel('bending angle', fontsize=18)
plt.ylabel('x', fontsize=18)
plt.scatter(tmp[0], tmp[1])
plt.savefig('data/modelx.png')

plt.figure()
plt.xlabel('bending angle', fontsize=18)
plt.ylabel('y', fontsize=18)
plt.scatter(tmp[0], tmp[2])
plt.savefig('data/modely.png')
