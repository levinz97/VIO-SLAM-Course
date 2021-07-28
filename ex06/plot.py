
import matplotlib.pyplot as plt
import csv
import numpy as np

x = []
y = []

# with open('./build/result.csv', 'r') as f:
#     plots = csv.reader(f, delimiter=',')
#     for row in plots:
#         for item in row:
#             if item != '':
#                 x.append(item)
#             else:
#                 break
#             print(item)

# print(x[0:7])

# plt.plot(x[0:7], x[7:])
# plt.show()

f = open('./build/result.csv', 'r')
csvReader = csv.reader(f)
data = list(csvReader)

for i in range(6):
    x.append(float(data[0][i+1]))
    y.append(np.log10(float(data[1][i+1])))
y = [90.292,108.664,173.653,2166.68,32029.1,178.18,0.215633]
x = range(8)

plt.plot(x[1:],y)
# plt.xlabel('noise level')
# plt.ylabel('scond smallest / smallest singular value (in logspace)')
plt.show()