import numpy as np
import matplotlib as mpl;
import matplotlib.pyplot as plt;



array1 = [
19.0,
19.0,
19.0,
19.0,
19.0,
19.0,
20.0,
21.0,
22.0,
26.0,
24.0,
23.0,
21.0,
20.0,
19.0,
19.0,
19.0,
19.0,
19.0,
19.0,
19.0,
19.0,
19.0,
19.0,
19.0,
19.0,
19.0,
19.0,
19.0,
19.0
]

array2 = [19.0 for _ in range(30)]
array3 = array1[::-1]
n = 29
y = []
x = []

for i in range(n):
    y.append(array1[i])
    x.append(i)

sumy, sumx, sumxy, sumxx = 0,0,0,0

for i in range(n):
    sumy += y[i]
    sumx += x[i]
    sumxy += y[i]*x[i]
    sumxx += x[i]*x[i]

print("\n\nThe calculated value of sumx, sumy, sumxy, sumxx is : ", sumx, ",", sumy, ",",sumxy," and ",sumxx, ".")

a=(sumx*sumxy-sumy*sumxx)/(sumx*sumx-n*sumxx)
b=(sumy*sumx-n*sumxy)/(sumx*sumx-n*sumxx)



print("\n\nThe calculated value of a and b is : ", a, " and ", b, ".")
print("\n\nThe best fit value of curve is : y = ", a, " + ", b, "x.\n\n")

# y = bx+a

def f(t1, b, a):
    return (a+(b*t1))

# t1 = np.arange(0.0, 5.0, 0.1)
t2 = np.arange(0.0, 30.0, 0.5)

plt.figure()
plt.subplot(211)
plt.plot(t2, f(t2, b, a), 'k')
plt.show()