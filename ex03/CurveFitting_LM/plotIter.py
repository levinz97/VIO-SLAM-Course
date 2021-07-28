import matplotlib.pyplot as plt

# x = range(0,12)
# y = [0.001, 699.051, 1864.14, 1242.76, 414.252,138.084, 46.028, 
#     15.3427, 5.11423, 1.70474, 0.568247, 0.378832]
y = [1e-05, 0.00011, 0.00121, 0.01331, 0.14641, 1.61051, 17.7156, 1.9684, 0.218711, 0.0243012, 0.00270014, 0.000300015, 3.3335e-05, 3.70389e-06]
# original method
y = [0.001, 0.002, 0.008, 0.064, 1.024, 32.768, 2097.15, 699.051, 1398.1, 5592.41, 1864.14, 1242.76, 414.252, 138.084, 46.028, 15.3427, 5.11423, 1.70474, 0.568247]
# method 1
# y = [1e-05, 0.00011, 0.00121, 0.01331, 0.14641, 1.61051, 17.7156, 1.9684, 0.218711, 0.0243012, 0.00270014, 0.000300015, 3.3335e-05, 3.70389e-06]
# method 2 first version iter:11
y = [0.001, 0.000909091, 0.000640061, 0.000384749, 0.000230966, 0.000138626, 8.32001e-05, 4.99341e-05, 2.99687e-05, 1.7986e-05, 1.07944e-05]


with open('./build/app/result.txt','r') as f:
    str = f.read()
    num = ""
    result = []
    for c in str:
        if (c != ','):
            num = num + c
        else:
            result.append(float(num))
            num = ""
    
    # print(f.read())
    # yy = list(map(float,f.read()))
print(f'number of iterations : {len(result)}')
print(result)

fig = plt.figure()
x = range(0, len(result))
ax = fig.add_subplot(111)
ax.plot(x,result, color = 'lightblue', linewidth = 3)
ax.set(title = 'damping parameter Lambda in each iteration',
        ylabel = 'Lambda',
        xlabel = 'iterations')
ax.set(xlim = [0,len(x)])
ax.xaxis.set(ticks = range(1,len(x)))
plt.grid()
plt.savefig('lambda_iterations_method2.png')
plt.show()