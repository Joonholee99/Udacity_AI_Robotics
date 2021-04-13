import math

def update(mean1, var1, mean2, var2):
    new_mean = (1/(var1+var2))*(var1*mean2+var2*mean1)
    new_var = (var1*var2)/(var1+var2)

    return [new_mean, new_var]

a = update(10,8,13,2)
print(a)