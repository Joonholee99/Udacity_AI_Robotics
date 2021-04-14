def update(mean1, var1, mean2, var2):
    new_mean = (1/(var1+var2))*(var1*mean2+var2*mean1)
    new_var = (var1*var2)/(var1+var2)

    return [new_mean, new_var]

def predict(mean1, var1, mean2, var2):
    new_mean = mean1 + mean2
    new_var = var1 + var2

    return [new_mean, new_var]
    
print(predict(10,4,12,4))


measurements = [5,6,7,9,10]
motions = [1,1,2,1,1]

measurements_sig = 4
motions_sig = 2
mu = 0
sig = 0.0000000001

def kalman_filter(measurements, motions,mu,sig,measurements_sig,motion_sig):
    for i in range(len(measurements)):
        [mu, sig] = update(measurements[i],measurements_sig,mu,sig)
        print("update: ",[mu,sig])
        [mu, sig] = predict(motions[i],motion_sig,mu,sig)
        print("predict: ",[mu,sig])

    return [mu, sig]

[mu_final, sig_final] = kalman_filter(measurements,motions,mu,sig,measurements_sig,motions_sig)
# print([mu_final, sig_final])