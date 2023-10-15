#from:
# https://thekalmanfilter.com/kalman-filter-explained-simply/

#Kalman filter uses observed values to estimate actual values of same variable AND the values of unmeasured variables
#It can estimate unmeasured variables by looking at the covariance with the measured variable
    #For example, position observations used to estimate actual position AND velocity of object

#Estimates of actual values (of measured variables) is useful to smooth measurements that may be noisy
#Estimates of unmeasured variables is main point, is useful to predict value that cant be measured or observed w/ accuracy

#Primary components are:
    #system state (trying to be estimated)
    #system state error covariance (xxx)
    #Kalman gain (xxx)
    #

#Variables:
# m is number of measurement parameters (eg. x, y for 2-D coordinates), n is number of prediction params??
    #outputs
    #   x, state variable (nx1)
    #   x_hat, state variable estimation (nx1)
    #   P, state covariance matrix (nxn)

    #input
    #   z, observation (mx1)
    #   u, control input (idk)
    #   w, process noise (nx1)
    #   v, measurement noise (mx1)
    #   R, measurement covariance matrix (mxm)
    #   T, time tag for estimate?

    #models or internal values
    #   A, state transition matrix (nxn). Applied to previous state (plus process noise) to get current one (assumes no control inputs)
    #   B, control-input matrix (idk). Applied to control input (if exists) and added to state prediction (along w/ F*x_k-1 + w_k-1)
    #   H, state-to-measurement matrix (mxn). Measurement equal to H applied to current state, plus measurement noise
    #   Q, process noice covariance matrix (nxn)
    #   K, Kalman gain (nxm)


#examples for 1-D motion case
    #outputs
    #   x_k = [x, x_dot] (n = 2 = number of prediction params???)
    #   x_hat =
    #   P_k = [[var of x, covar. of x & x_dot], [covar. of x & x_dot, var of x_dot]] * T_k

    #input
    #   z_k = [x_m,k, y_m,k] (measurements of x,y at timestep k)
    #   R_k = [[var of x_m, covar. of x_m & y_m], [covar. of x_m & y_m, var of y_m]] (variances of (x, y) through time)

    #models or internal values
    #   A = xx
    #   H = xx
    #   Q = xx
    #   K = xx


# Steps:

    # 1) initialize estimate and error covariance of system state with x_1, P_1
    # 2) repeat initialization with x_2, P_2 (I assume to give enough data points to work with?)
    # 3) predict/"propagate" system state (x_hat = A * x_2) and state error covar. to t (P_hat = A * P_2 * A^T)
    # 4) compute Kalman Gain
    # 5) update/"correct" system state and state error covar. to t (same variables as in step 3) using predictions and measurements:
    #     x_3 = x_hat + K_3(z_3 - (H * x_hat)) (k=3 here)
    #     P_3 = P_hat - K*H*P_hat
    # 6) repeat step 3-5 for each k > 3




.
.
.
.
.
keep reading starting at Kalman Filter Radar Tracking Tutorial
make sure and keep straight between x from (x,y) coordinate and x the state variable (eg. x_k (state at kth timestep = [x, y, x_dot, y_dot]))

#k is timestep number

# a is gain term
# x is transmitted signal
# n is additive noise
#to find observed signal (y, called z in other source) for each timestep:   y_k = a_k*x_x + n_k

#but already know y (b/c it is observed) and want to estimate x (x_k_hat is estimate of x_k)
#error is:   f(e_k) = f(x_k - x_k_hat),
#the RHS is usually taken as:   f(e_k) = (x_k - x_k_hat)^2 as squared fns are both positice & increase monotonically (desired traits)


#                                Measurements  |
#                                              v
# Initial Estimates  ->  Kalman Gain  ->  Updated Estimate  ->  Updated Covariance  ->  Project into k+1 timestep
#                                              |                                                  |
#                                              v                                                  v
#                                       Updated state estimates                              Projected estimates
