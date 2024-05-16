
def get_pose_error(xd, yd, x, y, phi):
    """ Returns the position and orientation errors. 
        Orientation error is bounded between -pi and +pi radians.
    """
    # Position error:
    x_err = xd - x
    y_err = yd - y
    dist_err = np.sqrt(x_err**2 + y_err**2)

    # Orientation error
    phi_d = np.arctan2(y_err,x_err)
    phi_err = phi_d - phi

    # Limits the error to (-pi, pi):
    phi_err_correct = np.arctan2(np.sin(phi_err),np.cos(phi_err))

    return dist_err, phi_err_correct



# necesitamos el PID 

def pid_controller(e, e_prev, e_acc, delta_t, kp=1.0, kd=0, ki=0):
    """ PID algortithm: must be executed every delta_t seconds
    The error e must be calculated as: e = desired_value - actual_value
    e_prev contains the error calculated in the previous step.
    e_acc contains the integration (accumulation) term.
    """
    P = kp*e                      # Proportional term; kp is the proportional gain
    I = e_acc + ki*e*delta_t    # Intergral term; ki is the integral gain
    D = kd*(e - e_prev)/delta_t   # Derivative term; kd is the derivative gain

    output = P + I + D              # controller output

    # store values for the next iteration
    e_prev = e     # error value in the previous interation (to calculate the derivative term)
    e_acc = I      # accumulated error value (to calculate the integral term)

    return output, e_prev, e_acc


#Calcular la velocidad de las ruedas a partir de las velocidades angulares y lineales
def wheel_speed_commands(u_d, w_d, d, r, Max_Speed_in):
    """Converts desired speeds to wheel speed commands"""
    wr_d = float((2 * u_d + d * w_d) / (2 * r))
    wl_d = float((2 * u_d - d * w_d) / (2 * r))
    
    # If saturated, correct speeds to keep the original ratio
    if np.abs(wl_d) > Max_Speed_in or np.abs(wr_d) > Max_Speed_in:
        speed_ratio = np.abs(wr_d)/np.abs(wl_d)
        if speed_ratio > 1:
            wr_d = np.sign(wr_d)*Max_Speed_in
            wl_d = np.sign(wl_d)*Max_Speed_in/speed_ratio
        else:
            wl_d = np.sign(wl_d)*Max_Speed_in
            wr_d = np.sign(wr_d)*Max_Speed_in*speed_ratio
    
    return wl_d, wr_d
