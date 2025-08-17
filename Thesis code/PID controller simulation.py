import numpy as np
import matplotlib.pyplot as plt
import control.matlab as control
import control as control
from matplotlib.patches import FancyArrowPatch

def pid_controller(setpoint, measured_point, Kp, Ki, Kd, previous_error, integral, dt, y_prev ):
    error = setpoint - measured_point
    P = Kp * error
    D = Kd * (error - previous_error) / dt
    integral += error * dt

    # Unsaturated control
    u = P + Ki*integral + D

    #Actuator dynamics
    K = 1
    alpha = dt / (0.0379 + dt)
    beta = 0.0379 / (0.0379 + dt)
    y = alpha *  K * u + beta * y_prev

    #Saturation
    if y > 10 * np.pi / 180:  
        integral -= error * dt  
        y = 10 * np.pi / 180
    elif y < -10 * np.pi / 180:
        integral -= error * dt  
        y = -10 * np.pi / 180
    
    return y, error, integral, y_prev
    


if __name__ == "__main__":
    b = 1
    v = 4  # Velocity [m/s]
    rho = 1.184  # Air density [kg/m^3]
    S = 0.134  # Reference area [m^2]
    C_L_p = -0.662 # Roll moment coefficient [1/rad]
    C_L_delta_a = 0.289  # Control derivative [1/rad]
    Ixx_total = 1.72e-2     # Moment of inertia [kg.m^2]
    Lp = 1/4 * rho * v * S * b**2 * C_L_p      # Aerodynamic moment [Nm]
    L_delta_a = 1/2 * rho * v**2 * S * b * C_L_delta_a    # Control derivative [Nm/rad]
    G = control.tf([L_delta_a], [Ixx_total, -Lp, 0])

    Ku = 6.9
    Tu = 3.485-2.884
    print(f"Ku: {Ku}, Tu: {Tu}")

    Kp = Ku*0.8
    Ki = 0
    Kd = 0.1*Ku*Tu

    print(f"Kp: {Kp}, Ki: {Ki}, Kd: {Kd}")

    # Simulation Setup
    t_final = 5         # total simulation time [s]
    dt = 0.02 #Time step
    t = np.arange(0, t_final, dt)    
    N = len(t)

    # State variables
    phi = np.zeros(N)     
    p = np.zeros(N)     

    control_values = np.zeros(N)  
    error_values = np.zeros(N)  

    #PID controller
    desired_phi = 0  # setpoint
    first_phi = 0    # Initial process variable in radians
    previous_error = desired_phi - first_phi  # Initial error
    integral = 0  # Integral term
    y_prev = 0  # Previous control action
    time = 0 # Time counter

    for i in range(1, N):  
        u, error, integral, u_prev = pid_controller(desired_phi, phi[i-1], Kp, Ki, Kd, previous_error, integral, dt, y_prev)
        if time >= 1:  
            desired_phi = np.deg2rad(10) 
        # u = 0   #Step input command
        # if time >= 1:  
        #     u = np.deg2rad(10)   

        # Plant dynamics
        p_dot = (L_delta_a * u + Lp * p[i-1]) / Ixx_total
        phi_dot = p[i-1]

        # State updates
        p[i] = p[i-1] + p_dot * dt
        phi[i] = phi[i-1] + phi_dot * dt

        control_values[i] = u
        error_values[i] = error

        previous_error = error
        y_prev = u
        time += dt


phi_deg = np.rad2deg(phi)

fig, axes = plt.subplots(2, 1, sharex=False, figsize=(10, 7))
ax_roll, ax_ctrl = axes 

for ax in (ax_roll, ax_ctrl): 
    ax.minorticks_on()
    ax.grid(which='major', linestyle='-', linewidth=0.7)
    ax.grid(which='minor', linestyle=':', linewidth=0.5)

ax_roll.plot(t, phi_deg)
ax_roll.axhline(10, color='red', linestyle='--', linewidth=0.8, label='Setpoint')
ax_roll.set_xlabel('Time [s]')
ax_roll.set_ylabel('Roll Angle [°]')
# ax_roll.set_ylim(-0.6, 12)
ax_roll.set_title("Step-input response of Roll Angle")
ax_roll.grid(True)
ax_roll.legend()

ax_ctrl.plot(t, np.rad2deg(control_values))
ax_ctrl.set_xlabel('Time [s]')
ax_ctrl.set_ylabel('Aileron deflection [°]')
ax_ctrl.set_title('Control Input')
# ax_ctrl.set_ylim(-2, 10.6)
ax_ctrl.grid(True)

plt.tight_layout()
plt.show()
