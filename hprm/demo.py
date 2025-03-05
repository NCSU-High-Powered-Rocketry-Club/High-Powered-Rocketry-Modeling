import math

import hprm


def main():
    print("Testing out the High Powered Rocket Modeling Program")

    # Define the Test Vehicle
    test_vehicle = hprm.Rocket(
        10.0,   # mass kg
        0.3,    # drag coefficient
        0.005,  # cross-sectional refference area
        0.05,   # lifting-surface refference area
        5.0,    # Moment of Inertia (for a 3DoF rocket)
        0.5,    # Dimensional stability margin (distance between cp and cg)
        0.2     # Derivative of lift coefficient with alpha(angle of attack)
    )

    ode = hprm.OdeMethod.Euler(1e-2)

    state_info = hprm.PyState(1) # 3DoF
    state_info.u1 = [0.0, 100.0]
    state_info.u3 = [0.0, 0.0, math.pi/2.0,
                     0.0, 100.0, 0.0]

    # Run the simulation
    hprm.main(test_vehicle, state_info, ode)

    # Run the simulation
    state_info.ndof = 3 # 3DoF
    hprm.main(test_vehicle, state_info, ode)
main()
