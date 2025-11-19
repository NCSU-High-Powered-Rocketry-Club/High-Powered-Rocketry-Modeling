from hprm import Rocket, ModelType, IntegrationMethod

def main():
    # Make a rocket with all its parameters/constants
    # Filled in: Mass=10.0, Cd=0.3, A_ref=0.005, A_lift=0.05, Inertia=5.0, Margin=0.5, CLa=0.2
    rocket = Rocket(10.0, 0.3, 0.005, 0.05, 5.0, 0.5, 0.2)

    initial_height = 0.0
    initial_velocity = 100.0 # Starting at 0 won't fly without a motor model, assuming coast start

    # This would get you the entire flight data
    sim_data = rocket.simulate_flight(initial_height, initial_velocity, ModelType.OneDOF, IntegrationMethod.RK45)
    
    # Verification print
    print(f"Full Sim Data Points: {sim_data.get_len()}")

    current_height = 124.3
    current_velocity = 182.1

    # This would just get you apogee
    apogee = rocket.predict_apogee(current_height, current_velocity, ModelType.OneDOF, IntegrationMethod.RK45)
    
    # Verification print
    print(f"Predicted Apogee: {apogee}")

if __name__ == "__main__":
    main()