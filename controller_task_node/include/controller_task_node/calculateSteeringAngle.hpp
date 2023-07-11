  double calculateSteeringAngle(double lateral_deviation,
                                double previous_lateral_error, double integral_of_lateral_error){
  // Define PID gains for lateral control
  double Kp = 1.0; // Proportional gain
  double Ki = 0.0; // Integral gain
  double Kd = 0.0; // Derivative gain

  // Calculate lateral error (difference between desired and current lateral position)
  //double lateral_error = desired_lateral_position - current_lateral_position;

  // Calculate PID terms
  double proportional_term = Kp * lateral_deviation;
  double integral_term = Ki * integral_of_lateral_error; // Accumulate integral error over time
  double derivative_term = Kd * (lateral_deviation - previous_lateral_error); // Calculate rate of change of error

  // Calculate total steering command
  double steer_cmd = proportional_term + integral_term + derivative_term;

  // Apply any necessary steering limits or constraints to steer_cmd

  //  update integral for next iteration
  integral_of_lateral_error += lateral_deviation;
  
  return steer_cmd;

  }
