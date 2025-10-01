// PI controller variables
float Kp = 1.0;       // proportional gain
float Ki = 0.5;       // integral gain
float dt = 0.01;      // time step (s)
float integral = 0.0; // integral accumulator

float minPWM = 0.0;   // minimum PWM
float maxPWM = 255.0; // maximum PWM

float computePI(float setpoint, float current) {
    // 1️⃣ Compute the error
    float error = setpoint - current;

    // 2️⃣ Compute candidate integral (what integral WOULD be if we integrate now)
    float integralCandidate = integral + error * dt;

    // 3️⃣ Compute candidate output using candidate integral
    float outputCandidate = Kp * error + Ki * integralCandidate;

    float output;

    // 4️⃣ Anti-windup via clamping
    if (outputCandidate > maxPWM) {
        // Output would exceed maxPWM
        output = maxPWM;

        // Only allow integral to change if it would help reduce saturation
        if (error < 0) {
            integral = integralCandidate; // unwind integral
        }
        // else integral is frozen (do not accumulate)
    }
    else if (outputCandidate < minPWM) {
        // Output would go below minPWM
        output = minPWM;

        // Only allow integral to change if it would help reduce saturation
        if (error > 0) {
            integral = integralCandidate; // unwind integral
        }
        // else integral is frozen
    }
    else {
        // Output is within limits → normal integration
        integral = integralCandidate;
        output = outputCandidate;
    }

    // 5️⃣ Return the final output
    return output;
}
