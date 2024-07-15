function output = anglePIDcontroller(input, setpoint, kp, ki, kd)
    persistent lastError cumError previousTime;
    
    if isempty(lastError)
        lastError = 0;
        cumError = 0;
        previousTime = 0;
    end
    
    currentTime = toc; % Get the current time in seconds since the last call to tic
    elapsedTime = currentTime - previousTime; % Calculate elapsed time
    
    error = setpoint - input; % Determine the error between setpoint and measurement
    cumError = cumError + error * elapsedTime; % Calculate the integral of the error
    rateError = (error - lastError) / elapsedTime; % Calculate the derivative of the error
    
    output = kp * error + ki * cumError + kd * rateError; % Calculate the PID output
    
    lastError = error; % Store previous error
    previousTime = currentTime; % Store previous time
end
