% Define the COM port and the baud rate
comPort = 'COM5'; % COM5 used for Arduino UNO %
baudRate = 9600;  

% Open the serial port for communication with Arduino
arduinoSerial = serialport(comPort, baudRate);

% Configure the terminator
configureTerminator(arduinoSerial, "CR/LF"); 

% Set up the figure for the pendulum simulation
figure;
axis equal;
axis([-200 200 0 120]);
hold on;
pendulumLength = 100; % Length of the pendulum
cartWidth = 20; % Width of the cart
cartHeight = 10; % Height of the cart
trailLength = 100;
trailX = NaN(1, trailLength);
trailY = NaN(1, trailLength);
trailPlot = plot(trailX, trailY, 'g');
xlabel('Cart Position');
ylabel('Pendulum Angle');
title('Inverted Pendulum on Cart Simulation');

% Text annotation for displaying current angle
angleText = text(-120, 120, sprintf('Current Angle: %.2f degrees', 0), 'FontSize', 12);

% PD Controller Gains
Kp = 50; % Proportional gain
Kd = 10; % Derivative gain
previousError = 0;
previousTime = tic;

% Desired Angle / Position
desiredAngle = 0;

% Initial plot for the pendulum
cart = rectangle('Position', [0 -cartHeight/2 cartWidth cartHeight], 'FaceColor', [0.5 0.5 0.5]);
h = plot([0 0], [0 pendulumLength], 'r', 'LineWidth', 2); % Pendulum
hx = plot(0, pendulumLength, 'bo', 'MarkerFaceColor', 'b'); % Pendulum end point
trailIdx = 1;

% Read and store data from the serial port
try
    while true

    % Read the first line of the serial monitor: GyrAngle
    gyrAngleLine = readline(arduinoSerial);
    gyrAngle = str2double(gyrAngleLine);
    gyrAngle = max(-30, min(30, gyrAngle));
    disp(gyrAngleLine);

    % Update the text Angle
    set(angleText, 'String', sprintf('Current Angle: %.2f degrees', gyrAngle));
    
    % Read the second line of the serial monitor: PosDist
    posDistLine = readline(arduinoSerial);
    posDist = str2double(posDistLine);
    disp(posDistLine);

    % Calculate the control error
    error = desiredAngle - gyrAngle;

    % Calculate the time difference
    currentTime = toc(previousTime);
    previousTime = tic;

    % Calculate the derivative of the error
    errorDerivative = (error - previousError) / currentTime;
    previousError = error;
    
    % Calculate the PWM output using PD control
    pwmValue = Kp * error + Kd * errorDerivative;
    
    % Limit PWM output to the range [-199, 199]
    pwmValue = max(-100, min(100, pwmValue));
    
    % Convert PWM value to an integer for Arduino communication
    pwmInt = round(pwmValue);
    disp(pwmInt);

    % Calculate pendulum position
    xPos = posDist + pendulumLength * sin(deg2rad(gyrAngle));
    yPos = cartHeight/2 + pendulumLength * cos(deg2rad(gyrAngle)); % Inverted Position
    trailIdx = mod(trailIdx, trailLength) + 1;
    trailX(trailIdx) = xPos;
    trailY(trailIdx) = yPos;
    set(trailPlot, 'XData', trailX, 'YData', trailY);
    trailIdx = mod(trailIdx, trailLength) + 1;

    % Update the cart position
    set(cart, 'Position', [posDist - cartWidth/2, -cartHeight/2, cartWidth, cartHeight]);

    % Update the pendulum plot
    set(h, 'XData', [posDist xPos], 'YData', [0 yPos]);
    set(hx, 'XData', xPos, 'YData', yPos);
    drawnow limitrate;
    
    % Use the recorded gyrAngle to interpolate PWM value
    % result = intpolCalibration(gyrAngle, 30, 0, -30, 199, 99.5, 0);
    result = num2str(pwmInt);
    % disp(result);

    % Write the calculated result back to Serial Monitor for Arduino
    writeline(arduinoSerial, result);

    pause(0.01);

    end

catch ME
% Clean up in case of error
disp('An error occurred:');
disp(ME.message);
end

% Clean up
clear arduinoSerial; % Close and delete the serial port object













