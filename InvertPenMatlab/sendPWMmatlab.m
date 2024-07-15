close all;

%% Define the COM port and the baud rate
comPort = 'COM5'; % COM5 used for Arduino UNO %
baudRate = 9600;  
tic; 

% Open the serial port for communication with Arduino
arduinoSerial = serialport(comPort, baudRate);

% Configure the terminator
configureTerminator(arduinoSerial, "CR/LF");

%% PID Variables
i = 0;
Kpg = 15;
Kig = 0.0; %5
Kdg = 5;

Kpp = 100;
Kip = 1;
Kdp = 20;

refAngle = 0;
refPosition = 0.1;

currentTime = 0;
lastTime = 0;
elapsedTime = 0;
currentTimeP = 0;
lastTimeP = 0;
elapsedTimeP = 0;

errorG = 0;
errorGPast = 0;
errorGCum = 0;
errorGDif = 0;

errorP = 0;
errorPPast = 0;
errorPCum = 0;
errorPDif = 0;

%% Set up the figure for the pendulum simulation
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

% Initial plot for the pendulum
cart = rectangle('Position', [0 -cartHeight/2 cartWidth cartHeight], 'FaceColor', [0.5 0.5 0.5]);
h = plot([0 0], [0 pendulumLength], 'r', 'LineWidth', 2); % Pendulum
hx = plot(0, pendulumLength, 'bo', 'MarkerFaceColor', 'b'); % Pendulum end point
trailIdx = 1;

%% Read and store data from the serial port
try
    while true

    %% Read the first line of the serial monitor: GyrAngle
    gyrAngleLine = readline(arduinoSerial);
    gyrAngle = str2double(gyrAngleLine);
    gyrAngle = max(-30, min(30, gyrAngle));
    % save('gyrAngle.mat', "gyrAngle");
    disp(gyrAngleLine);

    % Update the text Angle
    set(angleText, 'String', sprintf('Current Angle: %.2f degrees', gyrAngle));
    
    % Read the second line of the serial monitor: PosDist
    posDistLine = readline(arduinoSerial);
    posDist = str2double(posDistLine);
    posDist = max(-100, min(100, posDist));
    % save('posDist.mat', "posDist");
    disp(posDistLine);

    % PID Controllers

    % Angle PID
    currentTime = toc;
    elapsedTime = currentTime - lastTime;

    errorG = refAngle - gyrAngle;
    if i >= 3
        errorGCum = 0;
        i = 0;
    else
        errorGCum = errorGCum + (errorG * elapsedTime);
        i = i + 1;
    end
    errorGDif = (errorG - errorGPast) / elapsedTime;

    PIDG = Kpg * errorG + Kig * errorGCum + Kdg * errorGDif;
    disp(PIDG);

    errorGPast = errorG;
    lastTime = currentTime;

    satPIDG = max(-255, min(255, PIDG));
    disp(satPIDG);
    
    % % Position PID
    % currentTimeP = toc;
    % elapsedTimeP = currentTimeP - lastTimeP;
    % 
    % errorP = refPosition - posDist;
    % errorPCum = errorPCum + (errorP * elapsedTimeP);
    % errorPDif = (errorP - errorPPast) / elapsedTimeP;
    % 
    % PIDP = Kpp * errorP + Kip * errorPCum + Kdp * errorPDif;
    % 
    % errorPPast = errorP;
    % lastTimeP = currentTimeP;
    % 
    % controlSignal = PIDG + PIDP;
    % satControlSignal = min(255, max(-255, controlSignal));
    % disp(satControlSignal);

  
        
    %% Calculate pendulum position
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
    
    %% Use the recorded gyrAngle to interpolate PWM value
    % result = intpolCalibration(gyrAngle, 30, 0, -30, 199, 99.5, 0);
    result = num2str(satPIDG);
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













