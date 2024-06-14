% Define the COM port and the baud rate
comPort = 'COM5'; % COM5 used for Arduino UNO %
baudRate = 9600;  

% Open the serial port for communication with Arduino
arduinoSerial = serialport(comPort, baudRate);

% Configure the terminator
configureTerminator(arduinoSerial, "CR/LF"); 

% Read and store data from the serial port
try
    while true

    % Read the first line of the serial monitor: GyrAngle
    gyrAngleLine = readline(arduinoSerial);
    gyrAngle = str2double(gyrAngleLine);
    % disp(gyrAngleLine);
    
    % Read the second line of the serial monitor: PosDist
    % posDistLine = readline(arduinoSerial);
    % posDist = str2double(posDistLine);
    % disp(posDistLine);
    
    % Use the recorded gyrAngle to interpolate PWM value
    result = intpolCalibration(gyrAngle, 30, 0, -30, 199, 99.5, 0);
    result = num2str(result);
    % disp(result);

    % Write the calculated result back to Serial Monitor for Arduino
    writeline(arduinoSerial, result);

    pause(0.1);

    end

catch ME
% Clean up in case of error
disp('An error occurred:');
disp(ME.message);
end

% Clean up
clear arduinoSerial; % Close and delete the serial port object







