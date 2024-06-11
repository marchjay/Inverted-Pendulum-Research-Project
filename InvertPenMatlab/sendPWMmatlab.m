% Define the COM port and the baud rate
comPort = 'COM5'; % Change this to your actual COM port
baudRate = 9600;  % Make sure this matches the baud rate in your Arduino code

% Create a serial port object
arduinoSerial = serialport(comPort, baudRate);

% Configure the terminator
configureTerminator(arduinoSerial, "CR/LF"); % Configure terminator if needed

% Read and store data from the serial port
try
    while true
    
    % Read the first line (gyrAngle)
    gyrAngleLine = readline(arduinoSerial);
    gyrAngle = str2double(gyrAngleLine);
    disp(gyrAngleLine);
    
    % Read the second line (posDist)
    posDistLine = readline(arduinoSerial);
    posDist = str2double(posDistLine);
    disp(posDistLine);

    pwmValue = readline(arduinoSerial);
    disp(pwmValue);
    
    % Process data
    % Output a PWM value according to gyrAngle value
    result = intpolCalibration(gyrAngle, 30, 0, -30, 255, 127.5, 0);
    disp(result);
    % write(arduinoSerial, result, "double");
    fprintf(arduinoSerial, '%f\n', result);
    end

catch ME
% Clean up in case of error
disp('An error occurred:');
disp(ME.message);
end

% Clean up
clear arduinoSerial; % Close and delete the serial port object







