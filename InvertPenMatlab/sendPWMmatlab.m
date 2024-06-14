% Define the COM port and the baud rate
comPort = 'COM5'; % Change this to your actual COM port
baudRate = 9600;  % Make sure this matches the baud rate in your Arduino code

% Initialize PWM Pin 
% a = arduino("COM5", "Uno");
% pwmPin = 'D9';
% configurePin(a, pwmPin, 'PWM');

% Create a serial port object
arduinoSerial = serialport(comPort, baudRate);

% Configure the terminator
configureTerminator(arduinoSerial, "CR/LF"); % Configure terminator if needed

% Read and store data from the serial port
try
    while true
    
    
    % % Read the first line (gyrAngle)
    gyrAngleLine = readline(arduinoSerial);
    gyrAngle = str2double(gyrAngleLine);
    disp(gyrAngleLine);
    % 
    % % Read the second line (posDist)
    % posDistLine = readline(arduinoSerial);
    % posDist = str2double(posDistLine);
    % disp(posDistLine);
    % 
    % pwmValue = readline(arduinoSerial);
    % disp(pwmValue);
    % 
    % % Process data
    % % Output a PWM value according to gyrAngle value
    result = intpolCalibration(gyrAngle, 30, 0, -30, 199, 99.5, 0);
    result = num2str(result);
    disp(result);
    % 
    % % writePWMDutyCycle(a, pwmPin, result);
    % 
    writeline(arduinoSerial, result);

    pause(0.1);
    % write(arduinoSerial, result, "double");
    end

catch ME
% Clean up in case of error
disp('An error occurred:');
disp(ME.message);
end

% Clean up
clear arduinoSerial; % Close and delete the serial port object







