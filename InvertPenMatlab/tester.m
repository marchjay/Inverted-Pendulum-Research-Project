% Initialize the connection to Arduino
a = arduino('COM5', 'Uno');

% Define the pin number
pin = 'D13';

% Set the pin mode to output
configurePin(a, pin, 'DigitalOutput');

% Write a high value to the pin to turn on the LED
writeDigitalPin(a, pin, 1);

% Pause for a while to keep the LED on
pause(5);  % Keep the LED on for 5 seconds

% Turn off the LED
writeDigitalPin(a, pin, 0);







