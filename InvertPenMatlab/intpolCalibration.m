function result = intpolCalibration(x, x0, x1, x2, y0, y1, y2)
    % MATLAB function to perform interpolation calibration
    % Input: x, x0, x1, x2, y0, y1, y2 - scalar values
    % Output: result - interpolated value
    
    % Calculate the interpolated result
    term1 = ((x - x1) * (x - x2)) / ((x0 - x1) * (x0 - x2)) * y0;
    term2 = ((x - x0) * (x - x2)) / ((x1 - x0) * (x1 - x2)) * y1;
    term3 = ((x - x0) * (x - x1)) / ((x2 - x0) * (x2 - x1)) * y2;
    
    result = term1 + term2 + term3;
end
