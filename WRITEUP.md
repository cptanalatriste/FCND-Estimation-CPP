# Writeup: Building an Estimator

## Implement Estimator

### Determine the standard deviation of the measurement noise

This task required updating the parameters `MeasuredStdDev_GPSPosXY`
and `MeasuredStdDev_AccelXY` from the `06_SensorNoise.txt`.
The original values (`.2` and `.1`) do not reflect the data stored
in `Graph1.txt` and `Graph2.txt`.

To obtain the true standard deviation values, I opened each file on
LibreOffice Calc --- a spreadsheet software --- and used the `STDEV` function
to calculate the standard deviation of the data points stored
in `Graph1.txt` and `Graph2.txt`.
Then, I plugged the values I obtained (`MeasuredStdDev_GPSPosXY = 0.686946473091773` and 
`MeasuredStdDev_AccelXY = 0.496309757154396`) into `06_SensorNoise.txt`.
As seen in the Flight Evaluation section, these values satisfied the requirements.

### Implement a better rate gyro attitude integration scheme

### Implement all of the elements of the prediction step

### Implement the magnetometer update

### Implement the GPS update



## Flight Evaluation

### Step 1: Sensor Noise

![img.png](img/step_1.png)



