#include <Wire.h>
#include <MPU6050.h>

// Create MPU6050 object
MPU6050 mpu;

// Calibration bias variables (raw sensor counts)
float accelBiasX = 0, accelBiasY = 0, accelBiasZ = 0;
float gyroBiasX = 0,  gyroBiasY = 0,  gyroBiasZ = 0;

// Timing variables
unsigned long previousTime = 0;
const unsigned long sampleInterval = 100; // 100ms interval = 10 Hz
const int numSamples = 100;               // Number of samples for calibration

// Buffering variables
const int MAX_LINES = 5;        // Store 4 lines before flushing
const int LINE_LENGTH = 160;    // Each line can hold 140 chars
char buffer[MAX_LINES][LINE_LENGTH];
int bufferLineCount = 0;


// Filtering parameters
const float alphaGyro = 0.9; // Low-pass filter constant for gyro
const float alphaAcc  = 0.8; // Low-pass filter constant for accelerometer

// Filtered data variables (in physical units)
float filteredAccelX, filteredAccelY, filteredAccelZ;
float filteredGyroX, filteredGyroY, filteredGyroZ;

// Flag to initialize filtered values on first sample
bool firstSample = true;


// Function to perform calibration for accelerometer and gyroscope
void calibrateSensors() {
  long sumAx = 0, sumAy = 0, sumAz = 0;
  long sumGx = 0, sumGy = 0, sumGz = 0;
  
  Serial.println("Calibrating sensors. Please hold the sensor still...");
  delay(2000); // Time to hold the sensor still
  
  int16_t ax, ay, az, gx, gy, gz;
  for (int i = 0; i < numSamples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumAx += ax;
    sumAy += ay;
    sumAz += az;
    sumGx += gx;
    sumGy += gy;
    sumGz += gz;
    delay(20); // Sample every 20ms (~2 seconds total)
  }
  
  // Compute average biases
  accelBiasX = sumAx / (float)numSamples;
  accelBiasY = sumAy / (float)numSamples;
  accelBiasZ = sumAz / (float)numSamples;
  gyroBiasX  = sumGx  / (float)numSamples;
  gyroBiasY  = sumGy  / (float)numSamples;
  gyroBiasZ  = sumGz  / (float)numSamples;
  
  Serial.println("Calibration complete!");
  Serial.print("Accel Biases: ");
  Serial.print(accelBiasX); Serial.print(", ");
  Serial.print(accelBiasY); Serial.print(", ");
  Serial.println(accelBiasZ);
  
  Serial.print("Gyro Biases: ");
  Serial.print(gyroBiasX); Serial.print(", ");
  Serial.print(gyroBiasY); Serial.print(", ");
  Serial.println(gyroBiasZ);
}

void setup() {
  // Initialize serial communication at 115200 baud
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected!");
  } else {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    while (1); // Halt execution if sensor not found
  }
  
  // Calibrate sensors (accelerometer and gyro)
  Serial.println("Calibrating sensors...");
  calibrateSensors();
  Serial.println("Calibration complete!");
  
  // Print CSV header (this will be the first line in your CSV file)
  Serial.println("time_ms,rawAccelX_g,rawAccelY_g,rawAccelZ_g,rawGyroX_dps,rawGyroY_dps,rawGyroZ_dps,filteredAccelX_g,filteredAccelY_g,filteredAccelZ_g,filteredGyroX_dps,filteredGyroY_dps,filteredGyroZ_dps");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read sensor data at defined interval
  if (currentTime - previousTime >= sampleInterval) {
    previousTime = currentTime;
    
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Convert raw data to calibrated values and then to physical units:
    float rawAccelX = (ax - accelBiasX) / 16384.0;
    float rawAccelY = (ay - accelBiasY) / 16384.0;
    float rawAccelZ = (az - accelBiasZ) / 16384.0;
    float rawGyroX  = (gx - gyroBiasX) / 131.0;
    float rawGyroY  = (gy - gyroBiasY) / 131.0;
    float rawGyroZ  = (gz - gyroBiasZ) / 131.0;

    
    // Read and convert temperature to °C using the datasheet formula:
    int16_t rawTemp = mpu.getTemperature();  // Raw temperature value
    float tempC = (rawTemp / 340.0) + 36.53;   // Converted temperature in °C

    if (firstSample) {
      filteredAccelX = rawAccelX;
      filteredAccelY = rawAccelY;
      filteredAccelZ = rawAccelZ;
      filteredGyroX  = rawGyroX;
      filteredGyroY  = rawGyroY;
      filteredGyroZ  = rawGyroZ;
      firstSample = false;
    }
    else {
      filteredAccelX = alphaAcc * filteredAccelX + (1 - alphaAcc) * rawAccelX;
      filteredAccelY = alphaAcc * filteredAccelY + (1 - alphaAcc) * rawAccelY;
      filteredAccelZ = alphaAcc * filteredAccelZ + (1 - alphaAcc) * rawAccelZ;
      filteredGyroX  = alphaGyro * filteredGyroX + (1 - alphaGyro) * rawGyroX;
      filteredGyroY  = alphaGyro * filteredGyroY + (1 - alphaGyro) * rawGyroY;
      filteredGyroZ  = alphaGyro * filteredGyroZ + (1 - alphaGyro) * rawGyroZ;
    }
    
    // Create a CSV formatted string for this data line
        // String dataLine = String(currentTime) + "," +
        //               String(rawAccelX, 6) + "," +
        //               String(rawAccelY, 6) + "," +
        //               String(rawAccelZ, 6) + "," +
        //               String(rawGyroX, 6) + "," +
        //               String(rawGyroY, 6) + "," +
        //               String(rawGyroZ, 6) + ",";
        // String datafiltered =  
        //               String(filteredAccelX, 6) + "," +
        //               String(filteredAccelY, 6) + "," +
        //               String(filteredAccelZ, 6) + "," +
        //               String(filteredGyroX, 6) + "," +
        //               String(filteredGyroY, 6) + "," +
        //               String(filteredGyroZ, 6) + "\n";

        char dataLine[160];
        char temp[16];
        int len = 0;

        len += sprintf(dataLine + len, "%lu,", currentTime);

        dtostrf(rawAccelX, 1, 6, temp); len += sprintf(dataLine + len, "%s,", temp);
        dtostrf(rawAccelY, 1, 6, temp); len += sprintf(dataLine + len, "%s,", temp);
        dtostrf(rawAccelZ, 1, 6, temp); len += sprintf(dataLine + len, "%s,", temp);

        dtostrf(rawGyroX, 1, 6, temp); len += sprintf(dataLine + len, "%s,", temp);
        dtostrf(rawGyroY, 1, 6, temp); len += sprintf(dataLine + len, "%s,", temp);
        dtostrf(rawGyroZ, 1, 6, temp); len += sprintf(dataLine + len, "%s,", temp);

        dtostrf(filteredAccelX, 1, 6, temp); len += sprintf(dataLine + len, "%s,", temp);
        dtostrf(filteredAccelY, 1, 6, temp); len += sprintf(dataLine + len, "%s,", temp);
        dtostrf(filteredAccelZ, 1, 6, temp); len += sprintf(dataLine + len, "%s,", temp);

        dtostrf(filteredGyroX, 1, 6, temp); len += sprintf(dataLine + len, "%s,", temp);
        dtostrf(filteredGyroY, 1, 6, temp); len += sprintf(dataLine + len, "%s,", temp);
        dtostrf(filteredGyroZ, 1, 6, temp); len += sprintf(dataLine + len, "%s\n", temp);

        // Now copy into circular buffer
        strncpy(buffer[bufferLineCount], dataLine, LINE_LENGTH - 1);
        buffer[bufferLineCount][LINE_LENGTH - 1] = '\0';  // make sure it's null-terminated
        bufferLineCount++;


    
    // Append data to the buffer
    // buffer += dataLine;
    // buffer += datafiltered;
    // bufferLineCount++;
    
    // When the buffer reaches MAX_LINES, flush it (simulate writing to SD)
    if (bufferLineCount >= MAX_LINES) {
      for (int i = 0; i < bufferLineCount; i++) {
        Serial.print(buffer[i]);
      }
      // Serial.println(); //only for debugging
      bufferLineCount = 0;
    }

  }
}
