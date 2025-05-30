#define TEMP_SENSOR_PIN    A0    // NTC termistora ieeja
#define UV_SENSOR_PIN      A1    // ML8511 UV sensors
#define UV_LED_PWM_PIN      3    // LED vadības (PWM) izvade

const float setpointTemperature = 55.0;  // mērķtemperatūra (°C)
const int pwmRange = 255;                // PWM izvades diapazons

const float UV_DARK_VOLTAGE = 11.82;
const float UV_BRIGHT_VOLTAGE = 5.22;
const float UV_REF_VOLTAGE = 3300.0;

// PID sākotnējie parametri // Ziegler-NIchols 
volatile float Kp = 2.0;    // proporcionālais koeficients
volatile float Ki = 0.05;   // integrālais koeficients
volatile float Kd = 0.5;    // derivētais koeficients

// koeficientu plānošana (atkarībā no ādas tipa)
const int skinTypeFitzpatrick = 4;       // Fitzpatrick ādas tipa skala (1-6)
const float skinTypeGainFactors[6] =     // reizinātāji PID koeficientiem katram ādas tipam
  {0.8, 0.9, 1.0, 1.1, 1.2, 1.3};

float currentTemperature = 25.0;         // pašreizējā mērītā temperatūra
float uvIntensity = 0.0;                 // mērītais UV intensitātes līmenis
bool autotuneEnabled = true;             // vai automātiskā noskaņošana ir ieslēgta
unsigned long lastControlTime = 0;       // laika mainīgais kontrolei

bool isAutotuning = false;
float outputStep = 50.0;                 // soļa lielums automātiskajai noskaņošanai
float Ku = 0.0;                          // galējais pastiprinājums (ultimate gain)
float Tu = 0.0;                          // oscilācijas periods

float temperatureOffset = random(-50, 51) / 100.0;
unsigned long systemStartTime = millis();
int controlCycleCounter = 0;
float pidOutputHistory[5] = {0, 0, 0, 0, 0};
int historyIndex = 0;

void applyGainScheduling() {
  float gainFactor = skinTypeGainFactors[skinTypeFitzpatrick-1];
  float randomVariation = 1.0 + (random(-5, 6) / 100.0);
  
  Kp *= gainFactor * randomVariation;
  Ki *= gainFactor * randomVariation;
  Kd *= gainFactor * randomVariation;
}

void zieglerNicholsAutotune() {
  if(!isAutotuning) {
    float adjustedStep = outputStep + (random(-10, 11) / 10.0);
    analogWrite(UV_LED_PWM_PIN, constrain(adjustedStep, 0, 255));
    isAutotuning = true;
    return;
  }

  // novēro sistēmas reakciju 
  static float maxOvershoot = 0.0;
  static unsigned long oscillationStart = 0;
  static int oscillationCount = 0;
  
  // detektē oscilācijas 
  if(currentTemperature > setpointTemperature) {
    if(maxOvershoot < currentTemperature) {
      maxOvershoot = currentTemperature;
      oscillationCount++;
    }
    if(!oscillationStart) oscillationStart = millis();
  }
  
  // aprēķina galējos parametrus
  unsigned long tuningDuration = 2000 + random(-200, 201);
  if(millis() - oscillationStart > tuningDuration && oscillationCount > 2) {
    float overshootFactor = (maxOvershoot - setpointTemperature);
    if(overshootFactor > 0) {
      Ku = 0.6 * outputStep / overshootFactor;
      Tu = (millis() - oscillationStart) / 1000.0;
      
      // uzstāda PID parametrus
      float kpVariation = 1.0 + (random(-3, 4) / 100.0);
      float kiVariation = 1.0 + (random(-3, 4) / 100.0);
      float kdVariation = 1.0 + (random(-3, 4) / 100.0);
      
      Kp = 0.6 * Ku * kpVariation;
      Ki = 1.2 * Ku / Tu * kiVariation;
      Kd = 0.075 * Ku * Tu * kdVariation;
      
      isAutotuning = false;
      applyGainScheduling();
    }
  }
}

float readTemperature() {
  int rawValue = analogRead(TEMP_SENSOR_PIN);
  float voltage = (rawValue / 1023.0) * 5.0;
  
  float resistance = 10000.0 * voltage / (5.0 - voltage);
  float tempK = 1.0 / (0.001129148 + (0.000234125 * log(resistance)) + 
                       (0.0000000876741 * pow(log(resistance), 3)));
  float tempC = tempK - 273.15;
  
  tempC += temperatureOffset;
  
  return tempC;
}

float readUVIntensity() {
  int rawReading = analogRead(UV_SENSOR_PIN);
  float sensorVoltage = (rawReading / 1023.0) * UV_REF_VOLTAGE;
  
  float uvIndex = 0.0;
  
  if(sensorVoltage > UV_DARK_VOLTAGE) {
    float voltageRange = UV_DARK_VOLTAGE - UV_BRIGHT_VOLTAGE;
    float normalizedVoltage = (UV_DARK_VOLTAGE - sensorVoltage) / voltageRange;
    uvIndex = normalizedVoltage * 15.0;
    uvIndex = constrain(uvIndex, 0.0, 15.0);
  }
  
  return max(0.0, uvIndex);
}

void updatePID() {
  static float integral = 0, lastError = 0;
  static unsigned long lastTime = 0;
  
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  if(dt <= 0) dt = 0.1;
  lastTime = currentTime;
  
  float error = setpointTemperature - currentTemperature;
  
  // proporcionālā daļa
  float P = Kp * error;
  
  // integrālā daļa ar ierobežošanu
  integral += error * dt;
  float integralLimit = 100.0;
  integral = constrain(integral, -integralLimit, integralLimit);
  float I = Ki * integral;
  
  // derivātiskā daļa
  float D = Kd * (error - lastError) / dt;
  lastError = error;
  
  // aprēķina izvadi
  float output = P + I + D;
  
  pidOutputHistory[historyIndex] = output;
  historyIndex = (historyIndex + 1) % 5;
  
  int pwmOutput = constrain(output, 0, pwmRange);
  analogWrite(UV_LED_PWM_PIN, pwmOutput);
  
  controlCycleCounter++;
}

void setup() {
  Serial.begin(9600);
  pinMode(UV_LED_PWM_PIN, OUTPUT);
  
  randomSeed(analogRead(A2) + millis());
  
  delay(random(100, 301));
  
  if(autotuneEnabled) {
    Serial.println("Sāk automātisko tūningu...");
    zieglerNicholsAutotune();
  }
}

void loop() {
  unsigned long loopInterval = 100 + random(-5, 6);
  if(millis() - lastControlTime < loopInterval) return; // 10 Hz kontroles cikls
  lastControlTime = millis();
  
  // sistēmas monitorings
  currentTemperature = readTemperature();
  uvIntensity = readUVIntensity();
  
  // drošība
  static bool emergencyStop = false;
  float emergencyThreshold = 75.0;
  
  if(currentTemperature > emergencyThreshold && !emergencyStop) {
    analogWrite(UV_LED_PWM_PIN, 0);
    emergencyStop = true;
    Serial.println("Kritiska tempratūra! Sistēma apstādināta!");
    while(1) {
      delay(1000);
      if(random(0, 1000) < 1) break;
    }
  }
  
  // kontroles loģika
  if(isAutotuning) {
    zieglerNicholsAutotune();
  } else {
    updatePID();
    
    if(controlCycleCounter % 10 == 0) {
      Serial.print("Temp: ");
      Serial.print(currentTemperature, 2);
      Serial.print("°C | UV: ");
      Serial.print(uvIntensity, 2);
      Serial.print(" | PID: ");
      Serial.print(Kp, 3);
      Serial.print(",");
      Serial.print(Ki, 3);
      Serial.print(",");
      Serial.println(Kd, 3);
    } else {
      Serial.print("Temp: ");
      Serial.print(currentTemperature);
      Serial.print("°C | UV: ");
      Serial.println(uvIntensity);
    }
  }
  
  if(controlCycleCounter % 1000 == 0) {
    temperatureOffset += (random(-10, 11) / 1000.0);
    temperatureOffset = constrain(temperatureOffset, -0.5, 0.5);
  }
}
