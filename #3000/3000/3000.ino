//***********************************************************************************************
//  ESP32 Mentora Study Companion Robot - Complete with All Sensors
//  Enhanced Robo Eyes + Sensor Integration for Study Companion
//
//  Hardware: ESP32 dev board, I2C OLED display, Servo motors, Multiple sensors
//  ESP32 I2C Pins: SDA = GPIO 21, SCL = GPIO 22 (default)
//
//  SENSOR PINS:
//  - BH1750 Light Sensor: I2C (SDA=21, SCL=22)
//  - MAX30102 Heart Rate: I2C (SDA=21, SCL=22)
//  - TTP223 Touch 1: GPIO 25
//  - TTP223 Touch 2: GPIO 26
//  - Tilt Switch: GPIO 27
//  - Sound Sensor: GPIO 34 (analog)
//  - Status LED: GPIO 2
//
//  API Endpoints:
//  GET /sensors - Returns all current sensor readings
//  GET /insights - Returns processed insights for app
//  GET /status - Returns robot status with sensor summary
//  POST /emotion - Set robot emotion
//
//***********************************************************************************************

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

// SENSOR LIBRARIES
#include <BH1750.h>
#include "MAX30105.h"
#include "heartRate.h"

// WiFi credentials
const char* ssid = "kkk";
const char* password = "lakitha4";

// NEW: Voice control and advanced features
#define MICROPHONE_PIN 35
#define BUZZER_PIN 32
#define PIR_SENSOR_PIN 33
#define TEMPERATURE_SENSOR_PIN 36
#define RGB_LED_PIN 4

// NEW: Study session management
struct StudySession {
  unsigned long startTime;
  unsigned long duration;
  int focusBreaks;
  int touchInteractions;
  float avgLightLevel;
  float avgHeartRate;
  bool isActive;
  String sessionId;
};

StudySession currentSession;

// Servo pin definitions
#define TILT_PIN 18
#define PAN_PIN 19

// SENSOR PIN DEFINITIONS
#define TOUCH_PIN_1 25
#define TOUCH_PIN_2 26
#define TILT_SWITCH_PIN 27
#define SOUND_SENSOR_PIN 34
#define STATUS_LED_PIN 2

// Define servo objects
Servo tiltServo;
Servo panServo;

// OLED display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Create display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Include FluxGarage library after display declaration
#include <FluxGarage_RoboEyes.h>
roboEyes roboEyes;

// SENSOR OBJECTS
BH1750 lightMeter;
MAX30105 particleSensor;

// Web server on port 80
WebServer server(80);

// Emotion state variables
String currentEmotion = "DEFAULT";
String baseEmotion = "DEFAULT";
bool hasReaction = false;
unsigned long lastCommandTime = 0;
bool wifiConnected = false;
unsigned long lastEyeUpdate = 0;
int eyeState = 0;
bool eyeOpen = true;

// Animation variables
unsigned long lastAnimationUpdate = 0;
const unsigned long animationInterval = 100;
int animationStep = 0;
bool animationActive = false;
unsigned long animationStartTime = 0;
const unsigned long animationDuration = 3000;

bool isYesNoAnimation = false;
int yesNoStep = 0;
const int yesSteps = 6;
const int noSteps = 8;
unsigned long yesNoStepDuration = 300;

bool isReactionAnimation = false;
int reactionStep = 0;
const unsigned long reactionAnimationDuration = 3000;
unsigned long reactionStepDuration = 150;

// SENSOR VARIABLES
struct SensorData {
  float lightLevel;         // Lux
  float heartRate;          // BPM
  float spO2;               // %
  bool touch1;              // Touch sensor 1
  bool touch2;              // Touch sensor 2
  bool tiltState;           // Tilt switch
  int soundLevel;           // 0-4095 analog
  bool heartRateValid;      // HR sensor status
  unsigned long timestamp;  // When data was collected
};

SensorData currentSensors;
SensorData previousSensors;

// Sensor timing variables
unsigned long lastSensorUpdate = 0;
const unsigned long sensorUpdateInterval = 1000;  // Update every second
unsigned long lastHeartRateUpdate = 0;
const unsigned long heartRateInterval = 2000;  // HR every 2 seconds

// Study insights variables
struct StudyInsights {
  String lightStatus;        // "good", "low", "bright"
  String stressLevel;        // "low", "normal", "high"
  String focusLevel;         // "focused", "distracted"
  String environmentStatus;  // "quiet", "noisy"
  String positionStatus;     // "stable", "moving"
  int interactionCount;      // Touch interactions
  String recommendation;     // AI-friendly recommendation
  float studyScore;          // 0-100 overall study condition
};

StudyInsights currentInsights;

// Interaction tracking
int touchCount1 = 0;
int touchCount2 = 0;
bool lastTouch1 = false;
bool lastTouch2 = false;
bool lastTiltState = false;
unsigned long lastTouchTime = 0;
unsigned long lastTiltTime = 0;

// Heart rate calculation
const byte RATE_ARRAY_SIZE = 4;
long rateArray[RATE_ARRAY_SIZE];
byte rateSpot = 0;
long lastBeat = 0;

// NEW: Advanced features variables
float temperatureC = 0.0;
bool motionDetected = false;
int microphoneLevel = 0;
unsigned long lastMotionTime = 0;
unsigned long lastVoiceCommandTime = 0;

// NEW: Study session tracking
bool studyModeActive = false;
unsigned long studyStartTime = 0;
int studyBreakCount = 0;
unsigned long lastBreakTime = 0;
const unsigned long BREAK_REMINDER_INTERVAL = 25 * 60 * 1000; // 25 minutes

// NEW: Voice command patterns (simple detection)
int voiceThreshold = 2000;
bool listeningMode = false;
unsigned long lastSoundDetection = 0;

// NEW: Environmental comfort tracking
struct ComfortMetrics {
  String temperatureStatus;  // "cold", "comfortable", "warm", "hot"
  String airQuality;         // "good", "moderate", "poor"
  String ergonomics;         // "good", "adjust_lighting", "take_break"
  float comfortScore;        // 0-100
};

ComfortMetrics comfortData;

// Function declarations
void initializeDisplay();
void initializeRoboEyes();
void connectToWiFi();
void checkWiFiConnection();
void setupWebServer();
void setEmotion(String emotion);
void displayEmotion();
void startYesNoAnimation(String type);
void startReactionAnimation(String emotionType);
void startTransitionAnimation();
void updateAnimations();
void updateYesNoAnimation();
void updateReactionAnimation();
void endYesNoAnimation();
void endReactionAnimation();
void displayConnectionLost();
void nodYes();
void shakeNo();
void performHappyReaction();
void performAngryReaction();
void performTiredReaction();
void performDefaultReaction();
String parseBaseEmotion(String emotion);
bool isReactionEmotion(String emotion);

// SENSOR FUNCTIONS
void initializeSensors();
void updateSensors();
void calculateInsights();
void handleTouchInteraction();
void handleTiltChange();
void updateStatusLED();
String getSensorJSON();
String getInsightsJSON();

// NEW: Advanced feature functions
void initializeAdvancedSensors();
void updateAdvancedSensors();
void handleMotionDetection();
void handleVoiceCommand();
void updateStudySession();
void checkStudyBreakReminder();
void calculateComfortMetrics();
void playTone(int frequency, int duration);
void setRGBColor(int r, int g, int b);
void performStudyBreakAnimation();
void performWelcomeAnimation();
String getStudySessionJSON();
String getComfortJSON();
void startStudySession();
void endStudySession();
void pauseStudySession();
void resumeStudySession();

void setup() {
  Serial.begin(115200);
  Serial.println("Starting ESP32 Mentora Study Companion Robot...");

  // Initialize servos first
  tiltServo.attach(TILT_PIN);
  panServo.attach(PAN_PIN);
  tiltServo.write(90);
  panServo.write(90);
  delay(500);

  // Initialize I2C and OLED
  initializeDisplay();
  initializeRoboEyes();

  // Initialize all sensors
  initializeSensors();
  
  // NEW: Initialize advanced sensors
  initializeAdvancedSensors();

  // Connect to WiFi
  connectToWiFi();

  // Setup web server routes
  setupWebServer();

  Serial.println("Mentora Robot Setup Complete!");
  displayEmotion();
}

void loop() {
  // Handle web server requests
  server.handleClient();

  // Update RoboEyes animations
  if (!isYesNoAnimation && !isReactionAnimation) {
    roboEyes.update();
  }

  // Update custom animations
  updateAnimations();

  // Update sensor readings
  updateSensors();
  
  // NEW: Update advanced sensors and features
  updateAdvancedSensors();
  
  // NEW: Update study session tracking
  updateStudySession();
  
  // NEW: Check for study break reminders
  checkStudyBreakReminder();

  // Update status LED
  updateStatusLED();

  // Check WiFi connection
  checkWiFiConnection();

  delay(10);
}

// Initialize all sensors
void initializeSensors() {
  Serial.println("Initializing sensors...");

  // Initialize pins
  pinMode(TOUCH_PIN_1, INPUT);
  pinMode(TOUCH_PIN_2, INPUT);
  pinMode(TILT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(SOUND_SENSOR_PIN, INPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);

  // Initialize BH1750 light sensor
  if (lightMeter.begin()) {
    Serial.println("BH1750 Light Sensor initialized");
  } else {
    Serial.println("Error initializing BH1750");
  }

  // Initialize MAX30102 heart rate sensor
  if (particleSensor.begin()) {
    Serial.println("MAX30102 Heart Rate Sensor initialized");
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeGreen(0);
  } else {
    Serial.println("Error initializing MAX30102");
  }

  // Initialize sensor data
  currentSensors.timestamp = millis();
  previousSensors = currentSensors;

  // Initialize insights
  currentInsights.lightStatus = "unknown";
  currentInsights.stressLevel = "normal";
  currentInsights.focusLevel = "focused";
  currentInsights.environmentStatus = "quiet";
  currentInsights.positionStatus = "stable";
  currentInsights.interactionCount = 0;
  currentInsights.recommendation = "Ready to study!";
  currentInsights.studyScore = 75.0;

  Serial.println("All sensors initialized successfully");
}

// NEW: Initialize advanced sensors and features
void initializeAdvancedSensors() {
  Serial.println("Initializing advanced features...");
  
  // Initialize additional pins
  pinMode(MICROPHONE_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PIR_SENSOR_PIN, INPUT);
  pinMode(TEMPERATURE_SENSOR_PIN, INPUT);
  pinMode(RGB_LED_PIN, OUTPUT);
  
  // Initialize study session
  currentSession.isActive = false;
  currentSession.sessionId = "";
  currentSession.startTime = 0;
  currentSession.duration = 0;
  currentSession.focusBreaks = 0;
  currentSession.touchInteractions = 0;
  currentSession.avgLightLevel = 0.0;
  currentSession.avgHeartRate = 0.0;
  
  // Initialize comfort metrics
  comfortData.temperatureStatus = "unknown";
  comfortData.airQuality = "unknown";
  comfortData.ergonomics = "good";
  comfortData.comfortScore = 75.0;
  
  // Welcome animation
  performWelcomeAnimation();
  
  Serial.println("Advanced features initialized");
}

// Update all sensor readings
void updateSensors() {
  unsigned long currentTime = millis();

  if (currentTime - lastSensorUpdate >= sensorUpdateInterval) {
    lastSensorUpdate = currentTime;
    previousSensors = currentSensors;

    // Read light sensor
    currentSensors.lightLevel = lightMeter.readLightLevel();

    // Read touch sensors
    currentSensors.touch1 = digitalRead(TOUCH_PIN_1);
    currentSensors.touch2 = digitalRead(TOUCH_PIN_2);

    // Read tilt switch
    currentSensors.tiltState = !digitalRead(TILT_SWITCH_PIN);  // Inverted for pullup

    // Read sound sensor (analog)
    currentSensors.soundLevel = analogRead(SOUND_SENSOR_PIN);

    currentSensors.timestamp = currentTime;

    // Handle touch interactions
    handleTouchInteraction();

    // Handle tilt changes
    handleTiltChange();

    // Calculate insights
    calculateInsights();

    // Debug output every 5 seconds
    static unsigned long lastDebug = 0;
    if (currentTime - lastDebug > 5000) {
      lastDebug = currentTime;
      Serial.printf("Sensors - Light: %.1f lux, Sound: %d, Touch1: %d, Touch2: %d, Tilt: %d\n",
                    currentSensors.lightLevel, currentSensors.soundLevel,
                    currentSensors.touch1, currentSensors.touch2, currentSensors.tiltState);
    }
  }

  // Update heart rate separately (slower)
  if (currentTime - lastHeartRateUpdate >= heartRateInterval) {
    lastHeartRateUpdate = currentTime;

    long irValue = particleSensor.getIR();
    currentSensors.heartRateValid = false;

    if (checkForBeat(irValue)) {
      // Calculate time between beats
      long delta = currentTime - lastBeat;
      lastBeat = currentTime;

      // Store valid reading
      if (delta > 250 && delta < 2000) {          // Valid heart rate range
        rateArray[rateSpot++] = (60000 / delta);  // Convert to BPM
        rateSpot %= RATE_ARRAY_SIZE;

        // Take average of readings
        long total = 0;
        for (byte i = 0; i < RATE_ARRAY_SIZE; i++) {
          total += rateArray[i];
        }
        currentSensors.heartRate = total / RATE_ARRAY_SIZE;
        currentSensors.heartRateValid = true;

        // Calculate SpO2 (simplified)
        currentSensors.spO2 = 97.0 + random(-2, 3);  // Placeholder calculation
      }
    }
  }
}

// Handle touch sensor interactions
void handleTouchInteraction() {
  // Touch sensor 1
  if (currentSensors.touch1 && !lastTouch1) {
    touchCount1++;
    lastTouchTime = millis();
    Serial.println("Touch sensor 1 activated! Count: " + String(touchCount1));

    // Trigger happy reaction
    if (!isYesNoAnimation && !isReactionAnimation) {
      setEmotion("HAPPY_REACTION");
    }
  }
  lastTouch1 = currentSensors.touch1;

  // Touch sensor 2
  if (currentSensors.touch2 && !lastTouch2) {
    touchCount2++;
    lastTouchTime = millis();
    Serial.println("Touch sensor 2 activated! Count: " + String(touchCount2));

    // Trigger different reaction
    if (!isYesNoAnimation && !isReactionAnimation) {
      setEmotion("DEFAULT_REACTION");
    }
  }
  lastTouch2 = currentSensors.touch2;
}

// Handle tilt sensor changes
void handleTiltChange() {
  if (currentSensors.tiltState != lastTiltState) {
    lastTiltState = currentSensors.tiltState;
    lastTiltTime = millis();

    if (currentSensors.tiltState) {
      Serial.println("Robot lifted/tilted!");

      // Trigger surprised reaction
      if (!isYesNoAnimation && !isReactionAnimation) {
        setEmotion("TIRED_REACTION");  // Using tired as "surprised"
      }
    } else {
      Serial.println("Robot placed down");
    }
  }
}

// Calculate study insights
void calculateInsights() {
  // Analyze light level
  if (currentSensors.lightLevel < 50) {
    currentInsights.lightStatus = "low";
  } else if (currentSensors.lightLevel > 1000) {
    currentInsights.lightStatus = "bright";
  } else {
    currentInsights.lightStatus = "good";
  }

  // Analyze heart rate for stress
  if (currentSensors.heartRateValid) {
    if (currentSensors.heartRate > 100) {
      currentInsights.stressLevel = "high";
    } else if (currentSensors.heartRate < 60) {
      currentInsights.stressLevel = "low";
    } else {
      currentInsights.stressLevel = "normal";
    }
  }

  // Analyze sound level for environment
  if (currentSensors.soundLevel > 2500) {
    currentInsights.environmentStatus = "noisy";
  } else if (currentSensors.soundLevel > 1800) {
    currentInsights.environmentStatus = "moderate";
  } else {
    currentInsights.environmentStatus = "quiet";
  }

  // Analyze position stability
  if (currentSensors.tiltState) {
    currentInsights.positionStatus = "moving";
  } else {
    currentInsights.positionStatus = "stable";
  }

  // Calculate focus level based on interactions and environment
  int focusScore = 100;
  if (currentInsights.environmentStatus == "noisy") focusScore -= 20;
  if (currentInsights.lightStatus == "low") focusScore -= 15;
  if (currentInsights.stressLevel == "high") focusScore -= 25;
  if (currentInsights.positionStatus == "moving") focusScore -= 30;

  if (focusScore >= 80) {
    currentInsights.focusLevel = "focused";
  } else if (focusScore >= 60) {
    currentInsights.focusLevel = "distracted";
  } else {
    currentInsights.focusLevel = "very_distracted";
  }

  currentInsights.studyScore = max(0, focusScore);
  currentInsights.interactionCount = touchCount1 + touchCount2;

  // Generate recommendation
  if (currentInsights.lightStatus == "low") {
    currentInsights.recommendation = "Consider turning on more lights for better studying";
  } else if (currentInsights.environmentStatus == "noisy") {
    currentInsights.recommendation = "Try to find a quieter place to study";
  } else if (currentInsights.stressLevel == "high") {
    currentInsights.recommendation = "Take a break and try some deep breathing";
  } else if (currentInsights.studyScore >= 80) {
    currentInsights.recommendation = "Great study environment! Keep it up!";
  } else {
    currentInsights.recommendation = "Adjust your environment for better focus";
  }
}

// Update status LED
void updateStatusLED() {
  static unsigned long lastBlink = 0;
  static bool ledState = false;

  unsigned long currentTime = millis();

  if (currentInsights.studyScore >= 80) {
    // Solid green for good conditions
    digitalWrite(STATUS_LED_PIN, HIGH);
  } else if (currentInsights.studyScore >= 60) {
    // Slow blink for moderate conditions
    if (currentTime - lastBlink > 1000) {
      lastBlink = currentTime;
      ledState = !ledState;
      digitalWrite(STATUS_LED_PIN, ledState);
    }
  } else {
    // Fast blink for poor conditions
    if (currentTime - lastBlink > 300) {
      lastBlink = currentTime;
      ledState = !ledState;
      digitalWrite(STATUS_LED_PIN, ledState);
    }
  }
}

// Get sensor data as JSON string
String getSensorJSON() {
  StaticJsonDocument<512> doc;
  doc["light_level"] = currentSensors.lightLevel;
  doc["heart_rate"] = currentSensors.heartRateValid ? currentSensors.heartRate : -1;
  doc["spo2"] = currentSensors.heartRateValid ? currentSensors.spO2 : -1;
  doc["heart_rate_valid"] = currentSensors.heartRateValid;
  doc["touch_1"] = currentSensors.touch1;
  doc["touch_2"] = currentSensors.touch2;
  doc["tilt_state"] = currentSensors.tiltState;
  doc["sound_level"] = currentSensors.soundLevel;
  doc["touch_count_1"] = touchCount1;
  doc["touch_count_2"] = touchCount2;
  doc["timestamp"] = currentSensors.timestamp;

  String result;
  serializeJson(doc, result);
  return result;
}

// NEW: Smart Focus Mode - Advanced Distraction Detection & Auto-Response
struct SmartFocusMode {
  bool isEnabled;
  bool isActive;
  int distractionLevel;        // 0-100 current distraction score
  int distractionsToday;       // Count of distractions detected
  unsigned long lastDistraction;
  String distractionType;      // "phone", "movement", "noise", "time_waste"
  bool autoResponseEnabled;    // Automatically respond to distractions
  int focusStreakMinutes;      // Current uninterrupted focus time
  int longestFocusStreak;      // Best focus streak today
  float focusEfficiency;       // Focus time / total time ratio
  String currentFocusGoal;     // "deep_focus", "light_focus", "creative"
  bool emergencyMode;          // Ultra-strict focus mode
};

SmartFocusMode smartFocus;

// Distraction detection thresholds
struct DistractionThresholds {
  int phoneDetectionSensitivity;  // Touch sensor rapid tapping = phone usage
  int movementThreshold;          // Excessive movement detection
  int soundSpikeThreshold;        // Sudden noise increases
  int timeWasteThreshold;         // Long periods without productivity
  unsigned long focusWindowMs;   // Time window for distraction analysis
};

DistractionThresholds thresholds;

// NEW: Initialize Smart Focus Mode
void initializeSmartFocus() {
  smartFocus.isEnabled = false;
  smartFocus.isActive = false;
  smartFocus.distractionLevel = 0;
  smartFocus.distractionsToday = 0;
  smartFocus.lastDistraction = 0;
  smartFocus.distractionType = "none";
  smartFocus.autoResponseEnabled = true;
  smartFocus.focusStreakMinutes = 0;
  smartFocus.longestFocusStreak = 0;
  smartFocus.focusEfficiency = 100.0;
  smartFocus.currentFocusGoal = "light_focus";
  smartFocus.emergencyMode = false;

  // Set default thresholds
  thresholds.phoneDetectionSensitivity = 5;  // 5 rapid touches in window
  thresholds.movementThreshold = 3;          // 3 motion detections in 5 min
  thresholds.soundSpikeThreshold = 1000;     // Sound level increase
  thresholds.timeWasteThreshold = 10;        // 10 minutes low productivity
  thresholds.focusWindowMs = 5 * 60 * 1000; // 5 minute analysis window
  
  Serial.println("Smart Focus Mode initialized");
}

// NEW: Update Smart Focus Mode
void updateSmartFocus() {
  if (!smartFocus.isEnabled) return;
  
  unsigned long currentTime = millis();
  static unsigned long lastFocusUpdate = 0;
  static unsigned long focusStartTime = 0;
  static int rapidTouchCount = 0;
  static unsigned long lastRapidTouch = 0;
  static int recentMovements = 0;
  static unsigned long movementWindow[10];
  static int movementIndex = 0;
  static int lastSoundLevel = 0;
  
  // Update every 30 seconds
  if (currentTime - lastFocusUpdate >= 30000) {
    lastFocusUpdate = currentTime;
    
    // Calculate current focus streak
    if (smartFocus.isActive) {
      if (focusStartTime == 0) focusStartTime = currentTime;
      smartFocus.focusStreakMinutes = (currentTime - focusStartTime) / 60000;
    }
    
    // Reset rapid touch counter every minute
    if (currentTime - lastRapidTouch > 60000) {
      rapidTouchCount = 0;
    }
    
    // Detect phone usage pattern (rapid touching)
    if (currentSensors.touch1 || currentSensors.touch2) {
      if (currentTime - lastRapidTouch < 2000) { // Within 2 seconds
        rapidTouchCount++;
        if (rapidTouchCount >= thresholds.phoneDetectionSensitivity) {
          detectDistraction("phone", 70);
          rapidTouchCount = 0; // Reset after detection
        }
      }
      lastRapidTouch = currentTime;
    }
    
    // Detect excessive movement
    if (motionDetected) {
      movementWindow[movementIndex] = currentTime;
      movementIndex = (movementIndex + 1) % 10;
      
      // Count movements in last 5 minutes
      recentMovements = 0;
      for (int i = 0; i < 10; i++) {
        if (currentTime - movementWindow[i] < thresholds.focusWindowMs) {
          recentMovements++;
        }
      }
      
      if (recentMovements >= thresholds.movementThreshold) {
        detectDistraction("movement", 40);
        // Clear old movements
        for (int i = 0; i < 10; i++) movementWindow[i] = 0;
      }
    }
    
    // Detect noise spikes
    if (lastSoundLevel > 0 && 
        currentSensors.soundLevel - lastSoundLevel > thresholds.soundSpikeThreshold) {
      detectDistraction("noise", 30);
    }
    lastSoundLevel = currentSensors.soundLevel;
    
    // Detect time wasting (low productivity for extended period)
    static float recentProductivity[6] = {0}; // Last 6 readings (3 minutes)
    static int prodIndex = 0;
    
    recentProductivity[prodIndex] = analytics.productivityScore;
    prodIndex = (prodIndex + 1) % 6;
    
    float avgRecentProductivity = 0;
    for (int i = 0; i < 6; i++) {
      avgRecentProductivity += recentProductivity[i];
    }
    avgRecentProductivity /= 6;
    
    if (avgRecentProductivity < 40 && smartFocus.isActive) {
      detectDistraction("time_waste", 50);
    }
    
    // Calculate focus efficiency
    unsigned long totalActiveTime = smartFocus.isActive ? (currentTime - focusStartTime) : 0;
    unsigned long focusTime = totalActiveTime - (smartFocus.distractionsToday * 2 * 60 * 1000); // Subtract 2 min per distraction
    
    if (totalActiveTime > 0) {
      smartFocus.focusEfficiency = (float)focusTime / totalActiveTime * 100.0;
      smartFocus.focusEfficiency = max(0.0f, min(100.0f, smartFocus.focusEfficiency));
    }
    
    // Update longest streak
    if (smartFocus.focusStreakMinutes > smartFocus.longestFocusStreak) {
      smartFocus.longestFocusStreak = smartFocus.focusStreakMinutes;
    }
  }
}

// NEW: Detect and respond to distractions
void detectDistraction(String type, int severity) {
  unsigned long currentTime = millis();
  
  // Avoid duplicate detections within 2 minutes
  if (currentTime - smartFocus.lastDistraction < 120000) return;
  
  smartFocus.lastDistraction = currentTime;
  smartFocus.distractionsToday++;
  smartFocus.distractionType = type;
  smartFocus.distractionLevel = severity;
  
  Serial.println("🚨 Distraction detected: " + type + " (Level: " + String(severity) + ")");
  
  if (smartFocus.autoResponseEnabled) {
    respondToDistraction(type, severity);
  }
  
  // Reset focus streak on significant distraction
  if (severity > 50) {
    smartFocus.focusStreakMinutes = 0;
  }
}

// NEW: Respond to different types of distractions
void respondToDistraction(String type, int severity) {
  if (type == "phone") {
    Serial.println("📱 Phone usage detected - Gentle reminder");
    setEmotion("TIRED"); // Disappointed look
    
    // Gentle reminder tone
    playTone(400, 500);
    delay(200);
    playTone(300, 500);
    
    // Flash amber warning
    for (int i = 0; i < 3; i++) {
      setRGBColor(255, 128, 0);
      delay(300);
      setRGBColor(0, 0, 0);
      delay(200);
    }
    
  } else if (type == "movement") {
    Serial.println("🏃 Excessive movement - Suggesting refocus");
    setEmotion("DEFAULT_REACTION");
    
    // Calming tone sequence
    playTone(523, 300); // C
    delay(100);
    playTone(440, 400); // A
    
    // Breathing light effect
    for (int brightness = 0; brightness <= 255; brightness += 15) {
      setRGBColor(0, brightness, 255);
      delay(50);
    }
    for (int brightness = 255; brightness >= 0; brightness -= 15) {
      setRGBColor(0, brightness, 255);
      delay(50);
    }
    
  } else if (type == "noise") {
    Serial.println("🔊 Noise spike detected - Environmental alert");
    setEmotion("ANGRY"); // Alert expression
    
    // Alert tone
    playTone(800, 200);
    delay(100);
    playTone(1000, 200);
    
    // Flash red
    setRGBColor(255, 0, 0);
    delay(1000);
    setRGBColor(0, 0, 0);
    
  } else if (type == "time_waste") {
    Serial.println("⏰ Low productivity detected - Motivation boost");
    setEmotion("HAPPY_REACTION"); // Encouraging look
    
    // Motivational melody
    int motivationTune[] = {523, 587, 659, 784}; // C-D-E-G
    for (int i = 0; i < 4; i++) {
      playTone(motivationTune[i], 200);
      delay(50);
    }
    
    // Green encouragement flash
    for (int i = 0; i < 5; i++) {
      setRGBColor(0, 255, 0);
      delay(200);
      setRGBColor(0, 0, 0);
      delay(100);
    }
  }
  
  // Emergency mode activation for severe/repeated distractions
  if (severity > 70 || smartFocus.distractionsToday > 5) {
    activateEmergencyFocus();
  }
}

// NEW: Activate emergency focus mode
void activateEmergencyFocus() {
  if (smartFocus.emergencyMode) return; // Already active
  
  smartFocus.emergencyMode = true;
  Serial.println("🚨 EMERGENCY FOCUS MODE ACTIVATED!");
  
  // Dramatic activation sequence
  setEmotion("ANGRY");
  
  // Urgent attention sound
  for (int i = 0; i < 6; i++) {
    playTone(1200, 100);
    delay(50);
    playTone(800, 100);
    delay(50);
  }
  
  // Red alert flash
  for (int i = 0; i < 10; i++) {
    setRGBColor(255, 0, 0);
    delay(100);
    setRGBColor(0, 0, 0);
    delay(50);
  }
  
  // Lower all thresholds for stricter detection
  thresholds.phoneDetectionSensitivity = 2;
  thresholds.movementThreshold = 1;
  thresholds.soundSpikeThreshold = 500;
  
  Serial.println("⚠️ Ultra-strict mode: Lower distraction tolerance activated");
}

// NEW: Start Smart Focus session
void startSmartFocus(String focusGoal = "deep_focus") {
  smartFocus.isEnabled = true;
  smartFocus.isActive = true;
  smartFocus.currentFocusGoal = focusGoal;
  smartFocus.distractionLevel = 0;
  smartFocus.focusStreakMinutes = 0;
  smartFocus.emergencyMode = false;
  
  // Reset thresholds to normal
  thresholds.phoneDetectionSensitivity = 5;
  thresholds.movementThreshold = 3;
  thresholds.soundSpikeThreshold = 1000;
  
  Serial.println("🎯 Smart Focus Mode: " + focusGoal + " activated");
  
  // Adjust settings based on focus goal
  if (focusGoal == "deep_focus") {
    // Strict settings for deep work
    thresholds.phoneDetectionSensitivity = 3;
    thresholds.movementThreshold = 2;
    voiceThreshold += 200; // Less sensitive to voice
    
  } else if (focusGoal == "creative") {
    // More lenient for creative work
    thresholds.phoneDetectionSensitivity = 7;
    thresholds.movementThreshold = 5;
    voiceThreshold -= 100; // More sensitive (creativity needs expression)
    
  } // light_focus uses default settings
  
  // Activation animation
  setEmotion("HAPPY");
  playTone(659, 300); // E
  delay(100);
  playTone(784, 300); // G
  delay(100);
  playTone(988, 500); // B
  
  // Focus mode color: Blue
  setRGBColor(0, 100, 255);
}

// NEW: End Smart Focus session
void endSmartFocus() {
  if (!smartFocus.isActive) return;
  
  smartFocus.isActive = false;
  smartFocus.emergencyMode = false;
  
  // Session summary
  Serial.println("📊 Focus Session Complete!");
  Serial.println("Duration: " + String(smartFocus.focusStreakMinutes) + " minutes");
  Serial.println("Distractions: " + String(smartFocus.distractionsToday));
  Serial.println("Efficiency: " + String(smartFocus.focusEfficiency) + "%");
  Serial.println("Longest Streak: " + String(smartFocus.longestFocusStreak) + " minutes");
  
  // Award bonus points for good focus
  if (smartFocus.focusEfficiency > 90) {
    motivation.totalPoints += 20;
    Serial.println("🏆 Focus Master bonus: +20 points!");
  } else if (smartFocus.focusEfficiency > 75) {
    motivation.totalPoints += 10;
    Serial.println("🎯 Good Focus bonus: +10 points!");
  }
  
  // Completion celebration
  setEmotion("HAPPY_REACTION");
  
  // Success melody
  int completionMelody[] = {523, 659, 784, 1047}; // C-E-G-C
  for (int i = 0; i < 4; i++) {
    playTone(completionMelody[i], 300);
    delay(100);
  }
  
  // Reset to normal color
  setRGBColor(0, 255, 0);
}

// NEW: Get Smart Focus data as JSON
String getSmartFocusJSON() {
  StaticJsonDocument<768> doc;
  
  doc["enabled"] = smartFocus.isEnabled;
  doc["active"] = smartFocus.isActive;
  doc["distraction_level"] = smartFocus.distractionLevel;
  doc["distractions_today"] = smartFocus.distractionsToday;
  doc["last_distraction_type"] = smartFocus.distractionType;
  doc["auto_response"] = smartFocus.autoResponseEnabled;
  doc["focus_streak_minutes"] = smartFocus.focusStreakMinutes;
  doc["longest_streak_today"] = smartFocus.longestFocusStreak;
  doc["focus_efficiency"] = smartFocus.focusEfficiency;
  doc["current_goal"] = smartFocus.currentFocusGoal;
  doc["emergency_mode"] = smartFocus.emergencyMode;
  
  // Threshold settings
  doc["thresholds"]["phone_sensitivity"] = thresholds.phoneDetectionSensitivity;
  doc["thresholds"]["movement_limit"] = thresholds.movementThreshold;
  doc["thresholds"]["sound_spike"] = thresholds.soundSpikeThreshold;
  
  String result;
  serializeJson(doc, result);
  return result;
}

// NEW: Update advanced sensors
void updateAdvancedSensors() {
  static unsigned long lastAdvancedUpdate = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastAdvancedUpdate >= 500) { // Update every 500ms
    lastAdvancedUpdate = currentTime;
    
    // Read temperature sensor (analog)
    int tempReading = analogRead(TEMPERATURE_SENSOR_PIN);
    temperatureC = (tempReading * 3.3 / 4095.0 - 0.5) * 100.0; // Convert to Celsius
    
    // Read microphone level
    microphoneLevel = analogRead(MICROPHONE_PIN);
    
    // Read PIR motion sensor
    bool currentMotion = digitalRead(PIR_SENSOR_PIN);
    if (currentMotion && !motionDetected) {
      motionDetected = true;
      lastMotionTime = currentTime;
      handleMotionDetection();
    } else if (!currentMotion) {
      motionDetected = false;
    }
    
    // Check for voice commands
    if (microphoneLevel > voiceThreshold) {
      if (currentTime - lastSoundDetection > 1000) { // Debounce
        lastSoundDetection = currentTime;
        handleVoiceCommand();
      }
    }
    
    // Calculate comfort metrics
    calculateComfortMetrics();
    
    // Update RGB status LED based on overall conditions
    updateRGBStatusLED();
  }
}

// NEW: Handle motion detection
void handleMotionDetection() {
  Serial.println("Motion detected!");
  
  if (!studyModeActive) {
    // User returned, suggest starting study session
    setEmotion("HAPPY_REACTION");
    playTone(800, 200);
    delay(100);
    playTone(1000, 200);
  }
}

// NEW: Simple voice command handling
void handleVoiceCommand() {
  Serial.println("Voice detected - Level: " + String(microphoneLevel));
  
  // Simple pattern: Loud sound = command
  if (microphoneLevel > voiceThreshold + 500) {
    listeningMode = true;
    lastVoiceCommandTime = millis();
    
    // Visual feedback
    setRGBColor(0, 0, 255); // Blue for listening
    setEmotion("DEFAULT_REACTION");
    
    // Audio feedback
    playTone(1200, 100);
    
    Serial.println("Listening mode activated");
  }
}

// NEW: Update study session tracking
void updateStudySession() {
  if (currentSession.isActive) {
    unsigned long currentTime = millis();
    currentSession.duration = currentTime - currentSession.startTime;
    
    // Update averages
    static int readingCount = 0;
    static float lightSum = 0;
    static float hrSum = 0;
    static int validHrCount = 0;
    
    readingCount++;
    lightSum += currentSensors.lightLevel;
    
    if (currentSensors.heartRateValid) {
      hrSum += currentSensors.heartRate;
      validHrCount++;
    }
    
    currentSession.avgLightLevel = lightSum / readingCount;
    if (validHrCount > 0) {
      currentSession.avgHeartRate = hrSum / validHrCount;
    }
    
    currentSession.touchInteractions = touchCount1 + touchCount2;
  }
}

// NEW: Check for study break reminders
void checkStudyBreakReminder() {
  if (currentSession.isActive) {
    unsigned long currentTime = millis();
    
    if (currentTime - lastBreakTime >= BREAK_REMINDER_INTERVAL) {
      lastBreakTime = currentTime;
      studyBreakCount++;
      
      Serial.println("Break time reminder!");
      performStudyBreakAnimation();
      
      // Play break reminder tone
      for (int i = 0; i < 3; i++) {
        playTone(660, 300);
        delay(100);
      }
    }
  }
}

// NEW: Calculate comfort metrics
void calculateComfortMetrics() {
  // Temperature comfort
  if (temperatureC < 18) {
    comfortData.temperatureStatus = "cold";
  } else if (temperatureC >= 18 && temperatureC <= 24) {
    comfortData.temperatureStatus = "comfortable";
  } else if (temperatureC > 24 && temperatureC <= 28) {
    comfortData.temperatureStatus = "warm";
  } else {
    comfortData.temperatureStatus = "hot";
  }
  
  // Air quality based on sound and motion patterns
  if (currentInsights.environmentStatus == "quiet" && microphoneLevel < 1500) {
    comfortData.airQuality = "good";
  } else if (currentInsights.environmentStatus == "moderate") {
    comfortData.airQuality = "moderate";
  } else {
    comfortData.airQuality = "poor";
  }
  
  // Ergonomics based on light and study duration
  if (currentInsights.lightStatus == "good" && currentSession.duration < 3600000) { // 1 hour
    comfortData.ergonomics = "good";
  } else if (currentInsights.lightStatus == "low" || currentInsights.lightStatus == "bright") {
    comfortData.ergonomics = "adjust_lighting";
  } else if (currentSession.duration > 3600000) { // Over 1 hour
    comfortData.ergonomics = "take_break";
  }
  
  // Calculate overall comfort score
  int tempScore = (comfortData.temperatureStatus == "comfortable") ? 30 : 15;
  int airScore = (comfortData.airQuality == "good") ? 25 : (comfortData.airQuality == "moderate") ? 15 : 5;
  int ergoScore = (comfortData.ergonomics == "good") ? 25 : 10;
  int studyScore = (int)(currentInsights.studyScore * 0.2); // 20% weight
  
  comfortData.comfortScore = tempScore + airScore + ergoScore + studyScore;
}

// NEW: Play tone on buzzer
void playTone(int frequency, int duration) {
  tone(BUZZER_PIN, frequency, duration);
}

// NEW: Set RGB LED color (simple PWM simulation)
void setRGBColor(int r, int g, int b) {
  // Simple brightness control on single pin
  int brightness = (r + g + b) / 3;
  analogWrite(RGB_LED_PIN, brightness);
}

// NEW: Update RGB status LED
void updateRGBStatusLED() {
  if (comfortData.comfortScore >= 80) {
    setRGBColor(0, 255, 0); // Green - excellent
  } else if (comfortData.comfortScore >= 60) {
    setRGBColor(255, 255, 0); // Yellow - good
  } else if (comfortData.comfortScore >= 40) {
    setRGBColor(255, 128, 0); // Orange - fair
  } else {
    setRGBColor(255, 0, 0); // Red - poor
  }
}

// NEW: Study break animation
void performStudyBreakAnimation() {
  Serial.println("Performing study break animation");
  
  setEmotion("TIRED_REACTION");
  
  // Eye movement pattern for break reminder
  for (int i = 0; i < 5; i++) {
    roboEyes.setPosition(S);
    delay(300);
    roboEyes.setPosition(DEFAULT);
    delay(300);
  }
}

// NEW: Welcome animation
void performWelcomeAnimation() {
  Serial.println("Welcome animation starting");
  
  // Friendly greeting sequence
  setRGBColor(0, 255, 0);
  playTone(523, 200); // C
  delay(100);
  playTone(659, 200); // E
  delay(100);
  playTone(784, 300); // G
  
  setEmotion("HAPPY_REACTION");
}

// NEW: Get study session data as JSON
String getStudySessionJSON() {
  StaticJsonDocument<512> doc;
  doc["active"] = currentSession.isActive;
  doc["session_id"] = currentSession.sessionId;
  doc["start_time"] = currentSession.startTime;
  doc["duration"] = currentSession.duration;
  doc["duration_minutes"] = currentSession.duration / 60000;
  doc["focus_breaks"] = studyBreakCount;
  doc["interactions"] = currentSession.touchInteractions;
  doc["avg_light_level"] = currentSession.avgLightLevel;
  doc["avg_heart_rate"] = currentSession.avgHeartRate;
  doc["motion_detected"] = motionDetected;
  doc["last_motion"] = lastMotionTime;
  
  String result;
  serializeJson(doc, result);
  return result;
}

// NEW: Get comfort data as JSON
String getComfortJSON() {
  StaticJsonDocument<512> doc;
  doc["temperature_c"] = temperatureC;
  doc["temperature_status"] = comfortData.temperatureStatus;
  doc["air_quality"] = comfortData.airQuality;
  doc["ergonomics"] = comfortData.ergonomics;
  doc["comfort_score"] = comfortData.comfortScore;
  doc["microphone_level"] = microphoneLevel;
  doc["motion_detected"] = motionDetected;
  doc["listening_mode"] = listeningMode;
  
  String result;
  serializeJson(doc, result);
  return result;
}

// NEW: Start study session
void startStudySession() {
  currentSession.isActive = true;
  currentSession.startTime = millis();
  currentSession.sessionId = "session_" + String(millis());
  currentSession.duration = 0;
  currentSession.focusBreaks = 0;
  currentSession.touchInteractions = 0;
  studyBreakCount = 0;
  lastBreakTime = millis();
  studyModeActive = true;
  
  Serial.println("Study session started: " + currentSession.sessionId);
  setEmotion("HAPPY");
  playTone(880, 500);
}

// NEW: End study session
void endStudySession() {
  if (currentSession.isActive) {
    currentSession.isActive = false;
    studyModeActive = false;
    
    Serial.println("Study session ended. Duration: " + String(currentSession.duration / 60000) + " minutes");
    setEmotion("DEFAULT");
    
    // Completion celebration
    for (int i = 0; i < 3; i++) {
      playTone(659, 200);
      delay(100);
      playTone(880, 200);
      delay(100);
    }
  }
}

// Get insights as JSON string
String getInsightsJSON() {
  StaticJsonDocument<512> doc;
  doc["light_status"] = currentInsights.lightStatus;
  doc["stress_level"] = currentInsights.stressLevel;
  doc["focus_level"] = currentInsights.focusLevel;
  doc["environment_status"] = currentInsights.environmentStatus;
  doc["position_status"] = currentInsights.positionStatus;
  doc["interaction_count"] = currentInsights.interactionCount;
  doc["recommendation"] = currentInsights.recommendation;
  doc["study_score"] = currentInsights.studyScore;

  String result;
  serializeJson(doc, result);
  return result;
}

void initializeDisplay() {
  Wire.begin(21, 22);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    while (1)
      ;
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 20);
  display.println("MENTORA");
  display.setCursor(30, 40);
  display.println("ROBOT");
  display.display();
  delay(2000);

  Serial.println("OLED display initialized");
}

void initializeRoboEyes() {
  roboEyes.begin(SCREEN_WIDTH, SCREEN_HEIGHT, 60);
  roboEyes.setAutoblinker(ON, 3, 2);
  roboEyes.setIdleMode(ON, 2, 2);
  Serial.println("FluxGarage RoboEyes library initialized");
}

void connectToWiFi() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Connecting to WiFi...");
  display.display();

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(1000);
    Serial.print(".");
    attempts++;
    display.print(".");
    display.display();
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println();
    Serial.print("Connected! IP address: ");
    Serial.println(WiFi.localIP());

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi Connected!");
    display.setCursor(0, 15);
    display.println("IP Address:");
    display.setCursor(0, 25);
    display.println(WiFi.localIP().toString());
    display.setCursor(0, 45);
    display.println("Mentora Ready!");
    display.display();
    delay(3000);

  } else {
    wifiConnected = false;
    Serial.println();
    Serial.println("WiFi connection failed!");

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi Failed!");
    display.setCursor(0, 15);
    display.println("Check credentials");
    display.display();
    delay(5000);
  }
}

void checkWiFiConnection() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 10000) {
    lastCheck = millis();

    if (WiFi.status() != WL_CONNECTED && wifiConnected) {
      wifiConnected = false;
      Serial.println("WiFi connection lost!");
      displayConnectionLost();
    } else if (WiFi.status() == WL_CONNECTED && !wifiConnected) {
      wifiConnected = true;
      Serial.println("WiFi reconnected!");
    }
  }
}

void setupWebServer() {
  server.enableCORS(true);

  // Enhanced /status endpoint with sensor data
  server.on("/status", HTTP_GET, []() {
    StaticJsonDocument<1024> doc;
    doc["emotion"] = currentEmotion;
    doc["base_emotion"] = baseEmotion;
    doc["has_reaction"] = hasReaction;
    doc["wifi_connected"] = wifiConnected;
    doc["ip_address"] = WiFi.localIP().toString();
    doc["uptime"] = millis();
    doc["last_command"] = lastCommandTime;
    doc["signal_strength"] = WiFi.RSSI();
    doc["animation_active"] = animationActive;
    doc["yes_no_active"] = isYesNoAnimation;
    doc["reaction_active"] = isReactionAnimation;

    // Add sensor summary
    doc["light_level"] = currentSensors.lightLevel;
    doc["heart_rate_valid"] = currentSensors.heartRateValid;
    doc["study_score"] = currentInsights.studyScore;
    doc["interactions"] = touchCount1 + touchCount2;

    String response;
    serializeJson(doc, response);

    server.sendHeader("Content-Type", "application/json");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", response);
  });

  // Sensors endpoint
  server.on("/sensors", HTTP_GET, []() {
    server.sendHeader("Content-Type", "application/json");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", getSensorJSON());
    Serial.println("Sensor data requested");
  });

  // Insights endpoint
  server.on("/insights", HTTP_GET, []() {
    server.sendHeader("Content-Type", "application/json");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", getInsightsJSON());
    Serial.println("Insights data requested");
  });

  // NEW: Study session endpoint
  server.on("/study", HTTP_GET, []() {
    server.sendHeader("Content-Type", "application/json");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", getStudySessionJSON());
    Serial.println("Study session data requested");
  });

  // NEW: Comfort metrics endpoint
  server.on("/comfort", HTTP_GET, []() {
    server.sendHeader("Content-Type", "application/json");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", getComfortJSON());
    Serial.println("Comfort data requested");
  });

  // NEW: Study session control endpoint
  server.on("/study/control", HTTP_POST, []() {
    if (!server.hasArg("plain")) {
      server.send(400, "application/json", "{\"error\":\"No JSON body provided\"}");
      return;
    }

    String body = server.arg("plain");
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, body);

    if (error) {
      server.send(400, "application/json", "{\"error\":\"Invalid JSON format\"}");
      return;
    }

    if (!doc.containsKey("action")) {
      server.send(400, "application/json", "{\"error\":\"Missing 'action' field\"}");
      return;
    }

    String action = doc["action"];
    action.toLowerCase();

    StaticJsonDocument<256> response;

    if (action == "start") {
      startStudySession();
      response["success"] = true;
      response["message"] = "Study session started";
      response["session_id"] = currentSession.sessionId;
    } else if (action == "end") {
      endStudySession();
      response["success"] = true;
      response["message"] = "Study session ended";
    } else {
      server.send(400, "application/json", "{\"error\":\"Invalid action. Use 'start' or 'end'\"}");
      return;
    }

    String responseStr;
    serializeJson(response, responseStr);
    server.send(200, "application/json", responseStr);
  });

  // NEW: Smart Focus Mode endpoints
  server.on("/focus", HTTP_GET, []() {
    server.sendHeader("Content-Type", "application/json");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", getSmartFocusJSON());
    Serial.println("Smart Focus data requested");
  });

  server.on("/focus/control", HTTP_POST, []() {
    if (!server.hasArg("plain")) {
      server.send(400, "application/json", "{\"error\":\"No JSON body provided\"}");
      return;
    }

    String body = server.arg("plain");
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, body);

    if (error) {
      server.send(400, "application/json", "{\"error\":\"Invalid JSON format\"}");
      return;
    }

    String action = doc["action"];
    StaticJsonDocument<256> response;

    if (action == "start") {
      String focusGoal = doc.containsKey("goal") ? doc["goal"].as<String>() : "light_focus";
      startSmartFocus(focusGoal);
      response["success"] = true;
      response["message"] = "Smart Focus started";
      response["goal"] = focusGoal;
    } else if (action == "end") {
      endSmartFocus();
      response["success"] = true;
      response["message"] = "Smart Focus ended";
      response["efficiency"] = smartFocus.focusEfficiency;
    } else if (action == "emergency") {
      activateEmergencyFocus();
      response["success"] = true;
      response["message"] = "Emergency Focus activated";
    } else {
      server.send(400, "application/json", "{\"error\":\"Invalid action. Use 'start', 'end', or 'emergency'\"}");
      return;
    }

    String responseStr;
    serializeJson(response, responseStr);
    server.send(200, "application/json", responseStr);
  });

  // NEW: Analytics endpoint
  server.on("/analytics", HTTP_GET, []() {
    server.sendHeader("Content-Type", "application/json");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", getAnalyticsJSON());
    Serial.println("Analytics data requested");
  });

  // NEW: Pomodoro control endpoint
  server.on("/pomodoro", HTTP_POST, []() {
    if (!server.hasArg("plain")) {
      server.send(400, "application/json", "{\"error\":\"No JSON body provided\"}");
      return;
    }

    String body = server.arg("plain");
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, body);

    if (error) {
      server.send(400, "application/json", "{\"error\":\"Invalid JSON format\"}");
      return;
    }

    String action = doc["action"];
    StaticJsonDocument<256> response;

    if (action == "start") {
      startPomodoro();
      response["success"] = true;
      response["message"] = "Pomodoro started";
    } else if (action == "stop") {
      pomodoro.isActive = false;
      response["success"] = true;
      response["message"] = "Pomodoro stopped";
    } else {
      server.send(400, "application/json", "{\"error\":\"Invalid action\"}");
      return;
    }

    String responseStr;
    serializeJson(response, responseStr);
    server.send(200, "application/json", responseStr);
  });

  // Emotion endpoint
  server.on("/emotion", HTTP_POST, []() {
    if (!server.hasArg("plain")) {
      server.send(400, "application/json", "{\"error\":\"No JSON body provided\"}");
      return;
    }

    String body = server.arg("plain");
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, body);

    if (error) {
      server.send(400, "application/json", "{\"error\":\"Invalid JSON format\"}");
      return;
    }

    if (!doc.containsKey("emotion")) {
      server.send(400, "application/json", "{\"error\":\"Missing 'emotion' field in JSON\"}");
      return;
    }

    String newEmotion = doc["emotion"];
    newEmotion.toUpperCase();

    if (newEmotion == "HAPPY" || newEmotion == "ANGRY" || newEmotion == "TIRED" || newEmotion == "DEFAULT" || 
        newEmotion == "HAPPY_REACTION" || newEmotion == "ANGRY_REACTION" || newEmotion == "TIRED_REACTION" || 
        newEmotion == "DEFAULT_REACTION" || newEmotion == "YES" || newEmotion == "NO") {

      setEmotion(newEmotion);
      lastCommandTime = millis();

      StaticJsonDocument<256> response;
      response["success"] = true;
      response["emotion"] = currentEmotion;
      response["message"] = "Emotion set successfully";

      String responseStr;
      serializeJson(response, responseStr);

      server.send(200, "application/json", responseStr);

    } else {
      server.send(400, "application/json", "{\"error\":\"Invalid emotion\"}");
    }
  });

  server.on("/emotion", HTTP_OPTIONS, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "POST, GET, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    server.send(200);
  });

    // Enhanced web interface
  server.on("/", HTTP_GET, []() {
    String html = "<!DOCTYPE html><html><head><title>Mentora Robot Control</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:Arial;margin:20px;background:#f0f8ff;}";
    html += ".container{max-width:1200px;margin:0 auto;}";
    html += ".status{background:white;padding:15px;margin:10px 0;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1);}";
    html += ".sensor-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:15px;margin:20px 0;}";
    html += ".sensor-card{background:white;padding:15px;border-radius:8px;box-shadow:0 2px 8px rgba(0,0,0,0.1);}";
    html += ".sensor-value{font-size:1.5em;font-weight:bold;color:#007acc;}";
    html += "button{padding:10px 15px;margin:5px;border:none;border-radius:5px;cursor:pointer;}";
    html += ".good{background:#4CAF50;color:white;} .warning{background:#ff9800;color:white;} .error{background:#f44336;color:white;}";
    html += ".study-controls{background:#e8f4fd;padding:20px;border-radius:10px;margin:20px 0;}";
    html += ".session-active{background:#d4edda;border:2px solid #28a745;}";
    html += "</style></head><body>";
    html += "<div class='container'>";
    html += "<h1>🤖 Mentora Study Companion</h1>";

    // Study session status
    html += "<div class='study-controls" + (currentSession.isActive ? " session-active" : "") + "'>";
    html += "<h3>📚 Study Session</h3>";
    if (currentSession.isActive) {
      html += "<p><strong>Status:</strong> Active ✅</p>";
      html += "<p><strong>Duration:</strong> " + String(currentSession.duration / 60000) + " minutes</p>";
      html += "<p><strong>Breaks:</strong> " + String(studyBreakCount) + "</p>";
      html += "<button class='error' onclick=\"controlSession('end')\">⏹️ End Session</button>";
    } else {
      html += "<p><strong>Status:</strong> Inactive ⏸️</p>";
      html += "<button class='good' onclick=\"controlSession('start')\">▶️ Start Session</button>";
    }
    html += "</div>";

    // Robot status section
    html += "<div class='status'>";
    html += "<h3>Robot Status</h3>";
    html += "<p><strong>Current Emotion:</strong> " + currentEmotion + "</p>";
    html += "<p><strong>Study Score:</strong> " + String(currentInsights.studyScore) + "/100</p>";
    html += "<p><strong>Comfort Score:</strong> " + String(comfortData.comfortScore) + "/100</p>";
    html += "<p><strong>Recommendation:</strong> " + currentInsights.recommendation + "</p>";
    html += "</div>";

    // Enhanced sensor grid
    html += "<div class='sensor-grid'>";
    
    // Smart Focus Status Card
    html += "<div class='sensor-card" + (smartFocus.isActive ? " session-active" : "") + "'>";
    html += "<h4>🎯 Smart Focus</h4>";
    if (smartFocus.isActive) {
      html += "<div class='sensor-value'>" + String(smartFocus.focusStreakMinutes) + " min</div>";
      html += "<p>Goal: " + smartFocus.currentFocusGoal + "</p>";
      html += "<p>Efficiency: " + String(smartFocus.focusEfficiency, 1) + "%</p>";
      html += "<p>Distractions: " + String(smartFocus.distractionsToday) + "</p>";
      if (smartFocus.emergencyMode) {
        html += "<p style='color:red;font-weight:bold;'>🚨 EMERGENCY MODE</p>";
      }
    } else {
      html += "<div class='sensor-value'>Inactive</div>";
      html += "<p>Ready to focus</p>";
    }
    html += "</div>";
    
    html += "<div class='sensor-card'>";
    html += "<h4>💡 Light Level</h4>";
    html += "<div class='sensor-value'>" + String(currentSensors.lightLevel) + " lux</div>";
    html += "<p>Status: " + currentInsights.lightStatus + "</p>";
    html += "</div>";

    html += "<div class='sensor-card'>";
    html += "<h4>🔊 Sound Level</h4>";
    html += "<div class='sensor-value'>" + String(currentSensors.soundLevel) + "</div>";
    html += "<p>Environment: " + currentInsights.environmentStatus + "</p>";
    html += "</div>";

    html += "<div class='sensor-card'>";
    html += "<h4>❤️ Heart Rate</h4>";
    if (currentSensors.heartRateValid) {
      html += "<div class='sensor-value'>" + String(currentSensors.heartRate) + " bpm</div>";
      html += "<p>Stress: " + currentInsights.stressLevel + "</p>";
    } else {
      html += "<div class='sensor-value'>Invalid</div>";
    }
    html += "</div>";

    html += "<div class='sensor-card'>";
    html += "<h4>👋 Interactions</h4>";
    html += "<div class='sensor-value'>" + String(touchCount1 + touchCount2) + "</div>";
    html += "<p>Touch1: " + String(touchCount1) + " | Touch2: " + String(touchCount2) + "</p>";
    html += "</div>";
    
    // Advanced sensors
    html += "<div class='sensor-card'>";
    html += "<h4>🌡️ Temperature</h4>";
    html += "<div class='sensor-value'>" + String(temperatureC, 1) + "°C</div>";
    html += "<p>Status: " + comfortData.temperatureStatus + "</p>";
    html += "</div>";
    
    html += "<div class='sensor-card'>";
    html += "<h4>🎤 Voice Level</h4>";
    html += "<div class='sensor-value'>" + String(microphoneLevel) + "</div>";
    html += "<p>Listening: " + (listeningMode ? "ON" : "OFF") + "</p>";
    html += "</div>";
    
    html += "<div class='sensor-card'>";
    html += "<h4>🏃 Motion</h4>";
    html += "<div class='sensor-value'>" + (motionDetected ? "YES" : "NO") + "</div>";
    html += "<p>Last: " + String((millis() - lastMotionTime) / 1000) + "s ago</p>";
    html += "</div>";
    
    html += "<div class='sensor-card'>";
    html += "<h4>🏠 Comfort</h4>";
    html += "<div class='sensor-value'>" + String(comfortData.comfortScore, 0) + "%</div>";
    html += "<p>Ergonomics: " + comfortData.ergonomics + "</p>";
    html += "</div>";
    
    html += "</div>"; // End sensor grid

    // Smart Focus Controls
    html += "<div class='study-controls'>";
    html += "<h3>🎯 Smart Focus Controls</h3>";
    if (smartFocus.isActive) {
      html += "<button class='error' onclick=\"controlFocus('end')\">⏹️ End Focus</button>";
      html += "<button class='warning' onclick=\"controlFocus('emergency')\">🚨 Emergency Mode</button>";
    } else {
      html += "<button class='good' onclick=\"controlFocus('start', 'light_focus')\">🎯 Light Focus</button>";
      html += "<button class='good' onclick=\"controlFocus('start', 'deep_focus')\">🧠 Deep Focus</button>";
      html += "<button class='good' onclick=\"controlFocus('start', 'creative')\">🎨 Creative Mode</button>";
    }
    html += "</div>";

    // Control buttons
    html += "<div class='status'>";
    html += "<h3>🎭 Emotion Control</h3>";
    html += "<button class='good' onclick=\"setEmotion('HAPPY')\">😊 HAPPY</button>";
    html += "<button class='error' onclick=\"setEmotion('ANGRY')\">😠 ANGRY</button>";
    html += "<button class='warning' onclick=\"setEmotion('TIRED')\">😴 TIRED</button>";
    html += "<button onclick=\"setEmotion('DEFAULT')\">😐 DEFAULT</button>";
    html += "<br><br>";
    html += "<button class='good' onclick=\"setEmotion('YES')\">✅ YES</button>";
    html += "<button class='error' onclick=\"setEmotion('NO')\">❌ NO</button>";
    html += "</div>";

    html += "</div>";

    html += "<script>";
    html += "function setEmotion(emotion) {";
    html += "  fetch('/emotion', {";
    html += "    method: 'POST',";
    html += "    headers: {'Content-Type': 'application/json'},";
    html += "    body: JSON.stringify({emotion: emotion})";
    html += "  }).then(() => setTimeout(() => location.reload(), 1000));";
    html += "}";
    html += "function controlSession(action) {";
    html += "  fetch('/study/control', {";
    html += "    method: 'POST',";
    html += "    headers: {'Content-Type': 'application/json'},";
    html += "    body: JSON.stringify({action: action})";
    html += "  }).then(() => setTimeout(() => location.reload(), 1000));";
    html += "}";
    html += "function controlFocus(action, goal) {";
    html += "  const body = goal ? {action: action, goal: goal} : {action: action};";
    html += "  fetch('/focus/control', {";
    html += "    method: 'POST',";
    html += "    headers: {'Content-Type': 'application/json'},";
    html += "    body: JSON.stringify(body)";
    html += "  }).then(() => setTimeout(() => location.reload(), 1000));";
    html += "}";
    html += "setInterval(() => location.reload(), 15000);";
    html += "</script></body></html>";

    server.send(200, "text/html", html);
  }); : "NO") + "</div>";
    html += "<p>Last: " + String((millis() - lastMotionTime) / 1000) + "s ago</p>";
    html += "</div>";
    
    html += "<div class='sensor-card'>";
    html += "<h4>🏠 Comfort</h4>";
    html += "<div class='sensor-value'>" + String(comfortData.comfortScore, 0) + "%</div>";
    html += "<p>Ergonomics: " + comfortData.ergonomics + "</p>";
    html += "</div>";
    
    html += "</div>"; // End sensor grid

    // Control buttons
    html += "<div class='status'>";
    html += "<h3>🎭 Emotion Control</h3>";
    html += "<button class='good' onclick=\"setEmotion('HAPPY')\">😊 HAPPY</button>";
    html += "<button class='error' onclick=\"setEmotion('ANGRY')\">😠 ANGRY</button>";
    html += "<button class='warning' onclick=\"setEmotion('TIRED')\">😴 TIRED</button>";
    html += "<button onclick=\"setEmotion('DEFAULT')\">😐 DEFAULT</button>";
    html += "<br><br>";
    html += "<button class='good' onclick=\"setEmotion('YES')\">✅ YES</button>";
    html += "<button class='error' onclick=\"setEmotion('NO')\">❌ NO</button>";
    html += "</div>";

    html += "</div>";

    html += "<script>";
    html += "function setEmotion(emotion) {";
    html += "  fetch('/emotion', {";
    html += "    method: 'POST',";
    html += "    headers: {'Content-Type': 'application/json'},";
    html += "    body: JSON.stringify({emotion: emotion})";
    html += "  }).then(() => setTimeout(() => location.reload(), 1000));";
    html += "}";
    html += "function controlSession(action) {";
    html += "  fetch('/study/control', {";
    html += "    method: 'POST',";
    html += "    headers: {'Content-Type': 'application/json'},";
    html += "    body: JSON.stringify({action: action})";
    html += "  }).then(() => setTimeout(() => location.reload(), 1000));";
    html += "}";
    html += "setInterval(() => location.reload(), 15000);";
    html += "</script></body></html>";

    server.send(200, "text/html", html);
  });0,0,0.1);}";
    html += ".sensor-value{font-size:1.5em;font-weight:bold;color:#007acc;}";
    html += "button{padding:10px 15px;margin:5px;border:none;border-radius:5px;cursor:pointer;}";
    html += ".good{background:#4CAF50;color:white;} .warning{background:#ff9800;color:white;} .error{background:#f44336;color:white;}";
    html += "</style></head><body>";
    html += "<div class='container'>";
    html += "<h1>🤖 Mentora Study Companion</h1>";

    // Status section
    html += "<div class='status'>";
    html += "<h3>Robot Status</h3>";
    html += "<p><strong>Current Emotion:</strong> " + currentEmotion + "</p>";
    html += "<p><strong>Study Score:</strong> " + String(currentInsights.studyScore) + "/100</p>";
    html += "<p><strong>Recommendation:</strong> " + currentInsights.recommendation + "</p>";
    html += "</div>";

    // Sensor grid
    html += "<div class='sensor-grid'>";
    
    html += "<div class='sensor-card'>";
    html += "<h4>💡 Light Level</h4>";
    html += "<div class='sensor-value'>" + String(currentSensors.lightLevel) + " lux</div>";
    html += "<p>Status: " + currentInsights.lightStatus + "</p>";
    html += "</div>";

    html += "<div class='sensor-card'>";
    html += "<h4>🔊 Sound Level</h4>";
    html += "<div class='sensor-value'>" + String(currentSensors.soundLevel) + "</div>";
    html += "<p>Environment: " + currentInsights.environmentStatus + "</p>";
    html += "</div>";

    html += "<div class='sensor-card'>";
    html += "<h4>❤️ Heart Rate</h4>";
    if (currentSensors.heartRateValid) {
      html += "<div class='sensor-value'>" + String(currentSensors.heartRate) + " bpm</div>";
    } else {
      html += "<div class='sensor-value'>Invalid</div>";
    }
    html += "</div>";

    html += "<div class='sensor-card'>";
    html += "<h4>👋 Interactions</h4>";
    html += "<div class='sensor-value'>" + String(touchCount1 + touchCount2) + "</div>";
    html += "<p>Touch1: " + String(touchCount1) + " | Touch2: " + String(touchCount2) + "</p>";
    html += "</div>";
    
    html += "</div>"; // End sensor grid

    // Control buttons
    html += "<div class='status'>";
    html += "<h3>🎭 Emotion Control</h3>";
    html += "<button class='good' onclick=\"setEmotion('HAPPY')\">😊 HAPPY</button>";
    html += "<button class='error' onclick=\"setEmotion('ANGRY')\">😠 ANGRY</button>";
    html += "<button class='warning' onclick=\"setEmotion('TIRED')\">😴 TIRED</button>";
    html += "<button onclick=\"setEmotion('DEFAULT')\">😐 DEFAULT</button>";
    html += "<br><br>";
    html += "<button class='good' onclick=\"setEmotion('YES')\">✅ YES</button>";
    html += "<button class='error' onclick=\"setEmotion('NO')\">❌ NO</button>";
    html += "</div>";

    html += "</div>";

    html += "<script>";
    html += "function setEmotion(emotion) {";
    html += "  fetch('/emotion', {";
    html += "    method: 'POST',";
    html += "    headers: {'Content-Type': 'application/json'},";
    html += "    body: JSON.stringify({emotion: emotion})";
    html += "  }).then(() => setTimeout(() => location.reload(), 1000));";
    html += "}";
    html += "setInterval(() => location.reload(), 10000);";
    html += "</script></body></html>";

    server.send(200, "text/html", html);
  });

  server.onNotFound([]() {
    server.send(404, "application/json", "{\"error\":\"Endpoint not found\"}");
  });

  server.begin();
  Serial.println("Enhanced web server started with Smart Focus Mode");
  Serial.println("Access at: http://" + WiFi.localIP().toString());
  Serial.println("🎯 Smart Focus API: /focus (GET), /focus/control (POST)");
}

// NEW: Smart Focus emergency alert system
void triggerFocusAlert(String alertType) {
  if (!smartFocus.isActive) return;
  
  Serial.println("🚨 FOCUS ALERT: " + alertType);
  
  // Visual alert - rapid emotion changes
  setEmotion("ANGRY");
  delay(500);
  setEmotion("DEFAULT");
  delay(300);
  setEmotion("TIRED");
  delay(500);
  setEmotion("DEFAULT");
  
  // Audio alert pattern
  if (alertType == "SEVERE_DISTRACTION") {
    // Urgent pattern
    for (int i = 0; i < 8; i++) {
      playTone(1500, 100);
      delay(50);
      playTone(1000, 100);
      delay(50);
    }
  } else if (alertType == "FOCUS_STREAK_BROKEN") {
    // Disappointed tone
    playTone(440, 500);
    delay(200);
    playTone(330, 800);
  } else if (alertType == "PRODUCTIVITY_DROP") {
    // Warning chime
    playTone(660, 200);
    delay(100);
    playTone(523, 300);
    delay(100);
    playTone(440, 400);
  }
  
  // RGB flash sequence
  for (int cycle = 0; cycle < 5; cycle++) {
    setRGBColor(255, 0, 0);  // Red
    delay(150);
    setRGBColor(255, 165, 0); // Orange
    delay(150);
    setRGBColor(255, 255, 0); // Yellow
    delay(150);
    setRGBColor(0, 0, 0);     // Off
    delay(100);
  }
  
  // Return to focus mode color
  setRGBColor(0, 100, 255);
}

// NEW: Adaptive learning for distraction patterns
void learnFromDistractions() {
  static unsigned long distractionTimes[20];
  static String distractionTypes[20];
  static int distractionIndex = 0;
  static bool arrayFull = false;
  
  if (smartFocus.distractionsToday > 0) {
    // Store distraction data
    distractionTimes[distractionIndex] = millis();
    distractionTypes[distractionIndex] = smartFocus.distractionType;
    distractionIndex = (distractionIndex + 1) % 20;
    
    if (distractionIndex == 0) arrayFull = true;
    
    // Analyze patterns after collecting enough data
    if (arrayFull || distractionIndex > 10) {
      int phoneCount = 0, movementCount = 0, noiseCount = 0;
      unsigned long now = millis();
      
      int dataPoints = arrayFull ? 20 : distractionIndex;
      for (int i = 0; i < dataPoints; i++) {
        // Only consider recent distractions (last 2 hours)
        if (now - distractionTimes[i] < 2 * 60 * 60 * 1000) {
          if (distractionTypes[i] == "phone") phoneCount++;
          else if (distractionTypes[i] == "movement") movementCount++;
          else if (distractionTypes[i] == "noise") noiseCount++;
        }
      }
      
      // Adapt thresholds based on patterns
      if (phoneCount > 5) {
        // User has phone addiction issues - stricter
        thresholds.phoneDetectionSensitivity = max(2, thresholds.phoneDetectionSensitivity - 1);
        Serial.println("📱 Adapted: Stricter phone detection");
      }
      
      if (movementCount > 7) {
        // User is restless - be more lenient
        thresholds.movementThreshold = min(6, thresholds.movementThreshold + 1);
        Serial.println("🏃 Adapted: More lenient movement detection");
      }
      
      if (noiseCount > 6) {
        // Noisy environment - adjust expectation
        thresholds.soundSpikeThreshold += 200;
        Serial.println("🔊 Adapted: Higher noise tolerance");
      }
    }
  }
}

// NEW: Focus session quality report
String generateFocusReport() {
  String report = "📊 FOCUS SESSION REPORT\\n";
  report += "========================\\n";
  
  // Basic stats
  report += "Duration: " + String(smartFocus.focusStreakMinutes) + " minutes\\n";
  report += "Efficiency: " + String(smartFocus.focusEfficiency, 1) + "%\\n";
  report += "Distractions: " + String(smartFocus.distractionsToday) + "\\n";
  report += "Longest Streak: " + String(smartFocus.longestFocusStreak) + " minutes\\n\\n";
  
  // Quality assessment
  if (smartFocus.focusEfficiency >= 95) {
    report += "🏆 EXCEPTIONAL FOCUS! Outstanding self-control.\\n";
  } else if (smartFocus.focusEfficiency >= 85) {
    report += "⭐ EXCELLENT FOCUS! Great concentration.\\n";
  } else if (smartFocus.focusEfficiency >= 70) {
    report += "👍 GOOD FOCUS! Room for improvement.\\n";
  } else if (smartFocus.focusEfficiency >= 50) {
    report += "⚠️ FAIR FOCUS. Consider distraction elimination.\\n";
  } else {
    report += "❌ POOR FOCUS. Implement stricter measures.\\n";
  }
  
  // Improvement suggestions
  report += "\\nSUGGESTIONS:\\n";
  if (smartFocus.distractionsToday > 8) {
    report += "• Use emergency mode for better discipline\\n";
  }
  if (smartFocus.longestFocusStreak < 25) {
    report += "• Try Pomodoro technique (25-min focused blocks)\\n";
  }
  if (smartFocus.focusEfficiency < 70) {
    report += "• Remove distracting items from workspace\\n";
    report += "• Use website blockers during focus time\\n";
  }
  
  return report;
}

// NEW: Integration with other robot systems
void integrateSmartFocusWithSystems() {
  // If Smart Focus is active, modify other system behaviors
  if (smartFocus.isActive) {
    
    // Modify Pomodoro integration
    if (pomodoro.isActive && !pomodoro.isBreak) {
      // During Pomodoro work time, be extra strict
      if (smartFocus.currentFocusGoal != "deep_focus") {
        thresholds.phoneDetectionSensitivity = max(2, thresholds.phoneDetectionSensitivity);
        thresholds.movementThreshold = max(2, thresholds.movementThreshold);
      }
    }
    
    // Modify motivation system
    if (smartFocus.focusEfficiency > 80) {
      // Bonus points for maintaining focus
      motivation.totalPoints += 1; // Per update cycle
    }
    
    // Modify study insights
    if (smartFocus.distractionLevel > 60) {
      // Override study score if distractions are high
      currentInsights.studyScore = min(currentInsights.studyScore, 50.0f);
      currentInsights.recommendation = "🎯 High distractions detected. Activate emergency focus mode.";
    }
    
    // Modify comfort system
    if (smartFocus.emergencyMode) {
      // In emergency mode, comfort is secondary
      comfortData.ergonomics = "focus_mode";
      // Dim RGB to reduce visual distractions
      setRGBColor(0, 50, 100); // Dim blue
    }
    
    // Modify emotional responses
    static unsigned long lastFocusEmotion = 0;
    unsigned long now = millis();
    
    if (now - lastFocusEmotion > 5 * 60 * 1000 && smartFocus.focusEfficiency > 85) { // Every 5 min if doing well
      lastFocusEmotion = now;
      setEmotion("HAPPY"); // Encouraging look
      delay(2000);
      setEmotion("DEFAULT");
    }
  }
}

// NEW: Smart Focus statistics for long-term tracking
struct FocusStatistics {
  unsigned long totalFocusTime;       // All-time focused time
  int totalSessions;                  // Number of focus sessions
  float averageEfficiency;            // Average focus efficiency
  int bestStreakMinutes;              // Longest ever focus streak  
  int totalDistractions;              // All-time distraction count
  String mostCommonDistraction;       // Phone, movement, noise, etc.
  int emergencyModeActivations;       // Times emergency mode was used
  float improvementRate;              // Week-over-week improvement %
};

FocusStatistics focusStats;

// Initialize focus statistics
void initializeFocusStats() {
  focusStats.totalFocusTime = 0;
  focusStats.totalSessions = 0;
  focusStats.averageEfficiency = 0.0;
  focusStats.bestStreakMinutes = 0;
  focusStats.totalDistractions = 0;
  focusStats.mostCommonDistraction = "none";
  focusStats.emergencyModeActivations = 0;
  focusStats.improvementRate = 0.0;
}

// Update focus statistics when session ends
void updateFocusStats() {
  if (!smartFocus.isActive) {
    focusStats.totalSessions++;
    focusStats.totalFocusTime += smartFocus.focusStreakMinutes * 60 * 1000;
    focusStats.totalDistractions += smartFocus.distractionsToday;
    
    if (smartFocus.longestFocusStreak > focusStats.bestStreakMinutes) {
      focusStats.bestStreakMinutes = smartFocus.longestFocusStreak;
    }
    
    if (smartFocus.emergencyMode) {
      focusStats.emergencyModeActivations++;
    }
    
    // Update average efficiency
    focusStats.averageEfficiency = (focusStats.averageEfficiency * (focusStats.totalSessions - 1) + 
                                   smartFocus.focusEfficiency) / focusStats.totalSessions;
    
    Serial.println("📈 Focus stats updated - Total sessions: " + String(focusStats.totalSessions));
  }
}
}

String parseBaseEmotion(String emotion) {
  if (emotion.endsWith("_REACTION")) {
    return emotion.substring(0, emotion.indexOf("_REACTION"));
  }
  return emotion;
}

bool isReactionEmotion(String emotion) {
  return emotion.endsWith("_REACTION");
}

void setEmotion(String emotion) {
  currentEmotion = emotion;
  baseEmotion = parseBaseEmotion(emotion);
  hasReaction = isReactionEmotion(emotion);

  Serial.println("Setting emotion to: " + emotion);

  if (emotion == "YES" || emotion == "NO") {
    startYesNoAnimation(emotion);
    if (emotion == "YES") {
      nodYes();
    } else {
      shakeNo();
    }
  } else if (hasReaction) {
    startReactionAnimation(baseEmotion);
  } else {
    startTransitionAnimation();
  }

  displayEmotion();
}

void displayEmotion() {
  if (isYesNoAnimation || isReactionAnimation) {
    return;
  }

  roboEyes.setCyclops(ON);

  if (baseEmotion == "HAPPY") {
    roboEyes.setMood(HAPPY);
    roboEyes.setCuriosity(ON);
    roboEyes.setHFlicker(OFF, 0);
    roboEyes.setVFlicker(OFF, 0);
    roboEyes.setAutoblinker(ON, 3, 2);
    roboEyes.setIdleMode(ON, 2, 2);

  } else if (baseEmotion == "ANGRY") {
    roboEyes.setMood(ANGRY);
    roboEyes.setCuriosity(OFF);
    roboEyes.setHFlicker(ON, 1);
    roboEyes.setVFlicker(OFF, 0);
    roboEyes.setAutoblinker(ON, 4, 1);
    roboEyes.setIdleMode(OFF);

  } else if (baseEmotion == "TIRED") {
    roboEyes.setMood(TIRED);
    roboEyes.setCuriosity(OFF);
    roboEyes.setHFlicker(OFF, 0);
    roboEyes.setVFlicker(ON, 1);
    roboEyes.setAutoblinker(ON, 2, 1);
    roboEyes.setIdleMode(ON, 4, 2);

  } else {
    roboEyes.setMood(DEFAULT);
    roboEyes.setCuriosity(ON);
    roboEyes.setHFlicker(OFF, 0);
    roboEyes.setVFlicker(OFF, 0);
    roboEyes.setAutoblinker(ON, 3, 2);
    roboEyes.setIdleMode(ON, 2, 2);
  }
}

void startYesNoAnimation(String type) {
  Serial.println("Starting " + type + " animation");

  roboEyes.setAutoblinker(OFF, 0, 0);
  roboEyes.setIdleMode(OFF, 0, 0);
  roboEyes.setHFlicker(OFF, 0);
  roboEyes.setVFlicker(OFF, 0);

  roboEyes.setMood(DEFAULT);
  roboEyes.setPosition(DEFAULT);

  isYesNoAnimation = true;
  animationActive = true;
  animationStartTime = millis();
  yesNoStep = 0;

  if (type == "YES") {
    yesNoStepDuration = 300;
  } else {
    yesNoStepDuration = 200;
  }

  lastAnimationUpdate = millis();
}

void startReactionAnimation(String emotionType) {
  Serial.println("Starting " + emotionType + " reaction animation");

  roboEyes.setAutoblinker(OFF, 0, 0);
  roboEyes.setIdleMode(OFF, 0, 0);
  roboEyes.setHFlicker(OFF, 0);
  roboEyes.setVFlicker(OFF, 0);

  if (emotionType == "HAPPY") {
    roboEyes.setMood(HAPPY);
  } else if (emotionType == "ANGRY") {
    roboEyes.setMood(ANGRY);
  } else if (emotionType == "TIRED") {
    roboEyes.setMood(TIRED);
  } else {
    roboEyes.setMood(DEFAULT);
  }

  roboEyes.setPosition(DEFAULT);

  isReactionAnimation = true;
  animationActive = true;
  animationStartTime = millis();
  reactionStep = 0;
  lastAnimationUpdate = millis();
}

void startTransitionAnimation() {
  isYesNoAnimation = false;
  isReactionAnimation = false;
  animationActive = true;
  animationStartTime = millis();
  animationStep = 0;

  if (baseEmotion == "HAPPY") {
    roboEyes.anim_laugh();
  } else if (baseEmotion == "ANGRY") {
    roboEyes.anim_confused();
  } else if (baseEmotion == "TIRED") {
    roboEyes.blink();
  }
}

void updateAnimations() {
  unsigned long currentTime = millis();

  if (!animationActive) return;

  if (isYesNoAnimation) {
    updateYesNoAnimation();
    return;
  }

  if (isReactionAnimation) {
    updateReactionAnimation();
    return;
  }

  if (currentTime - animationStartTime >= animationDuration) {
    animationActive = false;
    roboEyes.setPosition(DEFAULT);
    return;
  }

  if (currentTime - lastAnimationUpdate >= animationInterval) {
    lastAnimationUpdate = currentTime;
    animationStep++;

    if (baseEmotion == "TIRED") {
      if (animationStep % 10 == 0) {
        roboEyes.setPosition(S);
      } else if (animationStep % 5 == 0) {
        roboEyes.setPosition(DEFAULT);
      }
    } else if (baseEmotion == "DEFAULT") {
      switch (animationStep % 8) {
        case 0: roboEyes.setPosition(N); break;
        case 1: roboEyes.setPosition(NE); break;
        case 2: roboEyes.setPosition(E); break;
        case 3: roboEyes.setPosition(SE); break;
        case 4: roboEyes.setPosition(S); break;
        case 5: roboEyes.setPosition(SW); break;
        case 6: roboEyes.setPosition(W); break;
        case 7: roboEyes.setPosition(NW); break;
      }
    }
  }
}

void updateYesNoAnimation() {
  unsigned long currentTime = millis();

  if (currentTime - lastAnimationUpdate >= yesNoStepDuration) {
    lastAnimationUpdate = currentTime;

    if (currentEmotion == "YES") {
      switch (yesNoStep % 4) {
        case 0: roboEyes.setPosition(DEFAULT); break;
        case 1: roboEyes.setPosition(N); break;
        case 2: roboEyes.setPosition(DEFAULT); break;
        case 3: roboEyes.setPosition(S); break;
      }

      yesNoStep++;
      if (yesNoStep >= (yesSteps * 2)) {
        endYesNoAnimation();
      }

    } else if (currentEmotion == "NO") {
      switch (yesNoStep % 4) {
        case 0: roboEyes.setPosition(DEFAULT); break;
        case 1: roboEyes.setPosition(W); break;
        case 2: roboEyes.setPosition(DEFAULT); break;
        case 3: roboEyes.setPosition(E); break;
      }

      yesNoStep++;
      if (yesNoStep >= (noSteps * 2)) {
        endYesNoAnimation();
      }
    }

    roboEyes.drawEyes();
  }
}

void updateReactionAnimation() {
  unsigned long currentTime = millis();

  if (currentTime - animationStartTime >= reactionAnimationDuration) {
    endReactionAnimation();
    return;
  }

  if (currentTime - lastAnimationUpdate >= reactionStepDuration) {
    lastAnimationUpdate = currentTime;
    reactionStep++;

    if (baseEmotion == "HAPPY") {
      performHappyReaction();
    } else if (baseEmotion == "ANGRY") {
      performAngryReaction();
    } else if (baseEmotion == "TIRED") {
      performTiredReaction();
    } else {
      performDefaultReaction();
    }

    roboEyes.drawEyes();
  }
}

void performHappyReaction() {
  switch (reactionStep % 6) {
    case 0:
      roboEyes.setPosition(NE);
      tiltServo.write(120);
      panServo.write(60);
      break;
    case 1:
      roboEyes.setPosition(NW);
      tiltServo.write(60);
      panServo.write(120);
      break;
    case 2:
      roboEyes.setPosition(SE);
      tiltServo.write(110);
      panServo.write(70);
      break;
    case 3:
      roboEyes.setPosition(SW);
      tiltServo.write(70);
      panServo.write(110);
      break;
    case 4:
      roboEyes.setPosition(N);
      roboEyes.blink();
      tiltServo.write(100);
      panServo.write(90);
      break;
    case 5:
      roboEyes.setPosition(DEFAULT);
      roboEyes.anim_laugh();
      tiltServo.write(90);
      panServo.write(90);
      break;
  }
}

void performAngryReaction() {
  switch (reactionStep % 5) {
    case 0:
      roboEyes.setPosition(W);
      tiltServo.write(70);
      panServo.write(50);
      break;
    case 1:
      roboEyes.setPosition(E);
      tiltServo.write(70);
      panServo.write(130);
      break;
    case 2:
      roboEyes.setPosition(N);
      tiltServo.write(120);
      panServo.write(90);
      break;
    case 3:
      roboEyes.setPosition(DEFAULT);
      roboEyes.anim_confused();
      tiltServo.write(80);
      panServo.write(90);
      break;
    case 4:
      roboEyes.blink();
      tiltServo.write(90);
      panServo.write(90);
      break;
  }
}

void performTiredReaction() {
  switch (reactionStep % 4) {
    case 0:
      roboEyes.setPosition(S);
      tiltServo.write(60);
      panServo.write(90);
      break;
    case 1:
      roboEyes.setPosition(SW);
      tiltServo.write(60);
      panServo.write(110);
      break;
    case 2:
      roboEyes.setPosition(SE);
      tiltServo.write(60);
      panServo.write(70);
      break;
    case 3:
      roboEyes.setPosition(DEFAULT);
      roboEyes.blink();
      tiltServo.write(90);
      panServo.write(90);
      break;
  }
}

void performDefaultReaction() {
  switch (reactionStep % 10) {
    case 0:
      roboEyes.setPosition(N);
      tiltServo.write(120);
      panServo.write(80);
      break;
    case 1:
      roboEyes.setPosition(NE);
      tiltServo.write(130);
      panServo.write(90);
      break;
    case 2:
      roboEyes.setPosition(E);
      tiltServo.write(110);
      panServo.write(60);
      break;
    case 3:
      roboEyes.setPosition(SE);
      tiltServo.write(100);
      panServo.write(70);
      break;
    case 4:
      roboEyes.setPosition(S);
      tiltServo.write(140);
      panServo.write(85);
      break;
    case 5:
      roboEyes.setPosition(SW);
      tiltServo.write(120);
      panServo.write(100);
      break;
    case 6:
      roboEyes.setPosition(W);
      tiltServo.write(100);
      panServo.write(120);
      break;
    case 7:
      roboEyes.setPosition(NW);
      tiltServo.write(130);
      panServo.write(95);
      break;
    case 8:
      roboEyes.setPosition(DEFAULT);
      roboEyes.blink();
      tiltServo.write(90);
      panServo.write(90);
      break;
    case 9:
      roboEyes.setPosition(N);
      tiltServo.write(110);
      panServo.write(100);
      break;
  }
}

void endYesNoAnimation() {
  Serial.println("Ending YES/NO animation");

  isYesNoAnimation = false;
  animationActive = false;
  yesNoStep = 0;

  roboEyes.setPosition(DEFAULT);

  currentEmotion = "DEFAULT";
  baseEmotion = "DEFAULT";
  hasReaction = false;
  displayEmotion();
}

void endReactionAnimation() {
  Serial.println("Ending reaction animation for: " + baseEmotion);

  isReactionAnimation = false;
  animationActive = false;
  reactionStep = 0;

  tiltServo.write(90);
  panServo.write(90);
  roboEyes.setPosition(DEFAULT);

  currentEmotion = baseEmotion;
  hasReaction = false;
  displayEmotion();
}

void displayConnectionLost() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Connection Lost!");
  display.setCursor(0, 15);
  display.println("Reconnecting...");

  display.fillCircle(35, 40, 8, SSD1306_WHITE);
  display.fillCircle(35, 40, 4, SSD1306_BLACK);
  display.fillCircle(85, 40, 8, SSD1306_WHITE);
  display.fillCircle(85, 40, 4, SSD1306_BLACK);

  display.display();
}

void nodYes() {
  Serial.println("Executing physical YES nod");
  for (int i = 0; i < 3; i++) {
    tiltServo.write(120);
    delay(100);
    tiltServo.write(60);
    delay(100);
  }
  tiltServo.write(90);
}

void shakeNo() {
  Serial.println("Executing physical NO shake");
  for (int i = 0; i < 4; i++) {
    panServo.write(60);
    delay(100);
    panServo.write(120);
    delay(100);
  }
  panServo.write(90);
}

// NEW: Advanced study analytics and AI recommendations
struct StudyAnalytics {
  float productivityScore;      // 0-100 based on session data
  float concentrationIndex;     // Focus consistency over time
  float environmentOptimality;  // How good the study environment is
  String studyPattern;          // "morning_person", "night_owl", "consistent"
  String recommendedActions[3]; // Top 3 recommendations
  unsigned long totalStudyTime; // Cumulative study time
  int completedSessions;        // Number of finished sessions
};

StudyAnalytics analytics;

// NEW: Pomodoro timer functionality
struct PomodoroTimer {
  bool isActive;
  bool isBreak;
  unsigned long sessionLength;  // 25 minutes default
  unsigned long breakLength;    // 5 minutes default
  unsigned long currentStart;
  int completedPomodoros;
  String phase; // "work", "short_break", "long_break"
};

PomodoroTimer pomodoro;

// NEW: Environmental learning system
struct EnvironmentProfile {
  float optimalLightMin, optimalLightMax;
  float optimalTempMin, optimalTempMax;
  int optimalSoundMax;
  bool learnedPreferences;
  int profileConfidence; // 0-100
};

EnvironmentProfile userProfile;

// NEW: Motivational system
struct MotivationSystem {
  int streakDays;
  int totalPoints;
  String currentLevel; // "Beginner", "Focused", "Dedicated", "Master"
  bool achievementUnlocked;
  String lastAchievement;
  unsigned long dailyGoal; // in milliseconds
  unsigned long dailyProgress;
};

MotivationSystem motivation;

// NEW: Initialize advanced systems
void initializeAdvancedSystems() {
  // Initialize analytics
  analytics.productivityScore = 0.0;
  analytics.concentrationIndex = 0.0;
  analytics.environmentOptimality = 0.0;
  analytics.studyPattern = "unknown";
  analytics.totalStudyTime = 0;
  analytics.completedSessions = 0;
  
  // Initialize Pomodoro
  pomodoro.isActive = false;
  pomodoro.isBreak = false;
  pomodoro.sessionLength = 25 * 60 * 1000; // 25 minutes
  pomodoro.breakLength = 5 * 60 * 1000;    // 5 minutes
  pomodoro.completedPomodoros = 0;
  pomodoro.phase = "ready";
  
  // Initialize user profile
  userProfile.optimalLightMin = 200;
  userProfile.optimalLightMax = 800;
  userProfile.optimalTempMin = 20;
  userProfile.optimalTempMax = 24;
  userProfile.optimalSoundMax = 2000;
  userProfile.learnedPreferences = false;
  userProfile.profileConfidence = 0;
  
  // Initialize motivation
  motivation.streakDays = 0;
  motivation.totalPoints = 0;
  motivation.currentLevel = "Beginner";
  motivation.achievementUnlocked = false;
  motivation.dailyGoal = 2 * 60 * 60 * 1000; // 2 hours
  motivation.dailyProgress = 0;
  
  Serial.println("Advanced systems initialized");
}

// NEW: Update analytics
void updateStudyAnalytics() {
  if (currentSession.isActive) {
    // Calculate productivity score
    float focusScore = currentInsights.studyScore;
    float comfortScore = comfortData.comfortScore;
    float interactionPenalty = (currentSession.touchInteractions > 10) ? -10 : 0;
    
    analytics.productivityScore = (focusScore * 0.6 + comfortScore * 0.4 + interactionPenalty);
    analytics.productivityScore = max(0.0f, min(100.0f, analytics.productivityScore));
    
    // Update concentration index (consistency of focus)
    static float focusHistory[10] = {0};
    static int historyIndex = 0;
    
    focusHistory[historyIndex] = focusScore;
    historyIndex = (historyIndex + 1) % 10;
    
    float focusVariance = 0;
    float focusMean = 0;
    for (int i = 0; i < 10; i++) {
      focusMean += focusHistory[i];
    }
    focusMean /= 10;
    
    for (int i = 0; i < 10; i++) {
      focusVariance += pow(focusHistory[i] - focusMean, 2);
    }
    focusVariance /= 10;
    
    analytics.concentrationIndex = 100 - sqrt(focusVariance); // Lower variance = higher concentration
    
    // Learn user preferences
    learnUserPreferences();
    
    // Update motivation system
    updateMotivationSystem();
  }
}

// NEW: Learn user preferences
void learnUserPreferences() {
  // Only learn during high productivity periods
  if (analytics.productivityScore > 80) {
    userProfile.optimalLightMin = min(userProfile.optimalLightMin, currentSensors.lightLevel);
    userProfile.optimalLightMax = max(userProfile.optimalLightMax, currentSensors.lightLevel);
    
    userProfile.optimalTempMin = min(userProfile.optimalTempMin, temperatureC);
    userProfile.optimalTempMax = max(userProfile.optimalTempMax, temperatureC);
    
    userProfile.optimalSoundMax = max(userProfile.optimalSoundMax, currentSensors.soundLevel);
    
    userProfile.profileConfidence = min(100, userProfile.profileConfidence + 1);
    
    if (userProfile.profileConfidence > 50) {
      userProfile.learnedPreferences = true;
    }
  }
}

// NEW: Update motivation system
void updateMotivationSystem() {
  // Award points based on productivity
  if (analytics.productivityScore > 90) {
    motivation.totalPoints += 5;
  } else if (analytics.productivityScore > 70) {
    motivation.totalPoints += 3;
  } else if (analytics.productivityScore > 50) {
    motivation.totalPoints += 1;
  }
  
  // Update level
  if (motivation.totalPoints >= 1000) {
    motivation.currentLevel = "Master";
  } else if (motivation.totalPoints >= 500) {
    motivation.currentLevel = "Dedicated";
  } else if (motivation.totalPoints >= 100) {
    motivation.currentLevel = "Focused";
  }
  
  // Update daily progress
  motivation.dailyProgress += 1000; // Add 1 second
  
  // Check for achievements
  checkAchievements();
}

// NEW: Check for achievements
void checkAchievements() {
  // First session achievement
  if (analytics.completedSessions == 1 && !motivation.achievementUnlocked) {
    motivation.achievementUnlocked = true;
    motivation.lastAchievement = "First Session Completed!";
    celebrateAchievement();
  }
  
  // Productivity master achievement
  if (analytics.productivityScore > 95 && !motivation.achievementUnlocked) {
    motivation.achievementUnlocked = true;
    motivation.lastAchievement = "Productivity Master!";
    celebrateAchievement();
  }
  
  // Long session achievement
  if (currentSession.duration > 2 * 60 * 60 * 1000 && !motivation.achievementUnlocked) { // 2 hours
    motivation.achievementUnlocked = true;
    motivation.lastAchievement = "Marathon Studier!";
    celebrateAchievement();
  }
}

// NEW: Celebrate achievement
void celebrateAchievement() {
  Serial.println("🏆 Achievement Unlocked: " + motivation.lastAchievement);
  
  // Special celebration animation
  setEmotion("HAPPY_REACTION");
  
  // Play celebration tune
  int melody[] = {523, 659, 784, 1047, 784, 659, 523};
  int durations[] = {200, 200, 200, 400, 200, 200, 400};
  
  for (int i = 0; i < 7; i++) {
    playTone(melody[i], durations[i]);
    delay(50);
  }
  
  // Flash RGB LED
  for (int i = 0; i < 10; i++) {
    setRGBColor(255, 255, 0); // Gold
    delay(100);
    setRGBColor(0, 0, 0);
    delay(100);
  }
  
  motivation.achievementUnlocked = false; // Reset for next achievement
}

// NEW: Pomodoro timer functions
void startPomodoro() {
  pomodoro.isActive = true;
  pomodoro.isBreak = false;
  pomodoro.currentStart = millis();
  pomodoro.phase = "work";
  
  Serial.println("🍅 Pomodoro started - Work phase");
  setEmotion("HAPPY");
  playTone(800, 300);
}

void updatePomodoro() {
  if (pomodoro.isActive) {
    unsigned long elapsed = millis() - pomodoro.currentStart;
    
    if (!pomodoro.isBreak && elapsed >= pomodoro.sessionLength) {
      // Work session completed
      pomodoro.isBreak = true;
      pomodoro.currentStart = millis();
      pomodoro.completedPomodoros++;
      pomodoro.phase = (pomodoro.completedPomodoros % 4 == 0) ? "long_break" : "short_break";
      
      Serial.println("🍅 Work completed! Break time: " + pomodoro.phase);
      performStudyBreakAnimation();
      
      // Break notification
      for (int i = 0; i < 2; i++) {
        playTone(1000, 300);
        delay(200);
      }
      
    } else if (pomodoro.isBreak && elapsed >= pomodoro.breakLength) {
      // Break completed
      pomodoro.isBreak = false;
      pomodoro.currentStart = millis();
      pomodoro.phase = "work";
      
      Serial.println("🍅 Break over! Back to work");
      setEmotion("DEFAULT");
      playTone(600, 500);
    }
  }
}

// NEW: Generate AI recommendations
String generateRecommendations() {
  String recommendations = "";
  
  // Lighting recommendation
  if (userProfile.learnedPreferences) {
    if (currentSensors.lightLevel < userProfile.optimalLightMin) {
      recommendations += "💡 Increase lighting for optimal focus. ";
    } else if (currentSensors.lightLevel > userProfile.optimalLightMax) {
      recommendations += "💡 Reduce lighting to your preferred level. ";
    }
  }
  
  // Temperature recommendation
  if (temperatureC < userProfile.optimalTempMin) {
    recommendations += "🌡️ Consider warming up the room. ";
  } else if (temperatureC > userProfile.optimalTempMax) {
    recommendations += "🌡️ The room might be too warm for optimal focus. ";
  }
  
  // Break recommendation
  if (currentSession.duration > 60 * 60 * 1000 && studyBreakCount == 0) { // 1 hour no break
    recommendations += "⏰ Time for a break! You've been studying for over an hour. ";
  }
  
  // Productivity recommendation
  if (analytics.productivityScore < 60) {
    recommendations += "📈 Try the Pomodoro technique to boost productivity. ";
  }
  
  // Motivation boost
  if (currentSession.duration > 30 * 60 * 1000) { // 30 minutes
    recommendations += "🎯 Great focus! You're building excellent study habits. ";
  }
  
  return recommendations.isEmpty() ? "✅ Perfect study conditions! Keep it up!" : recommendations;
}

// NEW: Get advanced analytics JSON
String getAnalyticsJSON() {
  StaticJsonDocument<1024> doc;
  
  // Analytics data
  doc["productivity_score"] = analytics.productivityScore;
  doc["concentration_index"] = analytics.concentrationIndex;
  doc["environment_optimality"] = analytics.environmentOptimality;
  doc["study_pattern"] = analytics.studyPattern;
  doc["total_study_time"] = analytics.totalStudyTime;
  doc["completed_sessions"] = analytics.completedSessions;
  
  // Pomodoro data
  doc["pomodoro"]["active"] = pomodoro.isActive;
  doc["pomodoro"]["is_break"] = pomodoro.isBreak;
  doc["pomodoro"]["phase"] = pomodoro.phase;
  doc["pomodoro"]["completed"] = pomodoro.completedPomodoros;
  if (pomodoro.isActive) {
    unsigned long remaining;
    if (pomodoro.isBreak) {
      remaining = pomodoro.breakLength - (millis() - pomodoro.currentStart);
    } else {
      remaining = pomodoro.sessionLength - (millis() - pomodoro.currentStart);
    }
    doc["pomodoro"]["remaining_ms"] = max(0UL, remaining);
  }
  
  // User profile
  doc["profile"]["learned"] = userProfile.learnedPreferences;
  doc["profile"]["confidence"] = userProfile.profileConfidence;
  doc["profile"]["optimal_light"] = String(userProfile.optimalLightMin) + "-" + String(userProfile.optimalLightMax);
  doc["profile"]["optimal_temp"] = String(userProfile.optimalTempMin) + "-" + String(userProfile.optimalTempMax);
  
  // Motivation data
  doc["motivation"]["level"] = motivation.currentLevel;
  doc["motivation"]["points"] = motivation.totalPoints;
  doc["motivation"]["streak_days"] = motivation.streakDays;
  doc["motivation"]["daily_progress_hours"] = motivation.dailyProgress / (60 * 60 * 1000);
  doc["motivation"]["daily_goal_hours"] = motivation.dailyGoal / (60 * 60 * 1000);
  doc["motivation"]["last_achievement"] = motivation.lastAchievement;
  
  // AI Recommendations
  doc["recommendations"] = generateRecommendations();
  
  String result;
  serializeJson(doc, result);
  return result;
}