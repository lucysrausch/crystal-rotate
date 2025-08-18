// ESP32 Biaxial Stepper Motor Controller with ArtNet Control
// Using AccelStepper Library for smooth motion control

#include <Arduino.h>
#include <ETH.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <AccelStepper.h>
#include <ArtnetWifi.h>  // Standard ArtnetWifi library
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Pin Definitions
#define STP0_STEP 2
#define STP0_DIR 4
#define STP1_STEP 14
#define STP1_DIR 12
#define MOTOR_ENABLE 5
#define ESTOP_PIN 34
#define SDA_PIN 13
#define SCL_PIN 16

// Ethernet PHY Configuration for ESP32-PoE compatible pinout
#define ETH_PHY_TYPE ETH_PHY_LAN8720
#define ETH_PHY_ADDR 0
#define ETH_PHY_MDC 23
#define ETH_PHY_MDIO 18
#define ETH_PHY_POWER -1  // No power pin needed for internal oscillator
#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT

// OLED Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// System Configuration
WebServer server(80);
Preferences preferences;
ArtnetWifi artnet;

// Create stepper instances
AccelStepper stepper0(AccelStepper::DRIVER, STP0_STEP, STP0_DIR);
AccelStepper stepper1(AccelStepper::DRIVER, STP1_STEP, STP1_DIR);

// Motor Control State
struct MotorControl {
    bool isVelocityMode;
    float targetSpeed;
    long targetPosition;
    float maxSpeed;
    float acceleration;
};

MotorControl motor0Control = {false, 0, 0, 1000, 10000};
MotorControl motor1Control = {false, 0, 0, 1000, 10000};

// System State
volatile bool motorsEnabled = false;
volatile bool estopActive = false;
volatile bool artnetConnected = false;
volatile unsigned long lastArtnetPacket = 0;
const unsigned long ARTNET_TIMEOUT = 5000; // 5 seconds timeout
bool ethConnected = false;

// Configuration
String nodeName = "ArtNet Stepper Controller";
int startUniverse = 0;
float stepsPerDegree = 4.4;
bool useManualControl = false;

// Note about microstepping:
// With 1/8 microstepping: stepsPerDegree = 1.8 * 8 = 14.4
// With 1/16 microstepping: stepsPerDegree = 1.8 * 16 = 28.8

// Manual Control
int manual0Position = 0;
int manual1Position = 0;
int manual0Speed = 0;
int manual1Speed = 0;
bool manualVelocityMode = false;

// Task handles for FreeRTOS
TaskHandle_t motorTaskHandle = NULL;
portMUX_TYPE motorMux = portMUX_INITIALIZER_UNLOCKED;

// Function Prototypes
void motorTask(void *pvParameters);
void onDmxFrame(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data);
void WiFiEvent(WiFiEvent_t event);
void setupEthernet();
void setupWebServer();
void handleRoot();
void handleConfig();
void handleManualControl();
void handleStatus();
void updateDisplay();
void checkEstop();
void checkArtnetTimeout();
void disableMotors();
void enableMotors();

// Motor control task - runs on Core 0
void motorTask(void *pvParameters) {
    // Configure task for high frequency operation
    const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1ms minimum for FreeRTOS
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // For high-speed stepping, we'll do multiple steps per task iteration
    const int STEPS_PER_ITERATION = 10; // Process up to 10 steps per task run
    
    while (true) {
        portENTER_CRITICAL(&motorMux);
        
        if (motorsEnabled && !estopActive) {
            // Process multiple steps per iteration for motor 0
            for (int i = 0; i < STEPS_PER_ITERATION; i++) {
                if (motor0Control.isVelocityMode) {
                    stepper0.setSpeed(motor0Control.targetSpeed);
                    if (stepper0.runSpeed()) {
                        // Wrap position at ±180°
                        long maxSteps = 180 * stepsPerDegree;
                        if (stepper0.currentPosition() > maxSteps) {
                            stepper0.setCurrentPosition(stepper0.currentPosition() - 2 * maxSteps);
                        } else if (stepper0.currentPosition() < -maxSteps) {
                            stepper0.setCurrentPosition(stepper0.currentPosition() + 2 * maxSteps);
                        }
                    }
                } else {
                    stepper0.moveTo(motor0Control.targetPosition);
                    if (!stepper0.run()) {
                        break; // No more steps needed
                    }
                }
            }
            
            // Process multiple steps per iteration for motor 1
            for (int i = 0; i < STEPS_PER_ITERATION; i++) {
                if (motor1Control.isVelocityMode) {
                    stepper1.setSpeed(motor1Control.targetSpeed);
                    if (stepper1.runSpeed()) {
                        // Wrap position at ±180°
                        long maxSteps = 180 * stepsPerDegree;
                        if (stepper1.currentPosition() > maxSteps) {
                            stepper1.setCurrentPosition(stepper1.currentPosition() - 2 * maxSteps);
                        } else if (stepper1.currentPosition() < -maxSteps) {
                            stepper1.setCurrentPosition(stepper1.currentPosition() + 2 * maxSteps);
                        }
                    }
                } else {
                    stepper1.moveTo(motor1Control.targetPosition);
                    if (!stepper1.run()) {
                        break; // No more steps needed
                    }
                }
            }
        }
        
        portEXIT_CRITICAL(&motorMux);
        
        // Use FreeRTOS delay to prevent watchdog issues
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ArtNet DMX callback
void onDmxFrame(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data) {
    if (universe != startUniverse) return;
    
    lastArtnetPacket = millis();
    artnetConnected = true;
    
    // Ensure we have at least 6 bytes
    if (length < 6) return;
    
    // Parse ArtNet data
    uint16_t motor0Value = (data[0] << 8) | data[1];
    uint16_t motor1Value = (data[2] << 8) | data[3];
    bool velocityMode = data[4] > 127;
    bool enableMotorsCmd = data[5] > 127;
    
    portENTER_CRITICAL(&motorMux);
    
    // Update motor control modes
    motor0Control.isVelocityMode = velocityMode;
    motor1Control.isVelocityMode = velocityMode;
    
    if (velocityMode) {
        // Velocity mode: map 0-65535 to speed range
        // 32767 = stop, 0 = max CW, 65535 = max CCW
        if (motor0Value == 32767) {
            motor0Control.targetSpeed = 0;
        } else {
            // Use floating point math for precise speed mapping
            motor0Control.targetSpeed = ((float)(32767 - motor0Value) / 32767.0) * motor0Control.maxSpeed;
        }
        
        if (motor1Value == 32767) {
            motor1Control.targetSpeed = 0;
        } else {
            // Use floating point math for precise speed mapping
            motor1Control.targetSpeed = ((float)(32767 - motor1Value) / 32767.0) * motor1Control.maxSpeed;
        }
    } else {
        // Position mode: map to degrees then steps
        // 32767 = 0 deg, 0 = +180 deg, 65535 = -180 deg
        float deg0, deg1;
        
        if (motor0Value == 32767) {
            deg0 = 0;
        } else {
            deg0 = ((float)(32767 - motor0Value) / 32767.0) * 180.0;
        }
        
        if (motor1Value == 32767) {
            deg1 = 0;
        } else {
            deg1 = ((float)(32767 - motor1Value) / 32767.0) * 180.0;
        }
        
        // Calculate target position in steps
        long targetSteps0 = deg0 * stepsPerDegree;
        long targetSteps1 = deg1 * stepsPerDegree;
        
        // Calculate shortest path considering wrap-around at ±180°
        long currentPos0 = stepper0.currentPosition();
        long currentPos1 = stepper1.currentPosition();
        
        // Calculate the difference without normalizing current position first
        long diff0 = targetSteps0 - currentPos0;
        long diff1 = targetSteps1 - currentPos1;
        
        // Normalize the difference to find shortest path
        long fullRotation = 360 * stepsPerDegree;
        
        // Normalize diff0 to be within -180 to +180 degrees
        while (diff0 > 180 * stepsPerDegree) diff0 -= fullRotation;
        while (diff0 < -180 * stepsPerDegree) diff0 += fullRotation;
        
        // Normalize diff1 to be within -180 to +180 degrees
        while (diff1 > 180 * stepsPerDegree) diff1 -= fullRotation;
        while (diff1 < -180 * stepsPerDegree) diff1 += fullRotation;
        
        // Set the target position
        motor0Control.targetPosition = currentPos0 + diff0;
        motor1Control.targetPosition = currentPos1 + diff1;
    }
    
    portEXIT_CRITICAL(&motorMux);
    
    // Motor enable control
    if (enableMotorsCmd && !estopActive) {
        enableMotors();
    } else {
        disableMotors();
    }
}

void WiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_ETH_START:
            Serial.println("ETH Started");
            ETH.setHostname(nodeName.c_str());
            break;
        case ARDUINO_EVENT_ETH_CONNECTED:
            Serial.println("ETH Connected");
            break;
        case ARDUINO_EVENT_ETH_GOT_IP:
            Serial.print("ETH MAC: ");
            Serial.print(ETH.macAddress());
            Serial.print(", IPv4: ");
            Serial.print(ETH.localIP());
            if (ETH.fullDuplex()) {
                Serial.print(", FULL_DUPLEX");
            }
            Serial.print(", ");
            Serial.print(ETH.linkSpeed());
            Serial.println("Mbps");
            ethConnected = true;
            break;
        case ARDUINO_EVENT_ETH_DISCONNECTED:
            Serial.println("ETH Disconnected");
            ethConnected = false;
            break;
        case ARDUINO_EVENT_ETH_STOP:
            Serial.println("ETH Stopped");
            ethConnected = false;
            break;
        default:
            break;
    }
}

void setupEthernet() {
    WiFi.onEvent(WiFiEvent);
    // For ESP32 Arduino Core 3.x, the parameter order is: type, phy_addr, mdc, mdio, power, clk_mode
    ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO, ETH_PHY_POWER, ETH_CLK_MODE);
}

void setupWebServer() {
    server.on("/", HTTP_GET, handleRoot);
    server.on("/config", HTTP_POST, handleConfig);
    server.on("/manual", HTTP_POST, handleManualControl);
    server.on("/status", HTTP_GET, handleStatus);
    server.begin();
    Serial.println("Web server started");
}

void handleRoot() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta charset='UTF-8'>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<title>" + nodeName + "</title>";
    html += "<style>";
    html += "body{font-family:Arial;margin:20px;background:#f4f4f4;}";
    html += ".container{max-width:600px;margin:auto;background:white;padding:20px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1);}";
    html += "h1{color:#333;text-align:center;}";
    html += "input[type='number'],input[type='text']{width:100%;padding:8px;margin:8px 0;border:1px solid #ddd;border-radius:4px;}";
    html += "input[type='submit']{background:#4CAF50;color:white;padding:10px;border:none;border-radius:4px;cursor:pointer;width:100%;}";
    html += "input[type='submit']:hover{background:#45a049;}";
    html += ".status{margin:20px 0;padding:15px;background:#e7f3fe;border-left:3px solid #2196F3;border-radius:4px;}";
    html += ".manual{margin-top:20px;padding:15px;background:#fff3cd;border-radius:4px;}";
    html += ".slider{width:100%;margin:10px 0;}";
    html += ".emergency{background:#ff4444;color:white;padding:10px;text-align:center;font-weight:bold;border-radius:4px;}";
    html += ".tab{overflow:hidden;border:1px solid #ccc;background-color:#f1f1f1;border-radius:4px 4px 0 0;}";
    html += ".tab button{background-color:inherit;float:left;border:none;outline:none;cursor:pointer;padding:14px 16px;transition:0.3s;}";
    html += ".tab button:hover{background-color:#ddd;}";
    html += ".tab button.active{background-color:#ccc;}";
    html += ".tabcontent{display:none;padding:12px;border:1px solid #ccc;border-top:none;}";
    html += "</style>";
    html += "<script>";
    html += "function updateStatus(){fetch('/status').then(r=>r.json()).then(d=>{";
    html += "document.getElementById('status').innerHTML=`";
    html += "Motor 0: ${d.m0pos}&deg; (${d.m0mode})<br>";
    html += "Motor 1: ${d.m1pos}&deg; (${d.m1mode})<br>";
    html += "ArtNet: ${d.artnet?'Connected':'Disconnected'}<br>";
    html += "E-Stop: ${d.estop?'ACTIVE':'OK'}<br>";
    html += "IP: ${d.ip}`;";
    html += "if(d.estop)document.getElementById('status').classList.add('emergency');";
    html += "else document.getElementById('status').classList.remove('emergency');";
    html += "});}";
    html += "setInterval(updateStatus,500);";
    html += "function openTab(evt,tabName){";
    html += "var i,tabcontent,tablinks;";
    html += "tabcontent=document.getElementsByClassName('tabcontent');";
    html += "for(i=0;i<tabcontent.length;i++){tabcontent[i].style.display='none';}";
    html += "tablinks=document.getElementsByClassName('tablinks');";
    html += "for(i=0;i<tablinks.length;i++){tablinks[i].className=tablinks[i].className.replace(' active','');}";
    html += "document.getElementById(tabName).style.display='block';";
    html += "evt.currentTarget.className+=' active';}";
    html += "</script></head><body>";
    html += "<div class='container'>";
    html += "<h1>" + nodeName + "</h1>";
    html += "<div id='status' class='status'>Loading...</div>";
    
    // Configuration Form
    html += "<h2>Configuration</h2>";
    html += "<form action='/config' method='POST'>";
    html += "<label>Node Name:</label>";
    html += "<input type='text' name='nodename' value='" + nodeName + "'>";
    html += "<label>Start Universe:</label>";
    html += "<input type='number' name='universe' value='" + String(startUniverse) + "'>";
    html += "<label>Motor 0 Max Speed (steps/sec):</label>";
    html += "<input type='number' name='m0speed' value='" + String(motor0Control.maxSpeed) + "'>";
    html += "<label>Motor 0 Acceleration (steps/sec²):</label>";
    html += "<input type='number' name='m0accel' value='" + String(motor0Control.acceleration) + "'>";
    html += "<label>Motor 1 Max Speed (steps/sec):</label>";
    html += "<input type='number' name='m1speed' value='" + String(motor1Control.maxSpeed) + "'>";
    html += "<label>Motor 1 Acceleration (steps/sec²):</label>";
    html += "<input type='number' name='m1accel' value='" + String(motor1Control.acceleration) + "'>";
    html += "<label>Steps per Degree:</label>";
    html += "<input type='number' name='stepsdeg' step='0.1' value='" + String(stepsPerDegree) + "'>";
    html += "<input type='submit' value='Save Configuration'>";
    html += "</form>";
    
    // Manual Control with tabs
    html += "<div class='manual'>";
    html += "<h2>Manual Control</h2>";
    html += "<p>Manual control is only active when no ArtNet data is received.</p>";
    
    // Tab buttons
    html += "<div class='tab'>";
    html += "<button class='tablinks active' onclick=\"openTab(event,'Position')\">Position Mode</button>";
    html += "<button class='tablinks' onclick=\"openTab(event,'Velocity')\">Velocity Mode</button>";
    html += "</div>";
    
    // Position tab
    html += "<div id='Position' class='tabcontent' style='display:block'>";
    html += "<form action='/manual' method='POST'>";
    html += "<input type='hidden' name='mode' value='position'>";
    html += "<label>Motor 0 Position (degrees):</label>";
    html += "<input type='range' class='slider' name='m0pos' min='-180' max='180' value='0' id='m0posslider' oninput='document.getElementById(\"m0posval\").textContent=this.value+\"°\"'>";
    html += "<span id='m0posval'>0°</span><br>";
    html += "<label>Motor 1 Position (degrees):</label>";
    html += "<input type='range' class='slider' name='m1pos' min='-180' max='180' value='0' id='m1posslider' oninput='document.getElementById(\"m1posval\").textContent=this.value+\"°\"'>";
    html += "<span id='m1posval'>0°</span><br>";
    html += "<input type='submit' value='Set Position'>";
    html += "</form>";
    html += "</div>";
    
    // Velocity tab
    html += "<div id='Velocity' class='tabcontent'>";
    html += "<form action='/manual' method='POST'>";
    html += "<input type='hidden' name='mode' value='velocity'>";
    html += "<label>Motor 0 Speed (%):</label>";
    html += "<input type='range' class='slider' name='m0speed' min='-100' max='100' value='0' id='m0speedslider' oninput='document.getElementById(\"m0speedval\").textContent=this.value+\"%\"'>";
    html += "<span id='m0speedval'>0%</span><br>";
    html += "<label>Motor 1 Speed (%):</label>";
    html += "<input type='range' class='slider' name='m1speed' min='-100' max='100' value='0' id='m1speedslider' oninput='document.getElementById(\"m1speedval\").textContent=this.value+\"%\"'>";
    html += "<span id='m1speedval'>0%</span><br>";
    html += "<input type='submit' value='Set Speed'>";
    html += "</form>";
    html += "</div>";
    
    html += "</div>";
    html += "</div>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
}

void handleConfig() {
    if (server.method() == HTTP_POST) {
        nodeName = server.arg("nodename");
        startUniverse = server.arg("universe").toInt();
        motor0Control.maxSpeed = server.arg("m0speed").toFloat();
        motor0Control.acceleration = server.arg("m0accel").toFloat();
        motor1Control.maxSpeed = server.arg("m1speed").toFloat();
        motor1Control.acceleration = server.arg("m1accel").toFloat();
        stepsPerDegree = server.arg("stepsdeg").toFloat();
        
        // Update AccelStepper settings
        stepper0.setMaxSpeed(motor0Control.maxSpeed);
        stepper0.setAcceleration(motor0Control.acceleration);
        stepper1.setMaxSpeed(motor1Control.maxSpeed);
        stepper1.setAcceleration(motor1Control.acceleration);
        
        // Save to preferences
        preferences.begin("stepper_config", false);
        preferences.putString("nodename", nodeName);
        preferences.putInt("universe", startUniverse);
        preferences.putFloat("m0speed", motor0Control.maxSpeed);
        preferences.putFloat("m0accel", motor0Control.acceleration);
        preferences.putFloat("m1speed", motor1Control.maxSpeed);
        preferences.putFloat("m1accel", motor1Control.acceleration);
        preferences.putFloat("stepsdeg", stepsPerDegree);
        preferences.end();
        
        server.sendHeader("Location", "/", true);
        server.send(302, "text/plain", "");
        
        ESP.restart();
    }
}

void handleManualControl() {
    if (server.method() == HTTP_POST && !artnetConnected) {
        String mode = server.arg("mode");
        
        portENTER_CRITICAL(&motorMux);
        
        if (mode == "position") {
            manual0Position = server.arg("m0pos").toInt();
            manual1Position = server.arg("m1pos").toInt();
            
            // Calculate target position in steps
            long targetSteps0 = manual0Position * stepsPerDegree;
            long targetSteps1 = manual1Position * stepsPerDegree;
            
            // Calculate shortest path considering wrap-around at ±180°
            long currentPos0 = stepper0.currentPosition();
            long currentPos1 = stepper1.currentPosition();
            
            // Calculate the difference without normalizing current position first
            long diff0 = targetSteps0 - currentPos0;
            long diff1 = targetSteps1 - currentPos1;
            
            // Normalize the difference to find shortest path
            long fullRotation = 360 * stepsPerDegree;
            
            // Normalize diff0 to be within -180 to +180 degrees
            while (diff0 > 180 * stepsPerDegree) diff0 -= fullRotation;
            while (diff0 < -180 * stepsPerDegree) diff0 += fullRotation;
            
            // Normalize diff1 to be within -180 to +180 degrees
            while (diff1 > 180 * stepsPerDegree) diff1 -= fullRotation;
            while (diff1 < -180 * stepsPerDegree) diff1 += fullRotation;
            
            // Set the target position
            motor0Control.targetPosition = currentPos0 + diff0;
            motor1Control.targetPosition = currentPos1 + diff1;
            motor0Control.isVelocityMode = false;
            motor1Control.isVelocityMode = false;
            manualVelocityMode = false;
        } else if (mode == "velocity") {
            manual0Speed = server.arg("m0speed").toInt();
            manual1Speed = server.arg("m1speed").toInt();
            
            // Convert percentage to actual speed
            motor0Control.targetSpeed = (manual0Speed * motor0Control.maxSpeed) / 100.0;
            motor1Control.targetSpeed = (manual1Speed * motor1Control.maxSpeed) / 100.0;
            motor0Control.isVelocityMode = true;
            motor1Control.isVelocityMode = true;
            manualVelocityMode = true;
        }
        
        portEXIT_CRITICAL(&motorMux);
        
        useManualControl = true;
        enableMotors();
        
        server.sendHeader("Location", "/", true);
        server.send(302, "text/plain", "");
    }
}

void handleStatus() {
    // Calculate normalized positions for display
    long pos0 = stepper0.currentPosition();
    long pos1 = stepper1.currentPosition();
    long maxSteps = 180 * stepsPerDegree;
    
    // Normalize to ±180° range for display
    while (pos0 > maxSteps) pos0 -= 2 * maxSteps;
    while (pos0 < -maxSteps) pos0 += 2 * maxSteps;
    while (pos1 > maxSteps) pos1 -= 2 * maxSteps;
    while (pos1 < -maxSteps) pos1 += 2 * maxSteps;
    
    String json = "{";
    json += "\"m0pos\":" + String(pos0 / stepsPerDegree, 1) + ",";
    json += "\"m1pos\":" + String(pos1 / stepsPerDegree, 1) + ",";
    json += "\"m0mode\":\"" + String(motor0Control.isVelocityMode ? "Velocity" : "Position") + "\",";
    json += "\"m1mode\":\"" + String(motor1Control.isVelocityMode ? "Velocity" : "Position") + "\",";
    json += "\"artnet\":" + String(artnetConnected ? "true" : "false") + ",";
    json += "\"estop\":" + String(estopActive ? "true" : "false") + ",";
    json += "\"enabled\":" + String(motorsEnabled ? "true" : "false") + ",";
    json += "\"ip\":\"" + ETH.localIP().toString() + "\"";
    json += "}";
    
    server.send(200, "application/json", json);
}

void updateDisplay() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    if (estopActive) {
        // E-stop takes full display
        display.setCursor(30, 12);
        display.setTextSize(2);
        display.println("E-STOP!");
    } else {
        // Compact layout for 128x32
        // Line 1: Motor positions
        display.setCursor(0, 0);
        
        // Calculate normalized positions
        long pos0 = stepper0.currentPosition();
        long pos1 = stepper1.currentPosition();
        long maxSteps = 180 * stepsPerDegree;
        
        while (pos0 > maxSteps) pos0 -= 2 * maxSteps;
        while (pos0 < -maxSteps) pos0 += 2 * maxSteps;
        while (pos1 > maxSteps) pos1 -= 2 * maxSteps;
        while (pos1 < -maxSteps) pos1 += 2 * maxSteps;
        
        display.print("M0:");
        display.print(pos0 / stepsPerDegree, 0);
        display.print((char)247); // degree symbol
        display.print(" M1:");
        display.print(pos1 / stepsPerDegree, 0);
        display.print((char)247);
        
        // Line 2: Mode indicators
        display.setCursor(0, 9);
        display.print(motor0Control.isVelocityMode ? "VEL" : "POS");
        display.print(" ");
        display.print(motor1Control.isVelocityMode ? "VEL" : "POS");
        display.print(" ");
        display.print(motorsEnabled ? "EN" : "DIS");
        
        // Line 3: Network status
        display.setCursor(0, 18);
        if (artnetConnected) {
            display.print("ArtNet OK");
        } else {
            display.print("Manual");
        }
        
        // Line 4: IP address (if connected)
        display.setCursor(0, 25);
        if (ethConnected) {
            display.setTextSize(1);
            display.print(ETH.localIP());
        } else {
            display.print("No Network");
        }
    }
    
    display.display();
}

void checkEstop() {
    bool currentEstop = digitalRead(ESTOP_PIN) == HIGH;
    if (currentEstop != estopActive) {
        estopActive = currentEstop;
        if (estopActive) {
            disableMotors();
            Serial.println("EMERGENCY STOP ACTIVATED!");
        } else {
            Serial.println("E-Stop cleared");
        }
    }
}

void checkArtnetTimeout() {
    if (artnetConnected && millis() - lastArtnetPacket > ARTNET_TIMEOUT) {
        artnetConnected = false;
        disableMotors();
        Serial.println("ArtNet timeout - motors disabled");
    }
}

void enableMotors() {
    if (!estopActive) {
        digitalWrite(MOTOR_ENABLE, LOW); // TMC2209 enable is active low
        motorsEnabled = true;
        
        // Reset stepper positions to prevent jumps
        stepper0.setCurrentPosition(stepper0.currentPosition());
        stepper1.setCurrentPosition(stepper1.currentPosition());
    }
}

void disableMotors() {
    digitalWrite(MOTOR_ENABLE, HIGH);
    motorsEnabled = false;
    
    // Stop any ongoing motion
    portENTER_CRITICAL(&motorMux);
    stepper0.stop();
    stepper1.stop();
    motor0Control.targetSpeed = 0;
    motor1Control.targetSpeed = 0;
    portEXIT_CRITICAL(&motorMux);
}

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 ArtNet Stepper Controller Starting...");
    
    // Initialize pins
    pinMode(STP0_STEP, OUTPUT);
    pinMode(STP0_DIR, OUTPUT);
    pinMode(STP1_STEP, OUTPUT);
    pinMode(STP1_DIR, OUTPUT);
    pinMode(MOTOR_ENABLE, OUTPUT);
    pinMode(ESTOP_PIN, INPUT_PULLUP);
    
    // Disable motors on startup
    disableMotors();
    
    // Initialize I2C for OLED
    Wire.begin(SDA_PIN, SCL_PIN);
    
    // Initialize OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("SSD1306 allocation failed");
    }
    display.clearDisplay();
    display.display();
    
    // Load configuration
    preferences.begin("stepper_config", true);
    nodeName = preferences.getString("nodename", nodeName);
    startUniverse = preferences.getInt("universe", startUniverse);
    motor0Control.maxSpeed = preferences.getFloat("m0speed", motor0Control.maxSpeed);
    motor0Control.acceleration = preferences.getFloat("m0accel", motor0Control.acceleration);
    motor1Control.maxSpeed = preferences.getFloat("m1speed", motor1Control.maxSpeed);
    motor1Control.acceleration = preferences.getFloat("m1accel", motor1Control.acceleration);
    stepsPerDegree = preferences.getFloat("stepsdeg", stepsPerDegree);
    preferences.end();
    
    // Configure AccelStepper
    stepper0.setMaxSpeed(motor0Control.maxSpeed);
    stepper0.setAcceleration(motor0Control.acceleration);
    stepper1.setMaxSpeed(motor1Control.maxSpeed);
    stepper1.setAcceleration(motor1Control.acceleration);
    
    // Setup Ethernet
    setupEthernet();
    
    // Wait for network
    unsigned long start = millis();
    while (!ethConnected && millis() - start < 10000) {
        delay(100);
    }
    
    if (ethConnected) {
        // Initialize ArtNet
        artnet.begin();
        artnet.setArtDmxCallback(onDmxFrame);
        
        // Set the ArtNet node name (if supported by your version)
        // Some versions of ArtnetWifi library have these methods:
        // artnet.setShortName(nodeName.substring(0, 17).c_str());
        // artnet.setLongName(nodeName.c_str());
        
        // Alternative: If your library version doesn't support setting names,
        // you may need to modify the library or use a fork that does.
        // The node will still work but may show as "Art-Net Node" in Resolume
        
        Serial.print("ArtNet Node started at ");
        Serial.println(ETH.localIP());
        Serial.print("Node name: ");
        Serial.println(nodeName);
        Serial.print("Universe: ");
        Serial.println(startUniverse);
        Serial.println("Note: Node name may appear as 'Art-Net Node' in Resolume");
        Serial.println("if the library doesn't support custom names");
    } else {
        Serial.println("Ethernet not connected");
    }
    
    // Setup web server
    setupWebServer();
    
    // Create motor control task on Core 0 (Core 1 is for Arduino loop)
    xTaskCreatePinnedToCore(
        motorTask,
        "MotorTask",
        4096,
        NULL,
        1, // Priority
        &motorTaskHandle,
        0  // Core 0
    );
    
    // Note: The ArtnetWifi library handles ArtPoll replies automatically
    // when artnet.read() is called in the main loop
    
    Serial.println("Setup complete");
    Serial.println("ArtNet node will be discoverable in Resolume automatically");
}

void loop() {
    static unsigned long lastDisplayUpdate = 0;
    
    // Process ArtNet packets - this handles ArtPoll replies automatically
    if (ethConnected) {
        artnet.read();
    }
    
    // Handle web server
    server.handleClient();
    
    // Check E-Stop
    checkEstop();
    
    // Check ArtNet timeout
    checkArtnetTimeout();
    
    // Update display every 100ms
    if (millis() - lastDisplayUpdate > 100) {
        updateDisplay();
        lastDisplayUpdate = millis();
    }
    
    // Handle manual control when no ArtNet
    if (!artnetConnected && useManualControl && !estopActive) {
        enableMotors();
    }
    
    delay(1);
}