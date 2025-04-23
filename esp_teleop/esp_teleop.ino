#include <Acrome-SMD.h>
#include <HardwareSerial.h>
#include <WiFi.h>

// Motor definitions
#define ID_LEFT  0x00  // Left wheel motor ID
#define ID_RIGHT 0x01  // Right wheel motor ID
#define BAUDRATE 115200
#define ServoID 5

// Linear actuator definitions
#define LINEAR_MOTOR_ID  0x02
#define MAX_LENGTH_CM 10.16  // 4 inch = 10.16 cm

HardwareSerial mySerial(2);
Red motorLeft(ID_LEFT, mySerial, BAUDRATE);
Red motorRight(ID_RIGHT, mySerial, BAUDRATE);
Red linearMotor(LINEAR_MOTOR_ID, mySerial, BAUDRATE);

// Wi-Fi Settings
const char *ssid = "**************************";
const char *password = "**********************";

WiFiServer server(80);
int servoAngle = 0;
int robotVelocity = 50;  // Default speed for robot
int linearVelocity = 50;   // Default speed for linear actuator
String currentDirection = "STOP"; // Keeps track of current direction
String currentLinearDirection = "STOP"; // Keeps track of linear actuator direction

// Linear actuator variables
int adcValue = 0;
float positionCM = 0;
int adcMin = 4095;
int adcMax = 0;
bool calibrationDone = false;

void setup() {
    Serial.begin(115200);
    mySerial.begin(BAUDRATE, SERIAL_8N1, 17, 16);
    analogReadResolution(12);

    delay(100);

    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    server.begin();

    // Initialize robot motors
    motorLeft.begin();
    delay(10);
    motorRight.begin();
    delay(10);

    float CPR = 6533;

    motorLeft.setMotorCPR(CPR);
    delay(10);
    motorLeft.setMotorRPM(robotVelocity);
    delay(10);
    motorLeft.setOperationMode(VelocityControl);
    delay(10);
    motorLeft.setControlParameters(VelocityControl, 10.0, 0.1, 0, 0, 0);
    delay(10);

    motorRight.setMotorCPR(CPR);
    delay(10);
    motorRight.setMotorRPM(robotVelocity);
    delay(10);
    motorRight.setOperationMode(VelocityControl);
    delay(10);
    motorRight.setControlParameters(VelocityControl, 10.0, 0.1, 0, 0, 0);
    delay(10);

    // Initialize linear actuator
    linearMotor.begin();
    delay(200);
    linearMotor.setMotorCPR(6533);
    linearMotor.setMotorRPM(linearVelocity);
    linearMotor.setOperationMode(VelocityControl);
    linearMotor.setControlParameters(VelocityControl, 10, 0, 1, 0, 5);

    calibrateLinearActuator();
}

void calibrateLinearActuator() {
    Serial.println("Calibrating linear actuator...");
    controlLinearMotor("FORWARD");
    for (int i = 0; i < 30; i++) {
        readLinearPosition();
        if (adcValue > adcMax) adcMax = adcValue;
        delay(200);
    }

    controlLinearMotor("BACKWARD");
    for (int i = 0; i < 30; i++) {
        readLinearPosition();
        if (adcValue < adcMin) adcMin = adcValue;
        delay(200);
    }

    controlLinearMotor("STOP");
    calibrationDone = true;

    Serial.print("ADC Min: "); Serial.println(adcMin);
    Serial.print("ADC Max: "); Serial.println(adcMax);
}

void applyRobotSpeed() {
    if (currentDirection == "FORWARD") {
        motorLeft.setpoint(VelocityControl, -robotVelocity);
        motorRight.setpoint(VelocityControl, robotVelocity);
    } else if (currentDirection == "BACKWARD") {
        motorLeft.setpoint(VelocityControl, robotVelocity);
        motorRight.setpoint(VelocityControl, -robotVelocity);
    } else if (currentDirection == "LEFT") {
        motorLeft.setpoint(VelocityControl, robotVelocity);
        motorRight.setpoint(VelocityControl, robotVelocity);
    } else if (currentDirection == "RIGHT") {
        motorLeft.setpoint(VelocityControl, -robotVelocity);
        motorRight.setpoint(VelocityControl, -robotVelocity);
    }
}

void controlRobot(String direction) {
    currentDirection = direction;

    if (direction == "FORWARD" || direction == "BACKWARD" || direction == "LEFT" || direction == "RIGHT") {
        motorLeft.torqueEnable(1);
        motorRight.torqueEnable(1);
        applyRobotSpeed();
    } else if (direction == "STOP") {
        motorLeft.setpoint(VelocityControl, 0);
        motorRight.setpoint(VelocityControl, 0);
        delay(100);
        motorLeft.torqueEnable(0);
        motorRight.torqueEnable(0);
    }
}

void setLinearSpeed(int value) {
    linearVelocity = value;
    linearMotor.setMotorRPM(linearVelocity);
    
    // If motor is moving, apply new speed immediately
    if (currentLinearDirection == "FORWARD") {
        linearMotor.setpoint(VelocityControl, linearVelocity);
    } else if (currentLinearDirection == "BACKWARD") {
        linearMotor.setpoint(VelocityControl, -linearVelocity);
    }
}

void controlLinearMotor(String direction) {
    currentLinearDirection = direction;

    if (direction == "FORWARD") {
        linearMotor.torqueEnable(1);
        linearMotor.setpoint(VelocityControl, linearVelocity);
    } else if (direction == "BACKWARD") {
        linearMotor.torqueEnable(1);
        linearMotor.setpoint(VelocityControl, -linearVelocity);
    } else {
        linearMotor.setpoint(VelocityControl, 0);
        delay(100);
        linearMotor.torqueEnable(0);
    }
}

void readLinearPosition() {
    int sum = 0;
    for (int i = 0; i < 5; i++) {
        sum += linearMotor.getAnalogPort();
        delay(5);
    }
    adcValue = sum / 5;

    if (calibrationDone && adcMax != adcMin) {
        float ratio = float(adcValue - adcMin) / float(adcMax - adcMin);
        ratio = constrain(ratio, 0.0, 1.0);
        positionCM = ratio * MAX_LENGTH_CM;
    }

    Serial.print("ADC: "); Serial.print(adcValue);
    Serial.print(" | Pos (cm): "); Serial.println(positionCM);
}

void moveToCM(float targetCM) {
    targetCM = constrain(targetCM, 0.0, MAX_LENGTH_CM);
    int targetADC = map(targetCM * 100, 0, MAX_LENGTH_CM * 100, adcMin, adcMax);

    if (targetADC > adcValue) {
        controlLinearMotor("FORWARD");
        while (adcValue < targetADC - 5) {
            readLinearPosition();
        }
    } else {
        controlLinearMotor("BACKWARD");
        while (adcValue > targetADC + 5) {
            readLinearPosition();
        }
    }
    controlLinearMotor("STOP");
}

void loop() {
    readLinearPosition();

    WiFiClient client = server.available();
    if (client) {
        String request = client.readStringUntil('\r');
        client.flush();

        if (request.indexOf("/FORWARD") != -1) {
            controlRobot("FORWARD");
        } else if (request.indexOf("/BACKWARD") != -1) {
            controlRobot("BACKWARD");
        } else if (request.indexOf("/LEFT") != -1) {
            controlRobot("LEFT");
        } else if (request.indexOf("/RIGHT") != -1) {
            controlRobot("RIGHT");
        } else if (request.indexOf("/STOP") != -1) {
            controlRobot("STOP");
        } else if (request.indexOf("/ROBOT_SPEED?value=") != -1) {
            int index = request.indexOf("value=") + 6;
            String speedString = request.substring(index);
            robotVelocity = speedString.toInt();
            applyRobotSpeed();
        } else if (request.indexOf("/LINEAR_SPEED?value=") != -1) {
            int index = request.indexOf("value=") + 6;
            String speedString = request.substring(index);
            linearVelocity = speedString.toInt();
            setLinearSpeed(linearVelocity); // Apply speed immediately
        } else if (request.indexOf("/LINEAR_FORWARD") != -1) {
            controlLinearMotor("FORWARD");
        } else if (request.indexOf("/LINEAR_BACKWARD") != -1) {
            controlLinearMotor("BACKWARD");
        } else if (request.indexOf("/LINEAR_STOP") != -1) {
            controlLinearMotor("STOP");
        } else if (request.indexOf("/SET_CM_VALUE?value=") != -1) {
            int index = request.indexOf("value=") + 6;
            float targetCM = request.substring(index).toFloat();
            moveToCM(targetCM);
        } else if (request.indexOf("/SERVO_ANGLE?value=") != -1) {
            int index = request.indexOf("value=") + 6;
            servoAngle = request.substring(index).toInt();
            servoAngle = constrain(servoAngle, 0, 180);
            motorLeft.setServo(ServoID, servoAngle);
            Serial.print("Servo angle set to: ");
            Serial.println(servoAngle);
        }

        client.println("HTTP/1.1 200 OK");
        client.println("Content-type:text/html; charset=UTF-8");
        client.println();
        client.println("<!DOCTYPE html>");
        client.println("<html lang='en'><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
        client.println("<title>Mobile Robot ESP32 TeleOp</title>");
        client.println("<style>");
        client.println("body { text-align: center; font-family: 'Segoe UI', Arial, sans-serif; background-color: #000000; color: #ffffff; margin: 0; padding: 20px; }");
        client.println(".container { max-width: 1200px; margin: 0 auto; background-color: #121212; border-radius: 10px; padding: 20px; box-shadow: 0 4px 8px rgba(255,255,255,0.1); }");
        client.println("h1 { color: #ffffff; font-size: 28px; margin-bottom: 5px; }");
        client.println("h2 { color: #cccccc; font-size: 20px; margin-top: 10px; }");
        client.println(".stream-container { background-color: #1a1a1a; padding: 10px; border-radius: 10px; margin: 15px 0; border: 1px solid #333333; }");
        client.println(".control-panel { display: flex; flex-wrap: wrap; gap: 15px; justify-content: center; margin: 20px 0; }");
        client.println(".control-section { flex: 1; min-width: 300px; background-color: #1a1a1a; border-radius: 10px; padding: 15px; border: 1px solid #333333; }");
        client.println(".button { width: 100px; height: 100px; font-size: 18px; font-weight: bold; border: none; color: white; margin: 5px; border-radius: 10px; cursor: pointer; box-shadow: 0 4px 6px rgba(0,0,0,0.3); transition: all 0.2s ease; }");
        client.println(".button:hover { opacity: 0.9; }");
        client.println(".button:active { transform: scale(0.98); }");
        client.println(".forward { background-color: #8B0000; }");
        client.println(".backward { background-color: #8B0000; }");
        client.println(".left { background-color: #333333; }");
        client.println(".right { background-color: #333333; }");
        client.println(".stop { background-color: #555555; width: 110px; height: 110px; font-size: 22px; }");
        client.println(".controller { display: grid; grid-template-columns: repeat(3, 1fr); justify-content: center; align-items: center; gap: 10px; margin: 20px 0; }");
        client.println(".speed-control { padding: 15px; border-radius: 10px; margin: 15px 0; background-color: #1a1a1a; border: 1px solid #333333; }");
        client.println(".slider-container { display: flex; flex-direction: column; align-items: center; }");
        client.println(".slider { width: 90%; margin-top: 10px; -webkit-appearance: none; height: 12px; border-radius: 6px; background: #333333; outline: none; }");
        client.println(".slider::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 22px; height: 22px; border-radius: 50%; background: #8B0000; cursor: pointer; border: 2px solid #ffffff; }");
        client.println(".speed-value { font-size: 24px; color: #ffffff; margin: 10px 0; font-weight: bold; }");
        client.println(".speed-label { font-size: 18px; color: #cccccc; margin-bottom: 10px; }");
        client.println(".keybind-info { background-color: #1a1a1a; padding: 15px; border-radius: 10px; margin-top: 20px; text-align: left; border: 1px solid #333333; }");
        client.println(".keybind-info h3 { color: #ffffff; margin-top: 0; }");
        client.println(".keybind-grid { display: grid; grid-template-columns: auto auto; gap: 10px; }");
        client.println(".keybind { display: flex; align-items: center; }");
        client.println(".key { background-color: #333333; padding: 5px 10px; border-radius: 5px; margin-right: 10px; font-family: monospace; color: #ffffff; }");
        client.println(".status { position: fixed; top: 10px; right: 10px; background-color: #1a1a1a; padding: 10px; border-radius: 10px; font-size: 14px; border: 1px solid #333333; }");
        client.println(".linear-buttons { display: flex; justify-content: center; gap: 10px; margin-bottom: 15px; }");
        client.println(".linear-button { padding: 12px 24px; font-size: 16px; border-radius: 8px; border: none; color: white; cursor: pointer; transition: all 0.2s ease; }");
        client.println(".linear-button:hover { opacity: 0.9; }");
        client.println(".linear-forward { background-color: #8B0000; }");
        client.println(".linear-backward { background-color: #8B0000; }");
        client.println(".linear-stop { background-color: #555555; }");
        client.println(".linear-go { background-color: #333333; }");
        client.println(".position-input { display: flex; gap: 10px; justify-content: center; align-items: center; margin: 15px 0; }");
        client.println(".position-input input { width: 150px; padding: 10px; border-radius: 8px; border: 1px solid #333333; background-color: #1a1a1a; color: white; font-size: 16px; }");
        client.println(".clear-btn { background-color: #555555; padding: 10px 15px; border-radius: 8px; border: none; color: white; cursor: pointer; font-size: 16px; }");
        client.println(".servo-control { margin-top: 20px; background-color: #1a1a1a; padding: 15px; border-radius: 10px; border: 1px solid #333333; }");
        client.println(".servo-value { font-size: 24px; color: #ffffff; margin: 10px 0; }");
        client.println("@media (max-width: 768px) {");
        client.println("  .button { width: 80px; height: 80px; font-size: 16px; }");
        client.println("  .stop { width: 90px; height: 90px; }");
        client.println("  .control-section { min-width: 100%; }");
        client.println("}");
        client.println("</style>");
        client.println("</head><body>");
        client.println("<div class='container'>");
        client.println("<h1>Mobile Robot ESP32 TeleOp</h1>");
        
        client.println("<div class='stream-container'>");
        client.println("<h2>Camera Feed</h2>");
        client.println("<img src=\"http://192.168.167.130:81/stream\" width=\"320\" height=\"240\" style=\"border-radius: 8px; border: 1px solid #333333;\">");
        client.println("</div>");
        
        client.println("<div class='control-panel'>");
        
        // Robot Control Section
        client.println("<div class='control-section'>");
        client.println("<h2>Robot Control</h2>");
        
        client.println("<div class='controller'>");
        client.println("<div></div><button class='button forward' onclick=\"sendCommand('/FORWARD')\">Forward</button><div></div>");
        client.println("<button class='button left' onclick=\"sendCommand('/LEFT')\">Left</button>");
        client.println("<button class='button stop' onclick=\"sendCommand('/STOP')\">Stop</button>");
        client.println("<button class='button right' onclick=\"sendCommand('/RIGHT')\">Right</button>");
        client.println("<div></div><button class='button backward' onclick=\"sendCommand('/BACKWARD')\">Backward</button><div></div>");
        client.println("</div>");
        
        client.println("<div class='speed-control'>");
        client.println("<div class='speed-label'>Robot Speed</div>");
        client.println("<div class='slider-container'>");
        client.println("<div class='speed-value' id='robotSpeedValue'>" + String(robotVelocity) + "%</div>");
        client.println("<input type='range' min='0' max='100' value='" + String(robotVelocity) + "' class='slider' id='robotSpeed' oninput='updateRobotSpeed(this.value)' onchange='setRobotSpeed(this.value)'>");
        client.println("</div>");
        client.println("</div>");
        client.println("</div>"); // End robot control section
        
        // Linear Actuator Control Section
        client.println("<div class='control-section'>");
        client.println("<h2>Linear Actuator Control</h2>");
        
        client.println("<div class='linear-buttons'>");
        client.println("<button class='linear-button linear-forward' onclick=\"sendCommand('/LINEAR_FORWARD')\">Forward</button>");
        client.println("<button class='linear-button linear-stop' onclick=\"sendCommand('/LINEAR_STOP')\">Stop</button>");
        client.println("<button class='linear-button linear-backward' onclick=\"sendCommand('/LINEAR_BACKWARD')\">Backward</button>");
        client.println("</div>");
        
        client.println("<div class='speed-control'>");
        client.println("<div class='speed-label'>Linear Actuator Speed</div>");
        client.println("<div class='slider-container'>");
        client.println("<div class='speed-value' id='linearSpeedValue'>" + String(linearVelocity) + "%</div>");
        client.println("<input type='range' min='0' max='100' value='" + String(linearVelocity) + "' class='slider' id='linearSpeed' oninput='updateLinearSpeed(this.value)' onchange='setLinearSpeed(this.value)'>");
        client.println("</div>");
        client.println("</div>");
        
        client.println("<h3>Position Control</h3>");
        client.println("<div class='position-input'>");
        client.println("<input type='number' id='cmInput' min='0' max='" + String(MAX_LENGTH_CM, 2) + "' step='0.01' placeholder='0 - " + String(MAX_LENGTH_CM, 2) + " cm'>");
        client.println("<button class='linear-button linear-go' onclick='setTargetCM()'>Go to Position</button>");
        client.println("<button class='clear-btn' onclick=\"document.getElementById('cmInput').value=''\">Clear</button>");
        client.println("</div>");
        
        client.println("<h3>Current Position</h3>");
        client.println("<p><strong><span id='cmVal'>" + String(positionCM, 2) + "</span> cm</strong></p>");
        client.println("</div>"); // End linear actuator section
        
        // Servo Control Section (updated to match linear actuator style)
        client.println("<div class='control-section'>");
        client.println("<h2>Servo Control</h2>");
        client.println("<div class='servo-control'>");
        client.println("<div class='servo-value'>Current Angle: <span id='servoValue'>" + String(servoAngle) + "</span>°</div>");
        client.println("<h3>Angle Control</h3>");
        client.println("<div class='position-input'>");
        client.println("<input type='number' id='servoInput' min='0' max='180' placeholder='0 - 180'>");
        client.println("<button class='linear-button linear-go' onclick='setServoAngleFromInput()'>Set Angle</button>");
        client.println("<button class='clear-btn' onclick=\"document.getElementById('servoInput').value=''\">Clear</button>");
        client.println("</div>");
        client.println("</div>");
        client.println("</div>"); // End servo control section
        
        client.println("</div>"); // End control panel
        
        client.println("<div class='keybind-info'>");
        client.println("<h3>Keyboard Shortcuts</h3>");
        client.println("<div class='keybind-grid'>");
        client.println("<div class='keybind'><span class='key'>W</span> Robot Forward</div>");
        client.println("<div class='keybind'><span class='key'>S</span> Robot Backward</div>");
        client.println("<div class='keybind'><span class='key'>A</span> Robot Left</div>");
        client.println("<div class='keybind'><span class='key'>D</span> Robot Right</div>");
        client.println("<div class='keybind'><span class='key'>Space</span> Robot Stop</div>");
        client.println("<div class='keybind'><span class='key'>↑</span> Increase Robot Speed</div>");
        client.println("<div class='keybind'><span class='key'>↓</span> Decrease Robot Speed</div>");
        client.println("<div class='keybind'><span class='key'>F</span> Linear Forward</div>");
        client.println("<div class='keybind'><span class='key'>B</span> Linear Backward</div>");
        client.println("<div class='keybind'><span class='key'>X</span> Linear Stop</div>");
        client.println("<div class='keybind'><span class='key'>Backspace</span> Clear Input</div>");
        client.println("<div class='keybind'><span class='key'>Enter</span> Go to Position/Set Angle</div>");
        client.println("<div class='keybind'><span class='key'>+</span> Increase Linear Speed</div>");
        client.println("<div class='keybind'><span class='key'>-</span> Decrease Linear Speed</div>");
        client.println("<div class='keybind'><span class='key'>0-9</span> Quick Servo Positions (0=0°,1=10°,etc)</div>");
        client.println("</div>");
        client.println("</div>");
        
        client.println("<div class='status' id='status'>Status: " + currentDirection + " | Servo: " + String(servoAngle) + "°</div>");
        client.println("</div>");
        
        client.println("<script>");
        client.println("function sendCommand(command) {");
        client.println("  fetch(command);");
        client.println("  updateStatus(command.replace('/', ''));");
        client.println("}");
        
        client.println("function updateStatus(command) {");
        client.println("  document.getElementById('status').innerHTML = 'Status: ' + command + ' | Servo: ' + document.getElementById('servoValue').innerHTML;");
        client.println("}");
        
        client.println("function updateRobotSpeed(value) {");
        client.println("  document.getElementById('robotSpeedValue').innerHTML = value + '%';");
        client.println("}");
        
        client.println("function setRobotSpeed(value) {");
        client.println("  document.getElementById('robotSpeed').value = value;");
        client.println("  document.getElementById('robotSpeedValue').innerHTML = value + '%';");
        client.println("  fetch('/ROBOT_SPEED?value=' + value);");
        client.println("}");
        
        client.println("function updateLinearSpeed(value) {");
        client.println("  document.getElementById('linearSpeedValue').innerHTML = value + '%';");
        client.println("}");
        
        client.println("function setLinearSpeed(value) {");
        client.println("  document.getElementById('linearSpeed').value = value;");
        client.println("  document.getElementById('linearSpeedValue').innerHTML = value + '%';");
        client.println("  fetch('/LINEAR_SPEED?value=' + value);");
        client.println("}");
        
        client.println("function setTargetCM() {");
        client.println("  var cm = parseFloat(document.getElementById('cmInput').value);");
        client.println("  if (isNaN(cm) || cm < 0 || cm > " + String(MAX_LENGTH_CM, 2) + ") { ");
        client.println("    alert('Please enter a value between 0 and " + String(MAX_LENGTH_CM, 2) + " cm'); return; ");
        client.println("  }");
        client.println("  fetch('/SET_CM_VALUE?value=' + cm);");
        client.println("}");
        
        client.println("function setServoAngleFromInput() {");
        client.println("  var angle = parseInt(document.getElementById('servoInput').value);");
        client.println("  if (isNaN(angle) || angle < 0 || angle > 180) {");
        client.println("    alert('Please enter a value between 0 and 180'); return;");
        client.println("  }");
        client.println("  fetch('/SERVO_ANGLE?value=' + angle);");
        client.println("  document.getElementById('servoValue').innerHTML = angle + '°';");
        client.println("  updateStatus('Servo: ' + angle + '°');");
        client.println("  document.getElementById('servoInput').value = ''; // Clear input after setting");
        client.println("}");
        
        client.println("document.addEventListener(\"keydown\", function(event) {");
        client.println("  let cmInput = document.getElementById('cmInput');");
        client.println("  let servoInput = document.getElementById('servoInput');");
        client.println("  let robotSpeedSlider = document.getElementById('robotSpeed');");
        client.println("  let linearSpeedSlider = document.getElementById('linearSpeed');");
        client.println("  let currentRobotSpeed = parseInt(robotSpeedSlider.value);");
        client.println("  let currentLinearSpeed = parseInt(linearSpeedSlider.value);");
        
        client.println("  // Only prevent default for specific keys we handle");
        client.println("  const handledKeys = ['w', 'W', 's', 'S', 'a', 'A', 'd', 'D', ' ', 'ArrowUp', 'ArrowDown', ");
        client.println("    'f', 'F', 'b', 'B', 'x', 'X', '+', '-', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9'];");
        client.println("  ");
        client.println("  // Check if we're in an input field");
        client.println("  const activeElement = document.activeElement;");
        client.println("  const isInputFocused = (activeElement === cmInput || activeElement === servoInput);");
        client.println("  ");
        client.println("  // If we're in an input field, only handle Enter and Backspace");
        client.println("  if (isInputFocused) {");
        client.println("    if (event.key === 'Enter') {");
        client.println("      if (activeElement === cmInput) setTargetCM();");
        client.println("      else if (activeElement === servoInput) setServoAngleFromInput();");
        client.println("      event.preventDefault();");
        client.println("    }");
        client.println("    return;");
        client.println("  }");
        client.println("  ");
        client.println("  // For other keys, prevent default only if we handle them");
        client.println("  if (handledKeys.includes(event.key)) {");
        client.println("    event.preventDefault();");
        client.println("  }");
        
        client.println("  // Robot controls");
        client.println("  switch(event.key.toLowerCase()) {");
        client.println("    case \"w\": sendCommand('/FORWARD'); break;");
        client.println("    case \"s\": sendCommand('/BACKWARD'); break;");
        client.println("    case \"a\": sendCommand('/LEFT'); break;");
        client.println("    case \"d\": sendCommand('/RIGHT'); break;");
        client.println("    case \" \": sendCommand('/STOP'); break;");
        client.println("    case \"arrowup\":");
        client.println("      let newSpeedUp = Math.min(currentRobotSpeed + 5, 100);");
        client.println("      setRobotSpeed(newSpeedUp);");
        client.println("      break;");
        client.println("    case \"arrowdown\":");
        client.println("      let newSpeedDown = Math.max(currentRobotSpeed - 5, 0);");
        client.println("      setRobotSpeed(newSpeedDown);");
        client.println("      break;");
        client.println("  }");
        
        client.println("  // Linear actuator controls");
        client.println("  switch(event.key.toLowerCase()) {");
        client.println("    case \"f\": sendCommand('/LINEAR_FORWARD'); break;");
        client.println("    case \"b\": sendCommand('/LINEAR_BACKWARD'); break;");
        client.println("    case \"x\": sendCommand('/LINEAR_STOP'); break;");
        client.println("    case \"+\":");
        client.println("      let newLinearSpeedUp = Math.min(currentLinearSpeed + 5, 100);");
        client.println("      setLinearSpeed(newLinearSpeedUp);");
        client.println("      break;");
        client.println("    case \"-\":");
        client.println("      let newLinearSpeedDown = Math.max(currentLinearSpeed - 5, 0);");
        client.println("      setLinearSpeed(newLinearSpeedDown);");
        client.println("      break;");
        client.println("  }");
        
        client.println("  // Servo controls");
        client.println("  if (event.key >= '0' && event.key <= '9') {");
        client.println("    let angle = 0;");
        client.println("    if (event.key === '0') angle = 0;");
        client.println("    else angle = (parseInt(event.key) * 20) - 10; // 1=10°, 2=30°, ..., 9=170°");
        client.println("    fetch('/SERVO_ANGLE?value=' + angle);");
        client.println("    document.getElementById('servoValue').innerHTML = angle + '°';");
        client.println("    updateStatus('Servo: ' + angle + '°');");
        client.println("  }");
        client.println("});");
        
        client.println("// Update linear actuator position and servo angle every second");
        client.println("setInterval(() => {");
        client.println("  fetch('/').then(r => r.text()).then(t => {");
        client.println("    var cm = t.match(/<span id='cmVal'>([\\d\\.]+)<\\/span>/);");
        client.println("    if(cm) { document.getElementById('cmVal').innerText = cm[1]; }");
        client.println("    var servo = t.match(/<span id='servoValue'>([\\d]+)<\\/span>/);");
        client.println("    if(servo) { ");
        client.println("      document.getElementById('servoValue').innerText = servo[1];");
        client.println("    }");
        client.println("  });");
        client.println("}, 1000);");
        
        client.println("</script>");
        
        client.println("</body></html>");
        client.println();
    }
}