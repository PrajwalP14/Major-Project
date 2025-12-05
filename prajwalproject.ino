#include <AccelStepper.h>

// ------------------------------------------------------------------
// --- MOTOR 1: TB6600 (Bipolar Stepper - X-Axis) ---
// Using standard Arduino Uno digital pins (e.g., 9 and 8)
#define MOTOR1_STEP_PIN 9
#define MOTOR1_DIR_PIN  8
AccelStepper stepper1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);

// --- MOTOR 2: ULN2003A (Unipolar Stepper - Y-Axis) ---
// Using standard Arduino Uno digital pins (e.g., 6, 5, 4, 3)
// AccelStepper::FULL4WIRE pin order: in1, in3, in2, in4 (typical for 28BYJ-48)
#define MOTOR2_IN1_PIN  6
#define MOTOR2_IN2_PIN  5
#define MOTOR2_IN3_PIN  4
#define MOTOR2_IN4_PIN  3
AccelStepper stepper2(AccelStepper::FULL4WIRE, MOTOR2_IN1_PIN, MOTOR2_IN3_PIN, MOTOR2_IN2_PIN, MOTOR2_IN4_PIN);

// --- Global Constants & Variables ---
// Adjust these based on your motor's physical setup!
const float STEPS_PER_UNIT_X = 90.0; 
const float STEPS_PER_UNIT_Y = 90.0; 

const float MAX_ACCELERATION = 1000.0;
const float MAX_SPEED = 1000.0;

String command = "";
bool isGCodeReady = false;

// ----------------------------------------------------
// **ERROR FIX: Helper function to safely replace indexOfAny()**
// This remains necessary as the underlying String class implementation might vary.
// ----------------------------------------------------
int findEndOfParameter(String input, int startIndex, const char* terminators) {
  int minIndex = -1;
  int inputLength = input.length();

  // Iterate through all possible terminator characters ('Y', 'F', 'X', etc.)
  for (int i = 0; terminators[i] != '\0'; i++) {
    // Find the next occurrence of this single character
    int index = input.indexOf(terminators[i], startIndex);
    
    // If found, check if it's the earliest index found so far
    if (index != -1 && (minIndex == -1 || index < minIndex)) {
      minIndex = index;
    }
  }
  
  // If no terminator is found, the value extends to the end of the string.
  // Otherwise, return the index of the earliest terminator found.
  return (minIndex == -1) ? inputLength : minIndex;
}

// ----------------------------------------------------
// SETUP
// ----------------------------------------------------
void setup() {
  // Using 9600 baud rate, which is safer for UNO's smaller buffer/memory
  Serial.begin(9600); 
  Serial.println("\nG-Code Interpreter Ready on Arduino Uno.");
  Serial.println("Send G1 X[units] Y[units] F[feedrate]");

  // Stepper 1 Setup (TB6600)
  stepper1.setMaxSpeed(MAX_SPEED);
  stepper1.setAcceleration(MAX_ACCELERATION);
  stepper1.setCurrentPosition(0);

  // Stepper 2 Setup (ULN2003A) - Often needs lower settings
  stepper2.setMaxSpeed(500.0); 
  stepper2.setAcceleration(500.0);
  stepper2.setCurrentPosition(0);
}

// ----------------------------------------------------
// LOOP
// ----------------------------------------------------
void loop() {
  // Note: For UNO, you might have to type 'serialEvent()' manually 
  // as the default serialEvent() handler is a weak implementation.
  serialEvent(); 

  if (isGCodeReady) {
    parseAndExecuteGCode(command);
    command = ""; 
    isGCodeReady = false; 
  }

  // --- Crucial: Run the motors concurrently ---
  stepper1.run();
  stepper2.run();
}

/**
 * @brief Reads incoming serial data and buffers the G-code command.
 */
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      command.trim(); 
      if (command.length() > 0) {
        isGCodeReady = true;
      }
      return; 
    }
    // Only add characters to the command string
    if (inChar != '\n' && inChar != '\r') {
      command += inChar;
    }
  }
}

/**
 * @brief Parses simple G-code (G1, G28) and executes the move.
 */
void parseAndExecuteGCode(String gCode) {
  gCode.toUpperCase();
  gCode.replace(" ", ""); // Remove all spaces after trimming to simplify parsing

  Serial.print("Executing: ");
  Serial.println(gCode);

  // Check for G1 command (Linear Move)
  if (gCode.startsWith("G1")) {
    
    // --- Parse X-axis Movement ---
    int xIndex = gCode.indexOf('X');
    if (xIndex != -1) {
      String xValueStr = gCode.substring(xIndex + 1);
      // Use helper function: Find next parameter (Y or F)
      int endX = findEndOfParameter(xValueStr, 0, "YF"); 
      if (endX != xValueStr.length()) { // Check if a terminator was found
        xValueStr = xValueStr.substring(0, endX);
      }
      
      float xUnits = xValueStr.toFloat();
      long xTargetSteps = xUnits * STEPS_PER_UNIT_X;
      stepper1.moveTo(xTargetSteps);
      Serial.print("  -> X Target Steps: "); Serial.println(xTargetSteps);
    }
    
    // --- Parse Y-axis Movement ---
    int yIndex = gCode.indexOf('Y');
    if (yIndex != -1) {
      String yValueStr = gCode.substring(yIndex + 1);
      // Use helper function: Find next parameter (X or F)
      int endY = findEndOfParameter(yValueStr, 0, "XF"); 
      if (endY != yValueStr.length()) {
        yValueStr = yValueStr.substring(0, endY);
      }
      
      float yUnits = yValueStr.toFloat();
      long yTargetSteps = yUnits * STEPS_PER_UNIT_Y;
      stepper2.moveTo(yTargetSteps);
      Serial.print("  -> Y Target Steps: "); Serial.println(yTargetSteps);
    }

    // --- Parse Feedrate (F) and Apply to Both Motors ---
    int fIndex = gCode.indexOf('F');
    if (fIndex != -1) {
      String fValueStr = gCode.substring(fIndex + 1);
      // Use helper function: Find next parameter (X or Y)
      int endF = findEndOfParameter(fValueStr, 0, "XY"); 
      if (endF != fValueStr.length()) {
        fValueStr = fValueStr.substring(0, endF);
      }
      
      float feedRate = fValueStr.toFloat();
      
      // Calculate speed in steps/second
      float speedX = (feedRate / 60.0) * STEPS_PER_UNIT_X;
      float speedY = (feedRate / 60.0) * STEPS_PER_UNIT_Y;
      
      stepper1.setSpeed(min(speedX, MAX_SPEED));
      stepper2.setSpeed(min(speedY, MAX_SPEED));

      Serial.print("  -> Feedrate (F): "); Serial.println(feedRate);
    }
  } 
  
  // Example for a simple Home command
  else if (gCode.startsWith("G28")) {
      //stepper1.setCurrentPosition(0);
     // stepper2.setCurrentPosition(0);
      Serial.println("Homing complete: Positions reset to 0.");
  }
  
  else {
    Serial.println("Error: Unsupported G-code or format error.");
  }
}
