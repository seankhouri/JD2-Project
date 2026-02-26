// ------------------------------------------------------------
// Sean Khouri
// JD2 Project 
// Two Axis Robotic Arm
// MCU BLOCK
// 
// ------------------------------------------------------------
// States:
//   S0: INIT             -> STATE_INIT
//   S1: HOME             -> STATE_HOME
//   S2: IDLE             -> STATE_IDLE
//   S3: EXECUTE          -> STATE_EXECUTE
//   S4: PAUSE            -> STATE_PAUSE
//   S5: ERROR            -> STATE_ERROR
//   S6: EMERGENCY STOP   -> STATE_EMERGENCY_STOP
// ------------------------------------------------------------

#include <Arduino.h>

// ---------------------- Pins / Inputs -----------------------


// Joint 1 (U1 in schematic)
const int PIN_J1_EN    = 25;   // EN
const int PIN_J1_STEP  = 26;   // STEP
const int PIN_J1_DIR   = 27;   // DIR
const int PIN_J1_LIMIT = 32;   // limit switch  (add later)

// Joint 2 (U2 in schematic)
const int PIN_J2_EN    = 14;   // EN
const int PIN_J2_STEP  = 12;   // STEP
const int PIN_J2_DIR   = 13;   // DIR
const int PIN_J2_LIMIT = 33;   // limit switch (add later)

const int PIN_ESTOP        = 4;   // active LOW emergency stop
const int PIN_BTN_START    = 5;   // start / send motion command (example)
const int PIN_BTN_PAUSE    = 18;  // pause
const int PIN_BTN_RESUME   = 19;  // resume
const int PIN_BTN_CANCEL   = 21;  // cancel / abort
const int PIN_BTN_RESET    = 22;  // reset after error / e-stop

// TODO: add your step/dir, UART to TMC2209, limit switch pins, etc.

// ---------------------- FSM Definition ----------------------

enum FsmState {
  STATE_INIT = 0,          // S0
  STATE_HOME,              // S1
  STATE_IDLE,              // S2
  STATE_EXECUTE,           // S3
  STATE_PAUSE,             // S4
  STATE_ERROR,             // S5
  STATE_EMERGENCY_STOP     // S6
};

FsmState currentState = STATE_INIT;

// Keep a simple structure for motion commands
struct MotionCommand {
  bool valid;
  // TODO: add fields like joint angles, speeds, etc.
};

MotionCommand currentCommand;

// ---------------------- Motion State (J1 only for now) -----
const long J1_STEPS_PER_MOVE      = 1000;        // steps per test move
const unsigned long STEP_INTERVAL_US = 1000;     // 1 kHz step rate

long j1_steps_done = 0;
unsigned long last_step_time_us = 0;
bool motion_active = false;

// ---------------------- GUI <-> MCU G-code Test ------------
const long GUI_BAUD = 115200;
const int GUI_MAX_LEN = 50;   // interface property: max G-code line length
String gui_line = "";         // stores one incoming line from GUI

// ---------------------- Helpers: Inputs ---------------------

bool estopPressed() {
  // Assume active LOW e-stop; change as needed
  return digitalRead(PIN_ESTOP) == LOW;
}

bool startRequested() {
  return digitalRead(PIN_BTN_START) == LOW;
}

bool pauseRequested() {
  return digitalRead(PIN_BTN_PAUSE) == LOW;
}

bool resumeRequested() {
  return digitalRead(PIN_BTN_RESUME) == LOW;
}

bool cancelRequested() {
  return digitalRead(PIN_BTN_CANCEL) == LOW;
}

bool resetRequested() {
  return digitalRead(PIN_BTN_RESET) == LOW;
}

// ---------------------- Hardware / Motion Stubs ------------
// You will fill these in with real code later. For now they
// just return dummy values so the FSM structure compiles.

bool initHardware() {
  // ----- Joint 1 -----
  pinMode(PIN_J1_STEP, OUTPUT);
  pinMode(PIN_J1_DIR,  OUTPUT);
  pinMode(PIN_J1_EN,   OUTPUT);
  pinMode(PIN_J1_LIMIT, INPUT_PULLUP);

  digitalWrite(PIN_J1_STEP, LOW);
  digitalWrite(PIN_J1_DIR,  LOW);

  // ----- Joint 2 -----
  pinMode(PIN_J2_STEP, OUTPUT);
  pinMode(PIN_J2_DIR,  OUTPUT);
  pinMode(PIN_J2_EN,   OUTPUT);
  pinMode(PIN_J2_LIMIT, INPUT_PULLUP);

  digitalWrite(PIN_J2_STEP, LOW);
  digitalWrite(PIN_J2_DIR,  LOW);

  // Enable both drivers at startup
  enableAllMotors();

  return true;
}

void singleStepJ1() {
  digitalWrite(PIN_J1_STEP, HIGH);
  delayMicroseconds(2);     // pulse width (TMC2209 only needs a few µs)
  digitalWrite(PIN_J1_STEP, LOW);
}

void enableAllMotors() {
  digitalWrite(PIN_J1_EN, LOW);   // LOW = enabled for TMC2209
  digitalWrite(PIN_J2_EN, LOW);
}

void disableAllMotors() {
  digitalWrite(PIN_J1_EN, HIGH);  // HIGH = disabled
  digitalWrite(PIN_J2_EN, HIGH);
  motion_active = false;          // stop any in-progress motion
}

void startHoming() {
  // Kick off homing sequence (non-blocking if possible).
  // Set any internal flags, starting positions, directions.
  // --------------------------------------------------
  // TODO: implement homing start
  // --------------------------------------------------
}

bool homingInProgress() {
  // Return true while homing is still running.
  // --------------------------------------------------
  // TODO: check limit switches / homing status
  // --------------------------------------------------
  return false;
}

bool homingSucceeded() {
  // Return true exactly once when all axes successfully homed.
  // --------------------------------------------------
  // TODO: implement homing success check
  // --------------------------------------------------
  return true;
}

bool homingFailed() {
  // Return true if homing times out or detects a fault.
  // --------------------------------------------------
  // TODO: implement homing fault detection
  // --------------------------------------------------
  return false;
}

MotionCommand readCommand() {
  MotionCommand cmd;
  cmd.valid = false;

  // Example placeholder: use a button press as "motion command"
  if (startRequested()) {
    cmd.valid = true;
    // TODO: parse real command from Serial / WiFi / etc.
  }

  return cmd;
}

bool commandIsValid(const MotionCommand &cmd) {
  // TODO: apply range checks, syntax checks, etc.
  return cmd.valid;
}

bool commandIsMotion(const MotionCommand &cmd) {
  // TODO: inspect cmd contents to see if it requires motion
  return cmd.valid;
}

void startMotion(const MotionCommand &cmd) {
  // For now: always move Joint 1 "forward" a fixed distance
  motion_active      = true;
  j1_steps_done      = 0;
  last_step_time_us  = micros();

  // Choose direction (flip LOW/HIGH if motor turns wrong way)
  digitalWrite(PIN_J1_DIR, LOW);
}

void updateMotion() {
  if (!motion_active) return;

  unsigned long now = micros();
  if (now - last_step_time_us >= STEP_INTERVAL_US) {
    last_step_time_us = now;

    // One step on J1
    singleStepJ1();
    j1_steps_done++;

    // Optional debug every 100 steps
    if (j1_steps_done % 100 == 0) {
      Serial.print("J1 steps: ");
      Serial.println(j1_steps_done);
    }
  }
}

bool motionComplete() {
  if (!motion_active) return false;

  if (j1_steps_done >= J1_STEPS_PER_MOVE) {
    motion_active = false;
    return true;
  }

  return false;
}

bool motionFault() {
  // Return true if driver error, unexpected limit, etc.
  // --------------------------------------------------
  // TODO: implement real motion fault detection
  // --------------------------------------------------
  return false;
}

bool errorCleared() {
  // Return true when fault condition is cleared and safe to reset.
  // For now, tie it to reset button as well.
  return resetRequested();
}

// ---------------------- GUI G-code Line Handler ------------

void gui_handle_line(const String &line) {
  if (line.length() == 0) {
    Serial.println("ERR:EMPTY");
    return;
  }

  // Extract first token (command) before first space
  int firstSpace = line.indexOf(' ');
  String cmd = (firstSpace == -1) ? line : line.substring(0, firstSpace);
  cmd.trim();  // remove \r or extra spaces

  bool supported =
      cmd == "G0"  || cmd == "G1"  ||
      cmd == "G90" || cmd == "G91" ||
      cmd == "G20" || cmd == "G21" ||
      cmd == "M2"  || cmd == "M6";

  if (supported) {
    Serial.print("OK:");
    Serial.print(cmd);
    Serial.print(" ");
    Serial.println(line);
  } else {
    Serial.print("ERR:UNKNOWN ");
    Serial.println(line);
  }
}

// ---------------------- State Entry Helpers ----------------

void enterState(FsmState newState) {
  // Optional: add per-state entry actions here
  switch (newState) {
    case STATE_INIT:
      Serial.println("Entering INIT");
      break;

    case STATE_HOME:
      Serial.println("Entering HOME");
      startHoming();
      break;

    case STATE_IDLE:
      Serial.println("Entering IDLE");
      // Optionally move arm to neutral "hover" pose here.
      break;

    case STATE_EXECUTE:
      Serial.println("Entering EXECUTE");
      startMotion(currentCommand);
      break;

    case STATE_PAUSE:
      Serial.println("Entering PAUSE");
      // TODO: stop step pulses but keep position context
      break;

    case STATE_ERROR:
      Serial.println("Entering ERROR");
      // TODO: disable motion, report error code
      break;

    case STATE_EMERGENCY_STOP:
      Serial.println("Entering EMERGENCY STOP");
      // TODO: immediately disable motor drivers
      break;
  }

  currentState = newState;
}

// ---------------------- Arduino Setup / Loop ---------------

void setup() {
  Serial.begin(GUI_BAUD);
  // Optional: wait a moment for Serial to be ready on some boards
  delay(100);
  Serial.println("MCU READY (gui_mcu_comm test)");

  pinMode(PIN_ESTOP,      INPUT_PULLUP);
  pinMode(PIN_BTN_START,  INPUT_PULLUP);
  pinMode(PIN_BTN_PAUSE,  INPUT_PULLUP);
  pinMode(PIN_BTN_RESUME, INPUT_PULLUP);
  pinMode(PIN_BTN_CANCEL, INPUT_PULLUP);
  pinMode(PIN_BTN_RESET,  INPUT_PULLUP);

  enterState(STATE_INIT);
}

void loop() {
  // ---- GUI G-code listener (runs in all states) ----
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\r') {
      continue;  // ignore CR
    }

    if (c == '\n') {
      // End of line
      if (gui_line.length() > 0) {
        if (gui_line.length() <= GUI_MAX_LEN) {
          gui_handle_line(gui_line);
        } else {
          Serial.print("ERR:TOO_LONG ");
          Serial.println(gui_line);
        }
        gui_line = "";
      }
    } else {
      // Add character to line buffer
      gui_line += c;

      // Hard cap to avoid growing without bound (not strictly needed)
      if (gui_line.length() > GUI_MAX_LEN + 10) {
        gui_line.remove(GUI_MAX_LEN + 10);
      }
    }
  }

  // ---- Global emergency-stop check (from ANY state) ----
  if (estopPressed() && currentState != STATE_EMERGENCY_STOP) {
    enterState(STATE_EMERGENCY_STOP);
  }

  // ---- Main FSM ----
  switch (currentState) {

    // ------------------ S0: INIT ----------------------
    case STATE_INIT: {
      static bool initDone = false;
      static bool initResult = false;

      if (!initDone) {
        initResult = initHardware();
        initDone = true;
      }

      if (initDone) {
        if (initResult) {
          enterState(STATE_HOME);
        } else {
          enterState(STATE_ERROR);
        }
      }
      break;
    }

    // ------------------ S1: HOME ----------------------
    case STATE_HOME: {
      // Non-blocking homing sequence
      if (homingInProgress()) {
        // keep homing
      } else if (homingFailed()) {
        enterState(STATE_ERROR);
      } else if (homingSucceeded()) {
        enterState(STATE_IDLE);
      }
      break;
    }

    // ------------------ S2: IDLE ----------------------
    case STATE_IDLE: {
      // For now, do nothing here.
      // Later you can check for new motion commands / buttons.
      break;
    }

    // ------------------ S3: EXECUTE -------------------
    case STATE_EXECUTE: {
      // Update motion and look for completion / faults
      updateMotion();

      if (motionFault()) {
        enterState(STATE_ERROR);
      } else if (motionComplete()) {
        enterState(STATE_IDLE);
      }
      break;
    }

    // ------------------ S4: PAUSE ---------------------
    case STATE_PAUSE: {
      if (motionFault()) {
        enterState(STATE_ERROR);
      } else if (resumeRequested()) {
        enterState(STATE_EXECUTE);
      } else if (cancelRequested()) {
        // Abort motion and return to idle
        enterState(STATE_IDLE);
      }
      break;
    }

    // ------------------ S5: ERROR ---------------------
    case STATE_ERROR: {
      disableAllMotors(); // kill power to motors on error
      // Stay here until user clears error
      if (errorCleared()) {
        enterState(STATE_INIT);
      }
      break;
    }

    // ------------- S6: EMERGENCY STOP ----------------
    case STATE_EMERGENCY_STOP: {
      disableAllMotors(); // immediate motor disable
      // Require reset after E-stop
      if (resetRequested()) {
        enterState(STATE_INIT);
      }
      break;
    }
  }

  // Small delay to avoid spamming Serial / bouncing buttons too fast
  delay(5);
}