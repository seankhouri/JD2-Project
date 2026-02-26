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
  // Configure GPIO, UART, drivers, timers, etc.
  // Return true on success, false on failure.
  // --------------------------------------------------
  // TODO: put real init code here
  // --------------------------------------------------
  return true;
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
  // Setup motion planning, timers, step generation, etc.
  // --------------------------------------------------
  // TODO: implement motion start
  // --------------------------------------------------
}

void updateMotion() {
  // Called repeatedly in EXECUTE to advance motion.
  // Non-blocking: just do one step or small chunk of work.
  // --------------------------------------------------
  // TODO: step generation / trajectory update
  // --------------------------------------------------
}

bool motionComplete() {
  // Return true when the current motion is finished.
  // --------------------------------------------------
  // TODO: implement real completion check
  // --------------------------------------------------
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
  Serial.begin(115200);

  pinMode(PIN_ESTOP,      INPUT_PULLUP);
  pinMode(PIN_BTN_START,  INPUT_PULLUP);
  pinMode(PIN_BTN_PAUSE,  INPUT_PULLUP);
  pinMode(PIN_BTN_RESUME, INPUT_PULLUP);
  pinMode(PIN_BTN_CANCEL, INPUT_PULLUP);
  pinMode(PIN_BTN_RESET,  INPUT_PULLUP);

  // TODO: configure step/dir pins, limit switches, etc.

  enterState(STATE_INIT);
}

void loop() {
  // 1) Global emergency-stop check (from ANY state)
  if (estopPressed() && currentState != STATE_EMERGENCY_STOP) {
    enterState(STATE_EMERGENCY_STOP);
  }

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
      // Check for incoming command
      MotionCommand cmd = readCommand();
      if (cmd.valid) {
        if (!commandIsValid(cmd)) {
          enterState(STATE_ERROR);
        } else if (commandIsMotion(cmd)) {
          currentCommand = cmd;
          enterState(STATE_EXECUTE);
        } else {
          // Non-motion commands can be handled here
          // (e.g., status request) without leaving IDLE
        }
      }
      break;
    }

    // ------------------ S3: EXECUTE -------------------
    case STATE_EXECUTE: {
      updateMotion();

      if (motionFault()) {
        enterState(STATE_ERROR);
      } else if (pauseRequested()) {
        enterState(STATE_PAUSE);
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
      // Stay here until user resets and error conditions are cleared
      if (resetRequested() && errorCleared()) {
        enterState(STATE_HOME);   // re-home after error
      }
      break;
    }

    // ------------------ S6: EMERGENCY STOP -----------
    case STATE_EMERGENCY_STOP: {
      // Remain here while e-stop is pressed
      if (!estopPressed() && resetRequested()) {
        // After e-stop is cleared + reset is requested, go home
        enterState(STATE_HOME);
      }
      break;
    }
  }

  // Small delay to avoid spamming Serial / bouncing buttons too fast
  delay(5);
}