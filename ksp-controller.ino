#include <krpc.h>
#include <krpc/services/krpc.h>
#include <krpc/services/space_center.h>

#include "max7219.h"

#define SAS_CONTROL_PIN 7
#define STAGE_PIN 8
#define STAGE_SAFETY_PIN 9
#define STAGE_LED_PIN 10
#define THRUST_1_PIN 22
#define THRUST_2_PIN 23
#define THRUST_PWM_PIN 2
#define SAS_BUTTON_PIN 27
#define SAS_LIGHT_PIN 28
#define GEAR_BUTTON_PIN 29
#define GEAR_LIGHT_PIN 30
#define BRAKES_BUTTON_PIN 31
#define BRAKES_LIGHT_PIN 32
#define RCS_BUTTON_PIN 33
#define RCS_LIGHT_PIN 34
#define ABORT_BUTTON_PIN 35
#define ABORT_LIGHT_PIN 36
#define ABORT_SAFETY_SWITCH_PIN 37
#define YAW_PIN A1
#define PITCH_PIN A2
#define ROLL_PIN A3
#define THROTTLE_PIN A4

HardwareSerial *conn;
krpc_SpaceCenter_Control_t instance;
krpc_SpaceCenter_Vessel_t vessel;
krpc_SpaceCenter_Control_t control;

bool previousStage = false;
int lastThrottleKsp = -1000;
int targetThrottle = 0;
unsigned long moveThrustEndTime = 0;
unsigned long updateTime = 0;
bool lastSasButton = false;
bool lastGearButton = false;
bool lastBrakesButton = false;
bool lastRcsButton = false;
bool lastAbortButton = false;

void blink_led(int count) {
  delay(1000);
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
  delay(1000);  
}

bool areClose(int a, int b, int tolerance) {
  int diff = a - b;
  if (diff < 0) {
    diff = -diff;
  }

  return diff <= tolerance;
}

void setup() {
  krpc_error_t error;
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(SAS_CONTROL_PIN, INPUT_PULLUP);
  pinMode(STAGE_PIN, INPUT_PULLUP);
  pinMode(STAGE_SAFETY_PIN, INPUT);
  pinMode(STAGE_LED_PIN, OUTPUT);
  digitalWrite(STAGE_LED_PIN, LOW);
  pinMode(SAS_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SAS_LIGHT_PIN, OUTPUT);
  pinMode(GEAR_BUTTON_PIN, INPUT_PULLUP);
  pinMode(GEAR_LIGHT_PIN, OUTPUT);
  pinMode(BRAKES_BUTTON_PIN, INPUT_PULLUP);
  pinMode(BRAKES_LIGHT_PIN, OUTPUT);
  pinMode(RCS_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RCS_LIGHT_PIN, OUTPUT);
  pinMode(ABORT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ABORT_LIGHT_PIN, OUTPUT);
  
  //setupLeds();

  conn = &Serial;
  delay(1000);

  do {
    // Open the serial port connection
    error = krpc_open(&conn, NULL);
    if (error != KRPC_OK) {
      delay(100);
    }
  } while (error != KRPC_OK);

  // Set up communication with the server
  do {
    error = krpc_connect(conn, "Milo's KSP Controller");
    if (error != KRPC_OK) {
      blink_led(-(int) error);
      delay(100);
    }
  } while (error != KRPC_OK);

  do {
    error = krpc_SpaceCenter_ActiveVessel(conn, &vessel);
    if (error != KRPC_OK) {
      delay(100);
    }
  } while (error != KRPC_OK);

  do {
    error = krpc_SpaceCenter_Vessel_Control(conn, &control, vessel);
    if (error != KRPC_OK) {
      delay(100);
      blink_led(- (int)error);
    }
  } while (error != KRPC_OK);  
}



void loop() {
  krpc_error_t error;
  unsigned long now = millis();

  boolean update = now >= updateTime;
  if (update) {
    updateTime = now + 100;
  }
  if (now < moveThrustEndTime) {
    update = false;
  }

  if (update) {
    bool sas;
    // Set SAS from switch.
   /// sas = digitalRead(SAS_CONTROL_PIN) == LOW;
   /// krpc_SpaceCenter_Control_set_SAS(conn, control, sas);
  
    // Get SAS for LED.
    if (false) {
      do {
        error = krpc_SpaceCenter_Control_SAS(conn, &sas, control);
        if (error != KRPC_OK) {
          delay(100);
        }
      } while (error != KRPC_OK);  
    }
  
    // Stage.
    bool stageSafetyOff = digitalRead(STAGE_SAFETY_PIN);
    digitalWrite(STAGE_LED_PIN, stageSafetyOff);
    bool stage = digitalRead(STAGE_PIN) == LOW;
    if (stage && !previousStage && stageSafetyOff) {
        krpc_SpaceCenter_Control_ActivateNextStage(conn, NULL, control);
    }
    previousStage = stage;
  
    int yaw = analogRead(YAW_PIN);
    krpc_SpaceCenter_Control_set_Yaw(conn, control, -(yaw/512.0f - 1.0f));
  
    int pitch = analogRead(PITCH_PIN);
    krpc_SpaceCenter_Control_set_Pitch(conn, control, -(pitch/512.0f - 1.0f));
  
    int roll = analogRead(ROLL_PIN);
    krpc_SpaceCenter_Control_set_Roll(conn, control, -(roll/512.0f - 1.0f));
  }

  // -----------------------------------------------------------------------------------------------
  // Thrust.

  if (update) {
    float newThrottleKspFloat;
    krpc_SpaceCenter_Control_Throttle(conn, &newThrottleKspFloat, control);
    int newThrottleKsp = newThrottleKspFloat*1023;
    if (!areClose(lastThrottleKsp, newThrottleKsp, 10)) {
      // KSP has changed the throttle.
      moveThrustEndTime = now + 500;
      lastThrottleKsp = newThrottleKsp;
      targetThrottle = newThrottleKsp;
    }
  }

  int throttlePosition = analogRead(THROTTLE_PIN);
  if (throttlePosition < 10) {
    throttlePosition = 0;
  }
  if (throttlePosition > 1015) {
    throttlePosition = 1023;    
  }

  if (now < moveThrustEndTime) {   
    if (throttlePosition > targetThrottle + 100) {
      digitalWrite(THRUST_1_PIN, HIGH);
      digitalWrite(THRUST_2_PIN, LOW);    
    } else if (throttlePosition < targetThrottle - 100) {
      digitalWrite(THRUST_1_PIN, LOW);
      digitalWrite(THRUST_2_PIN, HIGH);
    } else {
      digitalWrite(THRUST_1_PIN, LOW);
      digitalWrite(THRUST_2_PIN, LOW);          
    }    
  } else {
    if (update) {
      // TODO don't call this every time. It's slow and could update KSP wrongly. Instead only
      // update if we think that KSP is out of date with respect to slider
      lastThrottleKsp = throttlePosition;
      krpc_SpaceCenter_Control_set_Throttle(conn, control, throttlePosition/1023.0f);    
    }

    digitalWrite(THRUST_1_PIN, LOW);
    digitalWrite(THRUST_2_PIN, LOW);          
  }
  analogWrite(THRUST_PWM_PIN, 255);

  //loopLeds();

  // -----------------------------------------------------------------------------------------------
  // Buttons.
  if (update) {
    // SAS.
    bool sas;
    krpc_SpaceCenter_Control_SAS(conn, &sas, control);
    digitalWrite(SAS_LIGHT_PIN, sas);
    bool sasButton = !digitalRead(SAS_BUTTON_PIN);
    if (!lastSasButton && sasButton) {
      krpc_SpaceCenter_Control_set_SAS(conn, control, !sas); 
    }
    lastSasButton = sasButton;

    // Gear.
    bool gear;
    krpc_SpaceCenter_Control_Gear(conn, &gear, control);
    digitalWrite(GEAR_LIGHT_PIN, gear);
    bool gearButton = !digitalRead(GEAR_BUTTON_PIN);
    if (!lastGearButton && gearButton) {
      krpc_SpaceCenter_Control_set_Gear(conn, control, !gear);
    }
    lastGearButton = gearButton;

    // Brakes.
    bool brakes;
    krpc_SpaceCenter_Control_Brakes(conn, &brakes, control);
    digitalWrite(BRAKES_LIGHT_PIN, brakes);
    bool brakesButton = !digitalRead(BRAKES_BUTTON_PIN);
    if (!lastBrakesButton && brakesButton) {
      krpc_SpaceCenter_Control_set_Brakes(conn, control, !brakes); 
    }
    lastBrakesButton = brakesButton;

    // RCS.
    bool rcs;
    krpc_SpaceCenter_Control_RCS(conn, &rcs, control);
    digitalWrite(RCS_LIGHT_PIN, rcs);
    bool rcsButton = !digitalRead(RCS_BUTTON_PIN);
    if (!lastRcsButton && rcsButton) {
      krpc_SpaceCenter_Control_set_RCS(conn, control, !rcs); 
    }
    lastRcsButton = rcsButton;

    // Abort.
    bool abort;
    krpc_SpaceCenter_Control_Abort(conn, &abort, control);
    digitalWrite(ABORT_LIGHT_PIN, abort);
    bool abortButton = !digitalRead(ABORT_BUTTON_PIN);
    if (!lastAbortButton && abortButton) {
      krpc_SpaceCenter_Control_set_Abort(conn, control, !abort);
    }
    lastAbortButton = abortButton;
  }
}
