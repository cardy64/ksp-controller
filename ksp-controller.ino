#include <krpc.h>
#include <krpc/services/krpc.h>
#include <krpc/services/space_center.h>

#define SAS_CONTROL_PIN 7
#define STAGE_PIN 8
#define STAGE_SAFETY_PIN 9
#define STAGE_LED_PIN 10
#define YAW_PIN A1
#define PITCH_PIN A2
#define ROLL_PIN A3
#define THROTTLE_PIN A4

HardwareSerial *conn;
krpc_SpaceCenter_Control_t instance;
krpc_SpaceCenter_Vessel_t vessel;
krpc_SpaceCenter_Control_t control;

bool previousStage = false;

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

void setup() {
  krpc_error_t error;
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(SAS_CONTROL_PIN, INPUT_PULLUP);
  pinMode(STAGE_PIN, INPUT_PULLUP);
  pinMode(STAGE_SAFETY_PIN, INPUT_PULLUP);
  pinMode(STAGE_LED_PIN, OUTPUT);
  digitalWrite(STAGE_LED_PIN, LOW);

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
  
  if (sas) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  }

  // Stage.
  bool stageSafetyOff = true; // digitalRead(STAGE_SAFETY_PIN) == LOW;
  //digitalWrite(STAGE_LED_PIN, stageSafetyOff);
  bool stage = digitalRead(STAGE_PIN) == LOW;
  if (stage && !previousStage && stageSafetyOff) {
      krpc_SpaceCenter_Control_ActivateNextStage(conn, NULL, control);
  }
  previousStage = stage;

  /*
  // Read throttle.
  int throttle = analogRead(0);
  krpc_SpaceCenter_Control_set_Throttle(conn, control, throttle/1023.0);
  */

  int yaw = analogRead(YAW_PIN);
  krpc_SpaceCenter_Control_set_Yaw(conn, control, -(yaw/512.0f - 1.0f));

  int pitch = analogRead(PITCH_PIN);
  krpc_SpaceCenter_Control_set_Pitch(conn, control, -(pitch/512.0f - 1.0f));

  int roll = analogRead(ROLL_PIN);
  krpc_SpaceCenter_Control_set_Roll(conn, control, -(roll/512.0f - 1.0f));

  int throttle = analogRead(THROTTLE_PIN);
  if (throttle < 10) {
    throttle = 0;
  }
  if (throttle > 1015) {
    throttle = 1023;    
  }
  krpc_SpaceCenter_Control_set_Throttle(conn, control, throttle/1023.0f);

  delay(100);
}
