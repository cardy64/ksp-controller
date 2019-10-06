#include <krpc.h>
#include <krpc/services/krpc.h>
#include <krpc/services/space_center.h>

#define SAS_CONTROL_PIN 7
#define STAGE_PIN 8

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

void blink_forever() {
  while (true) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

void setup() {
  krpc_error_t error;
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(SAS_CONTROL_PIN, INPUT_PULLUP);
  pinMode(STAGE_PIN, INPUT_PULLUP);

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

/*
  // Stage.
  bool stage = digitalRead(STAGE_PIN) == LOW;
  if (stage && !previousStage) {
      krpc_SpaceCenter_Control_ActivateNextStage(conn, NULL, control);
  }
  previousStage = stage;

  // Read throttle.
  int throttle = analogRead(0);
  krpc_SpaceCenter_Control_set_Throttle(conn, control, throttle/1023.0);
  */
  delay(100);
}
