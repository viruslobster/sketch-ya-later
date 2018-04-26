#include <EEPROM.h>
#include <XYSteppers.h>

#define BAUD_R 9600
#define POINT_ARR_MAX 32
#define UI_SIZE 2
#define MAX_TIME 120000
#define BREAK_TIME 30000

typedef struct point {
  unsigned int x;
  unsigned int y; 
} point;

enum readiness {READY, NOT_READY};

point point_arr[POINT_ARR_MAX];

int rec = 0;

int prev_state = NOT_READY;

long lastStart = 0;
bool firstPoint = true;

#include <XYSteppers.h>

XYSteppers steppers(800, 4, 5, 6, 7, 8, 9, 10, 11);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_R);
  steppers.idleMotors();
  
  pinMode(LED_BUILTIN, OUTPUT);

  for (int i; i < POINT_ARR_MAX; i++)
    point_arr[i] = {.x = 0,.y = 0};
}

void loop() {
  // put your main code here, to run repeatedly:
  if (prev_state == NOT_READY) {
    Serial.println("ready");
    prev_state = READY;
  }
  
  byte temp_arr[POINT_ARR_MAX * 2 * UI_SIZE];
  for (int i; i < POINT_ARR_MAX * 2 * UI_SIZE; i++)
    temp_arr[i] = 0;

  if(Serial.available() > 0) {
    prev_state = NOT_READY;
    rec++;
    Serial.readBytes(temp_arr, POINT_ARR_MAX * 2 * UI_SIZE);

    for (int i = 0; i < POINT_ARR_MAX; i++) {
      point temp_p = {.x = 0,.y = 0};
      for (int j = 0; j < 2; j++) {
        unsigned int temp_int = 0; // calc current int
        for (int k = 0; k < UI_SIZE; k++) {
          if (k == 0)
            temp_int += temp_arr[4*i+2*j+k];
          else
            temp_int += temp_arr[4*i+2*j+k] << 8;
        }
        // put into point
        if (j == 0) {
          temp_p.x = temp_int;
        } else {
          temp_p.y = temp_int;
        }
      }

      point_arr[i] = temp_p;
    }

  for (int i = 0; i < POINT_ARR_MAX; i++) {
      //print_point(i);
      if (firstPoint) {
        lastStart = millis();
        firstPoint = false;
      }
      if (millis() - lastStart > MAX_TIME) {
        delay(BREAK_TIME);
        lastStart = millis();
      }
      steppers.moveToPoint(point_arr[i].x, point_arr[i].y); 
    }
  }
  steppers.idleMotors(); 
}

void print_point(int p_index) {
}

int process_points() {
  delay(100);
  return 1;
}

