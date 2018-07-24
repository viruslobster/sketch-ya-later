#include <EEPROM.h>
#include <XYSteppers.h>
//#include <LiquidCrystal.h>
#include <Wire.h>

#define EEPROM_ADR 0x54
#define UI_SIZE 2
#define MAX_TIME 60000
#define BREAK_TIME 000
#define BLOCK_SIZE 512

//LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

long last_start;

XYSteppers steppers(200, 4, 5, 6, 7, 8, 9, 10, 11);

long start_index;
long end_index;
long curr_index;
long block_index;
byte block[BLOCK_SIZE];

void setup() {
  steppers.idleMotors();

  //lcd.begin(16,2);
  Serial.begin(19200);
  Wire.begin();
  Wire.setClock(400000);

  byte num_images = read_EEPROM(0L);
  byte im_index = read_EEPROM(1L);
  write_EEPROM(1L, (im_index + 1) % num_images);
  delay(500);
  long table_index = 2 * im_index + 2;
  start_index = (read_EEPROM(table_index) << 8) + read_EEPROM(table_index + 1);
  end_index = (read_EEPROM(table_index + 2) << 8) + read_EEPROM(table_index + 3);
  curr_index = 0;
  block_index = start_index;
  read_block(block_index);

  Serial.print("Drawing Image #");
  Serial.println(im_index);
  Serial.print("Start index: ");
  Serial.println(start_index);
  Serial.print("End index: ");
  Serial.println(end_index);
  Serial.print("Total points: ");
  Serial.println((end_index - start_index) / 2);
  delay(2500);
  last_start = millis();
}

void loop() {
  if (block_index + curr_index < end_index) {
    if (millis() - last_start > MAX_TIME) {
      /*while (millis() - last_start - MAX_TIME < BREAK_TIME) {
        print_break();
        }*/
      steppers.idleMotors();
      delay(BREAK_TIME);
      last_start = millis();
    }

    if (curr_index >= BLOCK_SIZE) {
      curr_index = 0;
      block_index += BLOCK_SIZE;
      read_block(block_index);
    }
  
    unsigned int x_coord = (block[curr_index] << 8) + block[curr_index + 1];
    unsigned int y_coord = (block[curr_index + 2] << 8) + block[curr_index + 3];
    steppers.moveToPoint(x_coord, y_coord);
    //delay(10)
    //print_point((int)x_coord, (int)y_coord);
    curr_index += 4;
  }
  else {
    steppers.moveToPoint(0,0);
    steppers.idleMotors();
    delay(1000);
  }
}

void print_break() {
  /*lcd.setCursor(0,1);
    lcd.print("Drawing in ");
    lcd.print((BREAK_TIME - millis() + lastStart + MAX_TIME) / 1000);*/
  Serial.write(27);       // ESC command
  Serial.print("[2J");    // clear screen command
  Serial.write(27);
  Serial.print("[H");     // cursor to home command
  Serial.print("Drawing in ");
  Serial.println((BREAK_TIME - millis() + last_start + MAX_TIME) / 1000);
}

void print_point(unsigned int x, unsigned int y) {
  /*lcd.clear();
    lcd.print("pt ");
    lcd.print((curr_index - start_index) / 4);
    lcd.print(": ");
    lcd.print(x);
    lcd.print(", ");
    lcd.print(y);
    lcd.setCursor(0,1);
    lcd.print("Break in ");
    lcd.print((MAX_TIME - millis() + lastStart) / 1000);*/
  Serial.write(27);       // ESC command
  Serial.print("[2J");    // clear screen command
  Serial.write(27);
  Serial.print("[H");     // cursor to home command
  Serial.print("pt ");
  Serial.print((curr_index - start_index) / 4);
  Serial.print(": ");
  Serial.print(x);
  Serial.print(", ");
  Serial.println(y);
  Serial.print("Break in ");
  Serial.println((last_start + MAX_TIME - millis()) / 1000);
}


byte read_EEPROM(long eeaddress)
{
  Wire.beginTransmission(EEPROM_ADR);

  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();

  Wire.requestFrom(EEPROM_ADR, 1);

  byte rdata = 0xFF;
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}

void read_block(long eeadress) {
  for (int i = 0; i < BLOCK_SIZE; i++) {
    block[i] = read_EEPROM(eeadress + i);
  }
}

void write_EEPROM(long eeAddress, byte data)
{

  Wire.beginTransmission(EEPROM_ADR);

  Wire.write((int)(eeAddress >> 8)); // MSB
  Wire.write((int)(eeAddress & 0xFF)); // LSB

  //Write bytes to EEPROM
  Wire.write(data); //Write the data

  Wire.endTransmission(); //Send stop condition
}
