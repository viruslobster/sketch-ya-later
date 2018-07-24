//Include the Wire I2C Library
#include <Wire.h>

/*This address is determined by the way your address pins are wired.
In the diagram from earlier, we connected A0 and A1 to Ground and 
A2 to 5V. To get the address, we start with the control code from 
the datasheet (1010) and add the logic state for each address pin
in the order A2, A1, A0 (100) which gives us 0b1010100, or in 
Hexadecimal, 0x54*/

#define EEPROM_ADR 0x54

/*Theoretically, the 24LC256 has a 64-byte page write buffer but 
we'll write 16 at a time to be safe*/ 

#define MAX_I2C_WRITE 16 

byte tempStore[MAX_I2C_WRITE];

void setup()
{
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
  
//Start the I2C Library
  Wire.begin();
  Wire.setClock(400000);

//Start the serial port
  Serial.begin(9600);

//Here is where we'll keep track of where in the memory we're writing
  long currentSpot = 0;
  long timerReset = 0;
  byte counter = 0;

  //Here we listen for bytes on the serial port and increment
  //the counter as we store them in our tempStore variable
  while (1)
  {
    while (Serial.available())
    {
      tempStore[counter++] = Serial.read(); //Read this byte into the array
      Serial.print((unsigned int)(currentSpot + counter-1));
      Serial.print("\t");
      Serial.println((unsigned int)(tempStore[counter-1]));

      if (counter == MAX_I2C_WRITE)
      {
      //Once we've collected a page worth, go ahead and do 
      //a page write operation
        writeEEPROMPage(currentSpot);
        counter = 0; //Reset
        currentSpot += MAX_I2C_WRITE;
      }

      timerReset = millis();
    }

    if (millis() - timerReset > 2000 && timerReset > 0)
    {
      Serial.println(currentSpot);
      timerReset = millis();
      writeEEPROMPage(currentSpot);
      break;
    }
  }

  Serial.print("Done! Stored ");
  Serial.print(currentSpot + counter);
  Serial.println(" bytes");
}

void loop()
{
    // Don't do anything here
  digitalWrite(LED_BUILTIN,HIGH);
}

/* This is the 3 step memory writing procedure that
we talked about. First we send the MSB of the address. 
Then we send the LSB of the address. Then we send the 
data that we want to store. */

void writeEEPROMPage(long eeAddress)
{

  Wire.beginTransmission(EEPROM_ADR);

  Wire.write((int)(eeAddress >> 8)); // MSB
  Wire.write((int)(eeAddress & 0xFF)); // LSB

  //Write bytes to EEPROM
  for (byte x = 0 ; x < MAX_I2C_WRITE ; x++)
    Wire.write(tempStore[x]); //Write the data

  Wire.endTransmission(); //Send stop condition
}
