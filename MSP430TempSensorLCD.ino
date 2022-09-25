
/* -----------------------------------------------------------------
    MSP430 Remote Temp Sensor with built-in LaunchPad LCD Display
    https://github.com/Andy4495/MSP430TempSensorLCD

    01/17/2018 - Andy4495 - Initial version
    09/24/2022 - Andy4495 - Updates for publishing to GitHub

  -----------------------------------------------------------------
   Requires CC110L BoosterPack
   Makes use of the built-in LCD available on FR4133 and FR6989 BoosterPacks.
   Conditional compilation allows code to work with other MSP430 variants.

   Also supports Fuel Tank I BoosterPack

   Collect and send the following data:
   - MSP430: Internal Temperature                F * 10
   - MSP430: Supply voltage (Vcc)                mV
   - Internal Timing:
                  # of times loop() has run
                  If FUEL_TANK_ENABLED, the millis parameter contains the Fuel Tank's StandbyTimeToEmpty value
                  Else millis parameter contains current value of millis()
*/

/*
    External libraries:
    - MspTandV by Andy4495 https://github.com/Andy4495/MspTandV
    - SWI2C by Andy4495 https://github.com/Andy4495/SWI2C
*/

// If using the Fuel Tank BoosterPack (Version 1, not Version 2),
// then uncomment the following line which will then send
// voltage levels from the Fuel Tank and not directly from Vcc.
// Fuel tank shuts down at ~3.65 LiPo voltage
#define FUEL_TANK_ENABLED

// Comment out following line if testing without CC110L BoosterPack
#define RADIO_ENABLED

// Define the pin used to set the Radio ID address
#define ADDRESS_PIN 11

#if defined(__MSP430FR4133__)
#define LCD_ENABLED
#endif

#if defined(__MSP430FR6989__)
#define LCD_ENABLED
#endif

#include <SPI.h>
#include <AIR430BoostFCC.h>
#include "MspTandV.h"
#include "SWI2C.h"
#ifdef LCD_ENABLED
#include "LCD_Launchpad.h"
#endif


// Fuel Tank Register Addresses
#define BQ27510_Control                     0x00
#define BQ27510_Temperature                 0x06
#define BQ27510_Voltage                     0x08
#define BQ27510_Flags                       0x0a
#define BQ27510_NominalAvailableCapacity    0x0c
#define BQ27510_FullAvailableCapacity       0x0e
#define BQ27510_RemainingCapacity           0x10
#define BQ27510_FullChargeCapacity          0x12
#define BQ27510_AverageCurrent              0x14
#define BQ27510_TimeToEmpty                 0x16
#define BQ27510_G3_StandbyCurrent           0x18
#define BQ27510_G3_StandbyTimeToEmpty       0x1a
#define BQ27510_G3_StateOfHealth            0x1c
#define BQ27510_G3_CycleCount               0x1e
#define BQ27510_G3_StateOfCharge            0x20
#define BQ27510_G3_InstantaneousCurrent     0x22
#define BQ27510_G3_InternalTemperature      0x28
#define BQ27510_G3_ResistanceScale          0x2a
#define BQ27510_G3_OperationConfiguration   0x2c
#define BQ27510_G3_DesignCapacity           0x2e

// CC110L Declarations
#define ADDRESS_REMOTE  0x01  // Receiver hub address
int     TxRadioID;            // Transmitter address

enum {WEATHER_STRUCT, TEMP_STRUCT};

struct SensorData {
  int             MSP_T;     // Tenth degrees F
  unsigned int    Batt_mV;   // milliVolts
  unsigned int    Loops;
  unsigned long   Millis;
};

struct sPacket
{
  uint8_t from;           // Local node address that message originated from
  uint8_t struct_type;    // Flag to indicate type of message structure
  union {
    uint8_t message[20];    // Local node message [MAX. 58 bytes]
    SensorData sensordata;
  };
};

struct sPacket txPacket;

MspTemp myTemp;
MspVcc  myVcc;

#define SDA_PIN 10
#define SCL_PIN 9
#define FUEL_TANK_ADDR  0x55

#ifdef FUEL_TANK_ENABLED
SWI2C myFuelTank = SWI2C(SDA_PIN, SCL_PIN, FUEL_TANK_ADDR);
uint16_t  data16;
uint8_t   data8;
#endif

#ifdef LCD_ENABLED
LCD_LAUNCHPAD myLCD;
#endif

unsigned int loopCount = 0;
const unsigned long sleepTime = 71234UL;


long           msp430T;
unsigned long  msp430mV;

void setup() {

  Serial.begin(9600);
  myFuelTank.begin();
  Serial.println("Reset");

#ifdef LCD_ENABLED
  myLCD.init();
#endif

  // Determine the TX radio ID
  // Leave pin high for address 4 (INPUT_PULLUP pulls signal high)
  // Ground pin for address 5
  pinMode(ADDRESS_PIN, INPUT_PULLUP);
  if (digitalRead(ADDRESS_PIN) == LOW)
  {
    TxRadioID = 5;
    pinMode(ADDRESS_PIN, INPUT); // If LOW, then pin is connected to GND, so change to INPUT to decrease current drain from uA to nA
  }
  else
    TxRadioID = 4;
    // If HIGH, then pin is unconnected, so leave as INPUT_PULLUP so it isn't floating

#ifdef PUSH1
  pinMode(PUSH1, INPUT_PULLUP);
#endif

  myTemp.read();
  myVcc.read();

  // CC110L Setup
  txPacket.from = TxRadioID;
  txPacket.struct_type = TEMP_STRUCT;
  memset(txPacket.message, 0, sizeof(txPacket.message));
#ifdef RADIO_ENABLED
  Radio.begin(TxRadioID, CHANNEL_1, POWER_MAX);
#endif

  Serial.print("CC110L Device ID: ");
  Serial.println(TxRadioID);
  Serial.println(" ");

  Serial.println("On-chip Calibration Memory Location and Data");
  Serial.println("--------------------------------------------");
  Serial.print("CAL_ADC_T30:         @");
  Serial.print((int)ADC_CAL_T30, HEX);
  Serial.print("          ");
  Serial.println(*(int*)ADC_CAL_T30);
  Serial.print("CAL_ADC_T85:         @");
  Serial.print((int)ADC_CAL_T85, HEX);
  Serial.print("          ");
  Serial.println(*(int*)ADC_CAL_T85);
  Serial.print("CAL_ADC_REF1_FACTOR: @");
  Serial.print((int)ADC_CAL_REF1_FACTOR, HEX);
  Serial.print("          ");
  Serial.println(*(unsigned int*)ADC_CAL_REF1_FACTOR);
  Serial.print("CAL_ADC_REF2_FACTOR: @");
  Serial.print(ADC_CAL_REF2_FACTOR, HEX);
  Serial.print("          ");
  Serial.println(*(unsigned int*)ADC_CAL_REF2_FACTOR);
  Serial.print("CAL_ADC_GAIN_FACTOR: @");
  Serial.print((int)ADC_CAL_GAIN_FACTOR, HEX);
  Serial.print("          ");
  Serial.println(*(unsigned int*)ADC_CAL_GAIN_FACTOR);
  Serial.print("CAL_ADC_OFFSET:      @");
  Serial.print((int)ADC_CAL_OFFSET_FACTOR, HEX);
  Serial.print("          ");
  Serial.println(*(int*)ADC_CAL_OFFSET_FACTOR);
  Serial.println("----------------");
  Serial.println(" ");

#ifdef FUEL_TANK_ENABLED
  Serial.println("Fuel Tank Info");
  Serial.println("--------------");
  Serial.print("Temp degrees (F*10):                 ");
  myFuelTank.read2bFromRegister(BQ27510_Temperature, &data16);
  Serial.println(((int)data16 - 2730) * 9 / 5 + 32);
  Serial.print("Voltage (mV):                     ");
  myFuelTank.read2bFromRegister(BQ27510_Voltage, &data16);
  Serial.println(data16);
  Serial.print("Nominal Available Capacity (mAh): ");
  myFuelTank.read2bFromRegister(BQ27510_NominalAvailableCapacity, &data16);
  Serial.println(data16);
  Serial.print("Full Available Capacity (mAh):    ");
  myFuelTank.read2bFromRegister(BQ27510_FullAvailableCapacity, &data16);
  Serial.println(data16);
  Serial.print("Remaining Capacity (mAh):         ");
  myFuelTank.read2bFromRegister(BQ27510_RemainingCapacity, &data16);
  Serial.println(data16);
  Serial.print("Full Charge Capacity (mAh):       ");
  myFuelTank.read2bFromRegister(BQ27510_FullChargeCapacity, &data16);
  Serial.println(data16);
  Serial.print("Standby Time to Empty (min):              ");
  myFuelTank.read2bFromRegister(BQ27510_G3_StandbyTimeToEmpty, &data16);
  Serial.println(data16);
  Serial.print("Flags:                          0x");
  myFuelTank.read2bFromRegister(BQ27510_Flags, &data16);
  Serial.println(data16, HEX);
  Serial.println(" ");
#endif
}


void loop() {

  loopCount++;

  myTemp.read();
  myVcc.read();

  txPacket.sensordata.MSP_T = myTemp.getTempCalibratedF();
#ifdef FUEL_TANK_ENABLED
  myFuelTank.read2bFromRegister(BQ27510_Voltage, &data16);
  txPacket.sensordata.Batt_mV = data16;
#else
  txPacket.sensordata.Batt_mV = myVcc.getVccCalibrated();
#endif

  txPacket.sensordata.Loops = loopCount;
#ifdef FUEL_TANK_ENABLED
  myFuelTank.read2bFromRegister(BQ27510_G3_StandbyTimeToEmpty, &data16);
  txPacket.sensordata.Millis = data16;
#else
  txPacket.sensordata.Millis = millis();
#endif

  // Send the data over-the-air
#ifdef RADIO_ENABLED
  Radio.transmit(ADDRESS_REMOTE, (unsigned char*)&txPacket, sizeof(txPacket.sensordata) + 4);
#endif

#ifdef LCD_ENABLED
  myLCD.clear();
  delay(100);
#ifdef FUEL_TANK_ENABLED
#ifdef PUSH1
  if (digitalRead(PUSH1) == LOW) {
    myLCD.print("Hr ");
    myLCD.showSymbol(LCD_SEG_COLON2, true);
    myLCD.print(txPacket.sensordata.Millis / 60);         // Millis contains TimeToEmpty info when using FuelTank
    delay(3000);
    myLCD.clear();
  }
#endif
#endif
  myLCD.print(" ");
  myLCD.print(TxRadioID);
  myLCD.print(" ");
  myLCD.showSymbol(LCD_SEG_COLON2, true);
  myLCD.print(txPacket.sensordata.MSP_T);
  myLCD.showSymbol(LCD_SEG_DOT5, true);
  displayBattOnLCD(txPacket.sensordata.Batt_mV);
#endif

  Serial.println("           Calibrated    Uncalibrated");
  Serial.println("           ----------    ------------");
  Serial.print  ("Temp (C)       ");
  Serial.print(myTemp.getTempCalibratedC());
  Serial.print("           ");
  Serial.println(myTemp.getTempUncalibratedC());
  Serial.print("Temp (F)       ");
  Serial.print(myTemp.getTempCalibratedF());
  Serial.print("           ");
  Serial.println(myTemp.getTempUncalibratedF());
  Serial.print("Vcc  (mV)     ");
  Serial.print(myVcc.getVccCalibrated());
  Serial.print("          ");
  Serial.println(myVcc.getVccUncalibrated());
#ifdef FUEL_TANK_ENABLED
  Serial.print("FuelTank (mV) ");
  Serial.println(txPacket.sensordata.Batt_mV);
  Serial.print("Tank Rmn (m)  ");
  Serial.println(txPacket.sensordata.Millis);    // Millis holds the time remaining value when using FuelTank
#endif
  Serial.println(" ");

  sleep(sleepTime);
}

#ifdef LCD_ENABLED

#ifdef FUEL_TANK_ENABLED
void displayBattOnLCD(int mV) {
  if (mV > 4100) myLCD.showSymbol(LCD_SEG_BAT5, 1);
  if (mV > 4040) myLCD.showSymbol(LCD_SEG_BAT4, 1);
  if (mV > 3980) myLCD.showSymbol(LCD_SEG_BAT3, 1);
  if (mV > 3920) myLCD.showSymbol(LCD_SEG_BAT2, 1);
  if (mV > 3860) myLCD.showSymbol(LCD_SEG_BAT1, 1);
  if (mV > 3800) myLCD.showSymbol(LCD_SEG_BAT0, 1);
  myLCD.showSymbol(LCD_SEG_BAT_ENDS, 1);
  myLCD.showSymbol(LCD_SEG_BAT_POL, 1);
}
#else
void displayBattOnLCD(int mV) {
  if (mV > 3200) myLCD.showSymbol(LCD_SEG_BAT5, 1);
  if (mV > 3000) myLCD.showSymbol(LCD_SEG_BAT4, 1);
  if (mV > 2800) myLCD.showSymbol(LCD_SEG_BAT3, 1);
  if (mV > 2600) myLCD.showSymbol(LCD_SEG_BAT2, 1);
  if (mV > 2400) myLCD.showSymbol(LCD_SEG_BAT1, 1);
  if (mV > 2200) myLCD.showSymbol(LCD_SEG_BAT0, 1);
  myLCD.showSymbol(LCD_SEG_BAT_ENDS, 1);
  myLCD.showSymbol(LCD_SEG_BAT_POL, 1);
}
#endif

#endif
