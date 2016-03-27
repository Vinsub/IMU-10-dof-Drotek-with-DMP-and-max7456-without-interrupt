#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu(0x69);

#define HMC5883L_DEFAULT_ADDRESS    0x1E
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAZ_H         0x05
#define HMC5883L_RA_DATAY_H         0x07


bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
int16_t mx, my, mz;     //To store magnetometer readings
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float heading;          // Simple magnetic heading. (NOT COMPENSATED FOR PITCH AND ROLL)

byte compassCalibrated = 1;


double mxMin = 0;
double mxMax = 0;
double myMin = 0;
double myMax = 0;
double mzMin = 0;
double mzMax = 0;
double mxOffSet = 0;
double myOffSet = 0;
double mzOffSet = 0;

//=================================================================
//==                         OSD MAX7456                         ==
//=================================================================

// Included Libraries //////////////////////////////////////////////////////////

  #include <SPI.h>
  #include <MAX7456.h>

 const byte osdChipSelect = 10;
 const byte masterOutSlaveIn = 11;
 const byte masterInSlaveOut = 12;
 const byte slaveClock = 13;
 const byte osdReset = 0;
 
 MAX7456 OSD( osdChipSelect );
 // Timer
    char time[] = "00:00:00";
    char timeMillis[3];
    int msOffset = (timeMillis[3] - '0') * 100 + (timeMillis[2] - '0') * 10 + (time[1] - '0');
    
    bool toggle = false;
    int z = 0;
int x = 0; 
  // Fine Timer
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  
  //char compassOSD[] = {(char)1,(char)2,(char)3,(char)4};
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 12; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(200, true);
    #endif
    
    Serial.begin(115200);
    
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-6);
    mpu.setYGyroOffset(-35);
    mpu.setZGyroOffset(-19);
    mpu.setXAccelOffset(-3699);
    mpu.setYAccelOffset(1113);
    mpu.setZAccelOffset(1377); // 1688 factory default for my test chip
    
    // Magnetometer configuration

  mpu.setI2CMasterModeEnabled(0);
  mpu.setI2CBypassEnabled(1);

  Wire.beginTransmission(HMC5883L_DEFAULT_ADDRESS);
  Wire.write(0x02); 
  Wire.write(0x00);  // Set continuous mode
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(HMC5883L_DEFAULT_ADDRESS);
  Wire.write(0x00);
  Wire.write(B00011000);  // 75Hz
  Wire.endTransmission();
  delay(5);

  mpu.setI2CBypassEnabled(0);

  // X axis word
  mpu.setSlaveAddress(0, HMC5883L_DEFAULT_ADDRESS | 0x80); // 0x80 turns 7th bit ON, according to datasheet, 7th bit controls Read/Write direction
  mpu.setSlaveRegister(0, HMC5883L_RA_DATAX_H);
  mpu.setSlaveEnabled(0, true);
  mpu.setSlaveWordByteSwap(0, false);
  mpu.setSlaveWriteMode(0, false);
  mpu.setSlaveWordGroupOffset(0, false);
  mpu.setSlaveDataLength(0, 2);

  // Y axis word
  mpu.setSlaveAddress(1, HMC5883L_DEFAULT_ADDRESS | 0x80);
  mpu.setSlaveRegister(1, HMC5883L_RA_DATAY_H);
  mpu.setSlaveEnabled(1, true);
  mpu.setSlaveWordByteSwap(1, false);
  mpu.setSlaveWriteMode(1, false);
  mpu.setSlaveWordGroupOffset(1, false);
  mpu.setSlaveDataLength(1, 2);

  // Z axis word
  mpu.setSlaveAddress(2, HMC5883L_DEFAULT_ADDRESS | 0x80);
  mpu.setSlaveRegister(2, HMC5883L_RA_DATAZ_H);
  mpu.setSlaveEnabled(2, true);
  mpu.setSlaveWordByteSwap(2, false);
  mpu.setSlaveWriteMode(2, false);
  mpu.setSlaveWordGroupOffset(2, false);
  mpu.setSlaveDataLength(2, 2);

  mpu.setI2CMasterModeEnabled(1);

    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
// ================================================================
// ===                    SETUP MAX7456                         ===
// ================================================================
    SPI.begin();
    SPI.setClockDivider( SPI_CLOCK_DIV2 );      // Must be less than 10MHz.
    
    // Initialize the MAX7456 OSD:
    OSD.begin(MAX7456_COLS_P1,MAX7456_ROWS_P1,MAX7456_FULLSCREEN);                                // Use NTSC with default area.
    OSD.setSwitchingTime( 5 );                  // Set video croma distortion 
                                                //   to a minimum.
    //OSD.setCharEncoding( MAX7456_ASCII );       // Only needed if ascii font.
    OSD.display();                              // Activate the text display.
    OSD.setDefaultSystem(MAX7456_PAL);
    OSD.clear();
  Serial.println("osd");
  
  
  // Compass OSD
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  
          
          while (OSD.notInVSync()) {

        timeMillis[3] = (millis() + msOffset)        // The millis() function keeps 
                     % 10 + '0';                //   the time even when not
        timeMillis[2] = (millis() + msOffset)        //   calculating it.
                     / 10 % 10 + '0';
        timeMillis[1] = (millis() + msOffset)
                     / 100 % 10 + '0';

        if (!toggle && timeMillis[1] == '5') {        // Converts millis() into a 
          toggle = true;                        //   pulse per second.
        }
        if (toggle && timeMillis[1] == '0') {
          time[7]++;                            // Increment on falling edge.
          toggle = false;
        }
        
        if(time[7] > '9') {
          time[7] = '0';
          time[6]++;
        }
        if(time[6] > '5') {
          time[6] = '0';
          time[4]++;
        }
        if (time[4] > '9') {
          time[4] = '0';
          time[3]++;
        }
        if (time[3] > '5') {
          time[3] = '0';
          time[1]++;
        }
        if (time[1] > '9') {
          time[1] = '0';
          time[0]++;
        }
        if (time[0] >= '2' && time[1] > '3') {
          time[1] = '0';
          time[0] = '0';
        }

        
        // Add aditional code here.
      }
      OSD.setCursor( 0, -2);
      OSD.print( F("\xB3") );
      OSD.setCursor( 1, -2);
      OSD.print( F("\xB4") );
      OSD.setCursor( 2, -2);
      int y = millis();
      
      
      if ((y - z)> 300){
        z = y;
        OSD.print((char)x);
        x = x+1;
      }
      
      //OSD.print( "w" );

      OSD.setCursor( -1 - strlen(time), -1 );
      OSD.print( time);

      
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    
    // wait for MPU interrupt or extra packet(s) available
    mpu.resetFIFO();
    while (fifoCount < packetSize) {fifoCount = mpu.getFIFOCount();}
    
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
            //Read magnetometer measures
            mx=mpu.getExternalSensorWord(0);
            my=mpu.getExternalSensorWord(2);
            mz=mpu.getExternalSensorWord(4);
           
  double Xh = mx * cos(ypr[1]) + my * sin(ypr[2]) * sin(ypr[1]) - mz * cos(ypr[2])*sin(ypr[1]);
  double Yh = my * cos(ypr[2]) - mz * sin(ypr[2]);
  float heading = atan2 ( Yh, Xh );
  if(heading < 0) heading += 2*PI;
  if(heading > 2*PI) heading -= 2*PI;
  int headingDegrees = heading * 180/M_PI; //VALORE BUSSOLA COMPENSATA
  
           //float heading2 = atan2(my, mx);
            //if(heading2 < 0) heading2 += 2 * M_PI;
            /*
            Serial.print("\t");
            Serial.print("ypr\t");
            //Serial.print(ypr[0] * 180/M_PI);
            //Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[2] * 180/M_PI);
            //Serial.print("\t h:\t");
            //Serial.print(heading2 * 180/M_PI);
            Serial.print("\t");*/
            Serial.print("mag : ");
            Serial.print(headingDegrees);
            Serial.println("\t");
            
            
            
            OSD.setCursor( 5, 5);
            OSD.print( headingDegrees);
      }
    
          
                               

// Local Methods ///////////////////////////////////////////////////////////////

  byte isLeapYear( int year ) 
  {
    if (year % 400 == 0) return 1;
    else if (year % 100 == 0) return 0;
    else if (year % 4 == 0) return 1;
    return 0;
  }
  //end isLeapYear(
