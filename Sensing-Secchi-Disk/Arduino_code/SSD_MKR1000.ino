/*******************************
  5V INPUT DEFINE, VARIABLES...
  /********************************/
float voltage;
int sensorValue;

/*******************************
  AS7341 DEFINE, VARIABLES...
  /********************************/
#include <Adafruit_AS7341.h>
Adafruit_AS7341 as7341;

/*******************************
  MS5803 DEFINE, VARIABLES...
  /********************************/
#include <Wire.h>
#include <SparkFun_MS5803_I2C.h> // Click here to get the library: http://librarymanager/All#SparkFun_MS5803-14BA
// Begin class with selected address
// available addresses (selected by jumper on board)
// default is ADDRESS_HIGH
//  ADDRESS_HIGH = 0x76
//  ADDRESS_LOW  = 0x77
MS5803 sensor(ADDRESS_HIGH);
//Create variables to store results
float temperature_c, temperature_f;
double pressure_abs, pressure_relative, altitude_delta, pressure_baseline;
// Create Variable to store altitude in (m) for calculations;
double base_altitude = 1655.0; // Altitude of SparkFun's HQ in Boulder, CO. in (m)

/*******************************
  10K Precision Epoxy Thermistor
  /********************************/
// which analog pin to connect
#define THERMISTORPIN A0
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000
// Number of samples for averaging
int samples[NUMSAMPLES];
float Analog_read;
float Resistance_read;
float steinhart;

/*******************************
  TMP117EVM Temperature Sensor
  /********************************/
const int TMP117_Address = 0x48;
const int Temp_Reg = 0x00;
const int Config_Reg = 0x01;
double temperature_A;

/*******************************
  SD CODE
  /********************************/
#include <SPI.h>
#include <SD.h>
File myFile;
const int chipSelect = 4;
#define FILE_BASE_NAME "000000"
#define FILE_BASE_NAME2 "000000"

/*******************************
  FILENAMING
  /********************************/
String logFile;
String getDateStamp_FIN;
String getTimeStamp_FIN;
String longitude_FIN;
String latitude_FIN;
char fileName[] = "00000000.CSV";

/*******************************
  GPS
  /********************************/
// GPS ports and connect
#include <Adafruit_GPS.h>
#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false
float Start_prog = 0;
float Start_prog_NOGPS = 0;

/*******************************
  WEBSERVER
  /********************************/
#include <WiFi101.h>
int status = WL_IDLE_STATUS;
WiFiServer server(80);
//****Sensitive data****//
char ssid[] = "Sensing_Secchi_A004";    // your network SSID (name)
char pass[] = "A004";                // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                     // your network key Index number (needed only for WEP)
int led =  LED_BUILTIN;
//****Files for server transfers****//
File root;
char *CSV_EXT = "CSV";
char fname[12];
int const file_size = 12;
int index_1 = 0;
int index_2 = 0;
int file_count_1 = 0;
String get_info = "";
String readString_ = "0";
String Secchi = "";
String FU = "";
String pH = "";
String FileWanted = "";
String mystring = "";
String TESTI = "";
String TESTI2 = "";
String SSID_LENGTH = "";
String PASS_LENGTH = "";
String SSID_INPUT = "";
String PASS_INPUT = "";
int INA = 0;
int INB = 0;
int INC = 0;
int IND = 0;

//****IP Addresses 192.168.1.1****//
char link[]  = "http://192.168.1.1/get_data";
char link0[] = "http://192.168.1.1/data_delete_check";
char link1[] = "http://192.168.1.1/data_delete";
char link2[] = "http://192.168.1.1/data_input";
char link3[] = "http://192.168.1.1";
char link4[] = "http://192.168.1.1/router_connection";
char link5[] = "http://192.168.1.1/sketch_upload";

/*******************************
  WiFi Sketch upload
  /********************************/
#include <ArduinoOTA.h>
int sketch_dis = 0;
int sketch_log = 0;

/*******************************
  TIME
  /********************************/
#include <RTCZero.h>
RTCZero rtc;
uint32_t Light_time = millis();
uint32_t Press_time = millis();
uint32_t GPS_time   = millis();
uint32_t SD_time    = millis();
uint32_t Them_time  = millis();
uint32_t TMP117EVM_time= millis();
uint32_t Time_Lapse = 10;  //controls the logging rate, set to zero for maximim logging (1000 = 1 sec)
uint32_t Time_Lapse_GPS = 300000; // 5 minute check for GPS if not there start
int T_START = 0;
unsigned long currentMillis_start = 0;

/*******************************
  START CALLS
  /********************************/
int Data_collection = 0;
int Data_transfer = 0;

/*******************************
  LED PINS
  /********************************/
int led_R = 0;  

// the setup routine runs once when you press reset:
void setup() {
  /*******************************
    COMMUNICATE WITH COMPUTER SERIAL MONITOR OR NOT (COMMENT)
    /********************************/
  //    while (!Serial) {
  //      delay(1);
  //    }

  /*******************************
    SD CODE (NEEDED FOR ALL MODES)
  ********************************/
  Serial.begin(9600);
  pinMode(chipSelect, OUTPUT);
  if (!SD.begin(4)) {
    while (1);
  }
  root = SD.open("/");

  /*******************************
    LED
  ********************************/
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

}

// the loop routine runs over and over again forever:
void loop() {

  /*******************************
    5V INPUT SWITCH
    /********************************/
  // read the input on analog pin 0:
  sensorValue = analogRead(A1);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5.V):
  voltage = sensorValue * (5. / 1023.0);

  /*******************************
    SKETCH UPLOAD MODE
    /********************************/
  if (sketch_dis == 1) {
    Data_collection = 0;
    Data_transfer = 0;
    if (sketch_log == 0) {
      WiFi.end();
      setupC();
      sketch_log = 1;
    }
    ArduinoOTA.poll();
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(500);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(500);                       // wait for a second
  } else {
    if (voltage <= 2.0) {
      /*******************************
        DATA COLLECTION MODE
        /********************************/
      Data_transfer = 0;
      if (Data_collection == 0) {
        setupA();
        Data_collection = 1;
      }

      //Turn LED on
      digitalWrite(LED_BUILTIN, HIGH);
      //capture current time
      unsigned long currentMillis = millis();
      //Control time data collection starts
      if (T_START == 0) {
        unsigned long currentMillis_start = millis();
        T_START = 1;
      }
      if (Start_prog == 0) {
        // Make sure RED LED OFF
         digitalWrite(led_R, LOW);   // turn red led off
        /*******************************
          GPS AND FILE NAMEING / HEADERS
        ********************************/
        //Read GPS
        char c = GPS.read();
        // if you want to debug, this is a good time to do it!
        if ((c) && (GPSECHO))
          Serial.write(c);
        // if a sentence is received, we can check the checksum, parse it...
        if (GPS.newNMEAreceived()) {
          if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
            return;  // we can fail to parse a sentence in which case we should just wait for another
        }
      }
      //Start if GPS fix, if not 7 minute check for GPS, if not there start
      if (GPS.fix) {
        Start_prog = Start_prog + 1;
      } else {
        if (currentMillis - currentMillis_start  >= Time_Lapse_GPS) { // 5 minute check for GPS if not there start
          Start_prog = Start_prog + 1;
          Start_prog_NOGPS = 1;
        }
      }
      //Once fix or not get lat, lon, date, time and write file header
      if (Start_prog == 1) {
        //Turn Red LED on
        digitalWrite(led_R, HIGH);   // turn red led on
        //FIRST READING CREATE FILE
        if (Start_prog_NOGPS == 0) {
          //adjust RTC time using GPS
          latitude_FIN     = String(GPS.latitudeDegrees, 5);
          longitude_FIN    = String(GPS.longitudeDegrees, 5);
          getDateStamp_FIN = getDateStamp();
          logFile = fileNameFromDate();
          rtc.begin();
          rtc.setTime(GPS.hour , GPS.minute, GPS.seconds);
        } else {
          latitude_FIN     = "-999";
          longitude_FIN    = "-999";
          getDateStamp_FIN = "-999";
          logFile = "00000000.csv";
          const uint8_t BASE_NAME_SIZE2 = sizeof(FILE_BASE_NAME2) - 1;
          while (SD.exists(logFile)) {
            if (logFile[BASE_NAME_SIZE2 + 1] != '9') {
              logFile[BASE_NAME_SIZE2 + 1]++;
            } else if (fileName[BASE_NAME_SIZE2] != '9') {
              logFile[BASE_NAME_SIZE2 + 1] = '0';
              logFile[BASE_NAME_SIZE2]++;
            }
          }
          rtc.begin();
        }

        //Add header to file
        File myFile = SD.open(logFile, FILE_WRITE);
        if (myFile) {
          myFile.println("SmartSecchi S/N = A004" );;
          myFile.print("Latitude (degs N) = " ); myFile.println(latitude_FIN);
          myFile.print("Longitude (degs E) = "); myFile.println(longitude_FIN);
          myFile.print("Date = "); myFile.println(getDateStamp_FIN); // Date// Date
          myFile.print("Time (UTC)"); // Time
          myFile.print(", ");
          myFile.print("Pressure (mbar) MS5803");
          myFile.print(", ");
          myFile.print("Temp (deg C) MS5803");
          myFile.print(", ");
          myFile.print("415nm");
          myFile.print(", ");
          myFile.print("445nm");
          myFile.print(", ");
          myFile.print("480nm");
          myFile.print(", ");
          myFile.print("515nm");
          myFile.print(", ");
          myFile.print("555nm");
          myFile.print(", ");
          myFile.print("590nm");
          myFile.print(", ");
          myFile.print("630nm");
          myFile.print(", ");
          myFile.print("680nm");
          myFile.print(", ");
          myFile.print("Clear");
          myFile.print(", ");
          myFile.print("NIR");
          myFile.print(", ");
          myFile.print("Resistance 10KTHERM");
          myFile.print(", ");
          myFile.print("Temp (deg C) 10KTHERM");
          myFile.print(", ");
          myFile.print("Temp (deg C) TMP117");
          myFile.println();
          // close the file:
          myFile.close();
        }
        GPS_time = currentMillis;
      }
      // START DATA LOGGING
      if (Start_prog >= 1) {
        // Get time stamp
        getTimeStamp_FIN = getTimeStamp();
        /*******************************
          /* AS7341 LOOP CODE
          /********************************/
        uint16_t readings[12];
        int counts[12];  //float counts[12] for basic counts
        if (currentMillis  - Light_time > Time_Lapse) {
          if (!as7341.readAllChannels(readings)) {
            Serial.println("Error reading all channels!");
            return;
          }
          for (uint8_t i = 0; i < 12; i++) {
            if (i == 4 || i == 5) continue;
            // we skip the first set of duplicate clear/NIR readings
            // (indices 4 and 5)
            //For basic counts uncomment
            //counts[i] = as7341.toBasicCounts(readings[i]);
            //For readings uncomment
            counts[i] = readings[i];
          }
          Light_time = currentMillis;
        }
        /*******************************
          /* MS5803 LOOP CODE
          /********************************/
        if (currentMillis  - Press_time > Time_Lapse) {
          // To measure to higher degrees of precision use the following sensor settings:
          // ADC_256
          // ADC_512
          // ADC_1024
          // ADC_2048
          // ADC_4096
          // Read temperature from the sensor in deg C. This operation takes about
          temperature_c = sensor.getTemperature(CELSIUS, ADC_4096);
          // Read pressure from the sensor in mbar.
          pressure_abs = sensor.getPressure(ADC_4096);
          Press_time = currentMillis;
        }

        /*******************************
          /* TMP117 LOOP CODE
          /********************************/
        if (currentMillis  - TMP117EVM_time > Time_Lapse) {
          // Read temperature from the sensor in deg C.
          temperature_A = ReadTempSensor();
          TMP117EVM_time = currentMillis;
        }
        
        /*******************************
          /* 10K Precision Epoxy Thermistor
          /********************************/
        if (currentMillis  - Them_time > Time_Lapse) {
          uint8_t i;
          // take N samples in a row, with a slight delay
          for (i = 0; i < NUMSAMPLES; i++) {
            samples[i] = analogRead(THERMISTORPIN);
            delay(10);
          }
          // average all the samples out
          Analog_read = 0;
          for (i = 0; i < NUMSAMPLES; i++) {
            Analog_read += samples[i];
          }
          Analog_read /= NUMSAMPLES;
          // convert the value to resistance
          Resistance_read = 1023 / Analog_read - 1;
          Resistance_read = SERIESRESISTOR / Resistance_read;
          // convert to temperature using steinhart equation
          steinhart = Resistance_read / THERMISTORNOMINAL;     // (R/Ro)
          steinhart = log(steinhart);                  // ln(R/Ro)
          steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
          steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
          steinhart = 1.0 / steinhart;                 // Invert
          steinhart -= 273.15;                         // convert absolute temp to C
          Them_time = currentMillis;
        }
        /*******************************
          /*  SD LOOP
          /*********************************/
        if (currentMillis  - SD_time > Time_Lapse) {
          // open the file. note that only one file can be open at a time,
          // so you have to close this one before opening another.
          File myFile = SD.open(logFile, FILE_WRITE);
          // if the file opened okay, write to it:
          if (myFile) {
            myFile.print(getTimeStamp_FIN); // Time
            myFile.print(", ");
            myFile.print(pressure_abs);
            myFile.print(", ");
            myFile.print(temperature_c);
            myFile.print(", ");
            myFile.print(counts[0]);
            myFile.print(", ");
            myFile.print(counts[1]);
            myFile.print(", ");
            myFile.print(counts[2]);
            myFile.print(", ");
            myFile.print(counts[3]);
            myFile.print(", ");
            myFile.print(counts[6]);
            myFile.print(", ");
            myFile.print(counts[7]);
            myFile.print(", ");
            myFile.print(counts[8]);
            myFile.print(", ");
            myFile.print(counts[9]);
            myFile.print(", ");
            myFile.print(counts[10]);
            myFile.print(", ");
            myFile.print(counts[11]);
            myFile.print(", ");
            myFile.print(Resistance_read);
            myFile.print(", ");
            myFile.print(steinhart);
            myFile.print(", ");
            myFile.print(temperature_A);
            myFile.println();
            // close the file:
            myFile.close();
          }
          SD_time = currentMillis;
        }
      }
    } else {
      // Switch data logging LEDs off
      digitalWrite(led_R, LOW);   // turn red led off
      // CLEAR FOR START OF DATA COLLECTION
      Start_prog       = 0;
      Start_prog_NOGPS = 0;
      T_START = 0;
      Data_collection = 0;
      /*******************************
        DATA TRANSFER MODE
        /********************************/
      if (Data_transfer == 0) {
        setupB();
        Data_transfer = 1;
      }
      //Create intergers for storing data
      int r, t, p, q;
      // compare the previous status to the current status
      if (status != WiFi.status()) {
        // it has changed, so update the variable
        status = WiFi.status();
        if (status == WL_AP_CONNECTED) {
          byte remoteMac[6];
          WiFi.APClientMacAddress(remoteMac);
        }
      }
      WiFiClient client = server.available();   // listen for incoming clients
      if (client) {                             // if you get a client,
        String currentLine = "";                // make a String to hold incoming data from the client
        while (client.connected()) {            // loop while the client's connected
          delayMicroseconds(10);                // ADDED FOR MKR1000::::This is required for the Arduino Nano RP2040 Connect - otherwise it will loop so fast that SPI will never be served.
          if (client.available()) {             // if there are bytes to read from the client,
            char c = client.read();             // read a byte, then
            //read char by char HTTP request
            //store characters to string
            readString_ += c;
            if (c == '\n') {                    // if the byte is a newline character
              if (currentLine.length() == 0) {
                client.println("HTTP/1.1 200 OK");
                // ROUTER CONNECTION FOR SKETCH UPLOAD
                if (readString_.indexOf("GET /router_connection") >= 0) {
                  client.println("Content-type:text/html");
                  client.println();
                  client.write("<p></p>");
                  client.write("<b>Connect to wireless router for firmware update</b><br>");
                  client.write("<p>Length of SSID characters requires a 2 character number (e.g. 08 for 8 characters)<br>");
                  client.write("Length of PASSWORD characters requires a 2 character number (e.g. 06 for 6 characters)<br>");
                  client.write("<form method=GET> Length of SSID characters: <input type=text name=t><br>");
                  client.write("Length of PASSWORD characters: <input type=text name=r><br>");
                  client.write("SSID: <input type=text name=r><br>");
                  client.write("PASSWORD: <input type=text name=p><input type=submit></form>");
                  client.write("<p></p>");
                  client.print("<a href=\"");
                  client.print(link5);
                  client.println("\">Firmware-update-connection</a>");
                  client.write("<p></p>");
                  client.print("<a href=\"");
                  client.print(link3);
                  client.println("\">Home-page</a>");
                  // break out of the while loop:
                  break;
                }
                // SKETCH UPLOAD
                if (readString_.indexOf("GET /sketch_upload") >= 0) {
                  client.println("Content-type:text/html");
                  client.println();
                  client.write("<p></p>");
                  client.println("Sketch upload mode");
                  sketch_dis = 1;
                  client.stop();
                  break;
                }
                // DATA DELETE CHECKS
                if (readString_.indexOf("GET /data_delete_check") >= 0) {
                  //Remove file wanted
                  client.println("Content-type:text/html");
                  client.println();
                  client.write("<p></p>");
                  client.println(FileWanted + ".CSV");
                  client.println("Are you sure you want to delete this file?");
                  client.write("<p></p>");
                  client.print("<a href=\"");
                  client.print(link1);
                  client.println("\">Delete-file</a>");
                  client.write("<p></p>");
                  client.print("<a href=\"");
                  client.print(link3);
                  client.println("\">Home-page</a>");
                  break;
                }
                // DATA DELETE LINKS
                if (readString_.indexOf("GET /data_delete") >= 0) {
                  //Remove file wanted
                  SD.remove(FileWanted + ".CSV");
                  client.println("Content-type:text/html");
                  client.println();
                  client.write("<p></p>");
                  client.println(FileWanted + ".CSV");
                  client.println("Deleted");
                  client.write("<p></p>");
                  client.print("<a href=\"");
                  client.print(link3);
                  client.println("\">Home-page</a>");
                  break;
                }
                // GET ALL DATA + INPUTS
                if (readString_.indexOf("GET /get_data") >= 0) {
                  client.println("Content-type:text/plain");
                  client.println();
                  //Print Secchi FU
                  client.print("Secchi Depth (cm) = ");
                  client.println(Secchi);
                  client.print("Forel Ule Scale = ");
                  client.println(FU);
                  client.print("pH Colour Scale = ");
                  client.println(pH);
                  File dataFile = SD.open(FileWanted + ".CSV");
                  for (index_1 = 0; index_1 < dataFile.size(); index_1++) {
                    client.print((char)dataFile.read());
                  }
                  index_1 = 0;
                  dataFile.close();
                  break;
                }
                // DATA INPUT LINKS
                if (readString_.indexOf("GET /data_input") >= 0) {
                  client.println("Content-type:text/html");
                  client.println();
                  client.write("<p></p>");
                  client.write("<b>Input Secchi, Forel Ule and pH data</b><br>");
                  client.write("<p>Secchi Depth (cm) requires a 4 character number (e.g. 0128 for 128 cm, no data = 9999)<br>");
                  client.write("Forel Ule Scale requires a 2 character number (e.g. 01 for 1), no data = 99<br>");
                  client.write("pH Colour Scale requires a 2 character number (e.g. 08 for 8), no data = 99</p>");
                  client.write("<form method=GET> Secchi Depth (cm): <input type=text name=t><br>");
                  client.write("Forel Ule Scale: <input type=text name=r><br>");
                  client.write("pH Colour Scale: <input type=text name=p><input type=submit></form>");
                  client.print("<a href=\"");
                  client.print(link);
                  client.println("\">Get-Data</a>");
                  // break out of the while loop:
                  break;
                }
                // READ SD CARD PRINT FILES AND SELECT FILE
                if (readString_.indexOf("GET /get_data") != 0) {
                  if (readString_.indexOf("GET /data_input") != 0) {
                    if (readString_.indexOf("GET /data_delete") != 0) {
                      if (readString_.indexOf("GET /data_delete_check") != 0) {
                        if (readString_.indexOf("GET /router_connection") != 0) {
                          if (readString_.indexOf("GET /sketch_upload") != 0) {
                            client.println("Content-type:text/html");
                            client.println();
                            client.write("<p></p>");
                            client.write("<b>Filenames on SD Card [YEAR(2)-MONTH(2)-DAY(2)-CNT(2)]</b><br>");
                            root = SD.open("/");
                            file_count_1 = getFileCount("/");
                            for (index_2 = 0; index_2 < file_count_1; index_2++) {
                              File entry =  root.openNextFile();
                              mystring = entry.name();
                              if (strcmp(CSV_EXT, &entry.name()[strlen(entry.name()) - strlen(CSV_EXT)]) == 0) {
                                //Gets rid of odd hidden files (which I can't see on a mac??)
                                if (mystring.indexOf('~') >= 0 ) {
                                  SD.remove(mystring);
                                }
                                client.println(mystring + "<br>");
                              }
                              entry.close();
                            }
                            index_2 = 0;
                            client.println();
                            client.write("<form method=GET> Select File: <input type=text name=q><input type=submit></form>");
                            client.println("Click this link to input data to file and read it");
                            client.print("<a href=\"");
                            client.print(link2);
                            client.println("\">Input-Secchi-FU-pH</a>");
                            client.write("<p></p>");
                            client.println("Click this link if you want to delete file from device");
                            client.print("<a href=\"");
                            client.print(link0);
                            client.println("\">Data-delete-check</a>");
                            client.write("<p></p>");
                            client.println("Click this link to update fireware on device");
                            client.print("<a href=\"");
                            client.print(link4);
                            client.println("\">Router-connection-for-firmware-update</a>");
                            client.println();
                            break;
                          }
                        }
                      }
                    }
                  }
                }
              }
              else {      // if you got a newline, then clear currentLine:
                //read char by char HTTP request
                currentLine = "";
              }

            }
            else if (c != '\r') {    // if you got anything else but a carriage return character,
              currentLine += c;      // add it to the end of the currentLine
            }
          }
        }
        // STOP CLIENT
        client.stop();
        // EXTRACT INFORMATION FROM WEB INPUT
        TESTI = readString_.substring(8, 16);
        if (TESTI.toFloat() > 0) {
          FileWanted = readString_.substring(8, 16);
        }
        TESTI2 = readString_.substring(20, 24);
        if (TESTI2.toFloat() > 0) {
          Secchi = readString_.substring(18, 22);
          FU     = readString_.substring(25, 27);
          pH     = readString_.substring(30, 32);
        }
        SSID_LENGTH = readString_.substring(25, 27);
        PASS_LENGTH = readString_.substring(30, 32);
        SSID_LENGTH.toInt();
        PASS_LENGTH.toInt();
        if (SSID_LENGTH.toInt() > 0) {
          if (PASS_LENGTH.toInt() > 0) {
            INA = 35;
            INB = INA + SSID_LENGTH.toInt();
            INC = INB + 3;
            IND = INC + PASS_LENGTH.toInt();
            SSID_INPUT = readString_.substring(INA, INB);
            PASS_INPUT = readString_.substring(INC, IND);
          }
        }
        // CLEAR READSTRING
        readString_ = "";
      }
    }
  }
}

/*******************************
  FUNCTION TO GET DATE / TIME
********************************/
// format the time as hh:mm:ss
String getTimeStamp() {
  String timestamp_ = "";
  if (rtc.getHours() <= 9) timestamp_ += "0";
  timestamp_ += rtc.getHours();
  timestamp_ += ":";
  if (rtc.getMinutes() <= 9) timestamp_ += "0";
  timestamp_ += rtc.getMinutes();
  timestamp_ += ":";
  if (rtc.getSeconds() <= 9) timestamp_ += "0";
  timestamp_ += rtc.getSeconds();
  return timestamp_;
}
// format the date as dd-mm-yyyy: FROM GPS
String getDateStamp() {
  String datestamp = "";
  // add century:
  datestamp += "20";
  if (GPS.year <= 9) datestamp += "0";
  datestamp += String(GPS.year, DEC);
  datestamp += "-";
  if (GPS.month <= 9) datestamp += "0";
  datestamp += String(GPS.month, DEC);
  datestamp += "-";
  if (GPS.day <= 9) datestamp += "0";
  datestamp += String(GPS.day, DEC);
  return datestamp;
}
/*******************************
  FUNCTION TO GET FILENAME
********************************/
String fileNameFromDate() {
  //create file
  String result_P = "";
  if (GPS.year <= 9) result_P += "0";
  result_P += String(GPS.year, DEC);
  if (GPS.month <= 9) result_P += "0";
  result_P += String(GPS.month, DEC);
  if (GPS.day <= 9) result_P += "0";
  result_P += String(GPS.day, DEC);
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  fileName[0]  = result_P[0];
  fileName[1]  = result_P[1];
  fileName[2]  = result_P[2];
  fileName[3]  = result_P[3];
  fileName[4]  = result_P[4];
  fileName[5]  = result_P[5];
  fileName[6]  = '0';
  fileName[7]  = '0';
  fileName[8]  = '.';
  fileName[9]  = 'C';
  fileName[10] = 'S';
  fileName[11] = 'V';
  while (SD.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    } else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    }
  }
  return fileName;
}

/*******************************
  FUNCTION TO FILE COUNT
********************************/
int getFileCount(String dir )
{
  File d = SD.open( dir );
  int count_files = 0;
  while ( true )
  {
    File entry =  d.openNextFile();
    if ( !entry )
    {
      // no more files. Let's return the number of files.
      return count_files;
    }
    String file_name = entry.name();  //Get file name so that we can check
    //if it's a duplicate
    if ( file_name.indexOf('~') != 0 )  //Igrnore filenames with a ~. It's a mac thing.
    { //Just don't have file names that have a ~ in them
      count_files++;
    }
  }
}

void setupA() {
  /*******************************
     AS7341 SET-UP CODE
     /********************************/
  Serial.begin(115200);
  if (!as7341.begin()) {
    while (1) {
      delay(10);
    }
  }
  as7341.setATIME(59); //100 ms integration time 
  as7341.setASTEP(599);//100 ms integration time 
  as7341.setGain(AS7341_GAIN_4X);//*4 as in https://www.mdpi.com/1424-8220/21/10/3390/htm

  /*******************************
     TMP117EVM Temperature Sensor
     /********************************/
  // Nothing needed

  /*******************************
    MS5803 SET-UP CODE
    /********************************/
  Wire.begin();
  //Initialize Serial Monitor
  Serial.begin(9600);
  //Retrieve calibration constants for conversion math.
  sensor.reset();
  sensor.begin();
  pressure_baseline = sensor.getPressure(ADC_4096);

  /*******************************
    GPS
  ********************************/
  Serial.begin(115200);
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600); // > in my case I use UBLOX 6M: GY-GPS6Mv2
  // the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
}

void setupB() {
  /*******************************
    WEBSERVER
    /********************************/
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    // don't continue
    while (true);
  }
  // by default the local IP address of will be 192.168.1.1
  // you can override it with the following:
  ///WiFi.config(IPAddress(10, 10, 10, 10));
  // Create open network. Change this line if you want to create a WEP network:
  status = WiFi.beginAP(ssid);
  if (status != WL_AP_LISTENING) {
    // don't continue
    while (true);
  }
  // wait 5 seconds for connection:
  delay(5000);
  // start the web server on port 80
  server.begin();
}

void setupC() {
  //Initialize serial:
  Serial.begin(9600);
  //  09
  //  14
  //  BT-N6AKQF
  //  7vCEudmRnRFy4J
  const char* ssid = SSID_INPUT.c_str();
  const char* pass = PASS_INPUT.c_str();
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    // don't continue:
    while (true);
  }
  // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) {
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
  }
  // start the WiFi OTA library with internal (flash) based storage
  ArduinoOTA.begin(WiFi.localIP(), "Arduino", "password", InternalStorage);
}

double ReadTempSensor(void) {
  //Device address (can find this from the I2C scanner if unknown)
  //int TMP117_Address = 0x48;
  //Data array to store 2-byte from I2C line
  uint8_t datab[2];
  //Combination of 2-byte data into 16-bit data
  int16_t datac;
  //Points to device and begins tranmission
  Wire.beginTransmission(TMP117_Address);
  //Points to temperature register to read/write data
  Wire.write(Temp_Reg);
  //End data transfer and transmits data fronm register
  Wire.endTransmission();
  //Delay to allow sufficient conversion time
  delay(10);
  //Request 2-byte temperature data from device
  Wire.requestFrom(TMP117_Address, 2);
  //Checks if data recieved matches the requested 2-bytes
  if (Wire.available() <= 2) {
    datab[0] = Wire.read();
    datab[1] = Wire.read();
    //Combines data to make 16-bit binary number
    datac = ((datab[0] << 8) | datab[1]);
    //Convert to celcius (7.8125 mC resolution) and return
    return datac * 0.0078125;
  }
}
