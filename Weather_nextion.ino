// The Weather Station@mega2560 with nextion display
#include <Time.h>
#include <TimeLib.h>
#include <DS3232RTC.h>
#include <EEPROM.h>
#include <SPI.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <math.h>
#include <sunMoon.h>
#include <WlessOregonV2.h>

#define DHTTYPE    DHT22
#define OUR_latitude    55.751244
#define OUR_longtitude  37.618423
#define OUR_timezone    180                     // localtime with UTC difference in minutes

const byte w433_INTR = 5;                       // Interrupt number of wireless 433 MHz sensor
const byte DHT_PIN   = 4;                       // DHT22 data pin
const byte hbPIN     = A2;                      // NE555 automatic reset heartbeat pin


#if SD_SPI_CONFIGURATION >= 3                   // Must be set in SdFat/SdFatConfig.h to use software SPI
const uint8_t SOFT_MISO_PIN = 50;
const uint8_t SOFT_MOSI_PIN = 49;               // hardware MOSI pin is out of order in my Mega board
const uint8_t SOFT_SCK_PIN  = 52;
#endif

const byte SD_PIN           = 53;               // SD card reader select pin

const byte     max_sensors   = 5;               // The maximum number of the sensors (remote + one internal) connected
const uint16_t h_samples     = 288;             // History sapmples (dots) on the history graph
const byte     gh            = 200;             // Maximun height of the graph
const uint16_t high_pressure = 767-16;          // High pressure, depends on altitude
const uint16_t low_pressure  = 757-16;          // Low pressure,  depends on altitude

static const char* months[12] = {               // January-December (iso8859-5)
  "\xCF\xDD\xD2\xD0\xE0\xEF", "\xC4\xD5\xD2\xE0\xD0\xDB\xEF", "\xBC\xD0\xE0\xE2\xD0",
  "\xB0\xDf\xE0\xD5\xDB\xEF", "\xBC\xD0\xEF", "\xB8\xEE\xDD\xEF",
  "\xB8\xEE\xDB\xEF", "\xB0\xD2\xD3\xE3\xE1\xE2\xD0",
  "\xC1\xD5\xDD\xE2\xEF\xD1\xE0\xEF", "\xBE\xDA\xE2\xEF\xD1\xE0\xEF", "\xBD\xDE\xEF\xD1\xE0\xEF", "\xB4\xD5\xDA\xD0\xD1\xE0\xEF"
};
static const char* wday[7]    = {               // Sunday - Saturday (iso8859-5)
  "\xB2\xDE\xE1\xDA\xE0\xD5\xE1\xDD\xEC\xD5", "\xBF\xDE\xDD\xD5\xD4\xD5\xDB\xEC\xDD\xD8\xDA",
  "\xB2\xE2\xDE\xE0\xDD\xD8\xDA", "\xC1\xE0\xD5\xD4\xD0", "\xC7\xD5\xE2\xD2\xD5\xE0\xD3",
  "\xBF\xEF\xE2\xDD\xD8\xE6\xD0", "\xC1\xE3\xD1\xD1\xDE\xE2\xD0"
};

// Nextion SCREEN pages
typedef enum nxt_pages {
  NXT_MAIN = 0, SUNNY = 0, CLOUDY, RAIN, SNOW, MOONY, N_CLOUDY, N_RAIN, N_SNOW,
  NXT_SUNRISE = 8, NXT_GRAPH = 9, NXT_MENU = 10, NXT_BRIGHT = 11, NXT_SENSOR = 12, NXT_CLOCK = 13, NXT_MAXPAGE
} NxtPage; 

//------------------------------------------ Configuration data ------------------------------------------------
/* Config record in the EEPROM has the following format:
  uint32_t ID                           each time increment by 1
  struct cfg                            config data
  byte CRC                              the checksum
*/
struct cfg {
  byte ext_sensor_ID;                                 // external sensor ID (0 - random)
  byte backlight_morning;                             // morning time in 10-minute intervals
  byte backlight_evening;                             // evening time in 10-minute intervals
  byte br_day;                                        // daily brightness
  byte br_night;                                      // nightly brightness
};

class CONFIG {
  public:
    CONFIG() {
      can_write     = false;
      buffRecords   = 0;
      rAddr = wAddr = 0;
      eLength       = 0;
      nextRecID     = 0;
      byte rs = sizeof(struct cfg) + 5;               // The total config record size
      // Select appropriate record size; The record size should be power of 2, i.e. 8, 16, 32, 64, ... bytes
      for (record_size = 8; record_size < rs; record_size <<= 1);
    }
    void init();
    void load(void);
    void getConfig(struct cfg &Cfg);                  // Copy config structure from this class
    void updateConfig(struct cfg &Cfg);               // Copy updated config into this class
    bool save(void);                                  // Save current config copy to the EEPROM
    bool saveConfig(struct cfg &Cfg);                 // write updated config into the EEPROM
    
  private:
    void defaultConfig(void);
    struct cfg Config;
    bool readRecord(uint16_t addr, uint32_t &recID);
    bool can_write;                                   // The flag indicates that data can be saved
    byte buffRecords;                                 // Number of the records in the outpt buffer
    uint16_t rAddr;                                   // Address of thecorrect record in EEPROM to be read
    uint16_t wAddr;                                   // Address in the EEPROM to start write new record
    uint16_t eLength;                                 // Length of the EEPROM, depends on arduino model
    uint32_t nextRecID;                               // next record ID
    byte     record_size;                             // The size of one record in bytes
};

 // Read the records until the last one, point wAddr (write address) after the last record
void CONFIG::init(void) {
  eLength = EEPROM.length();
  uint32_t recID;
  uint32_t minRecID = 0xffffffff;
  uint16_t minRecAddr = 0;
  uint32_t maxRecID = 0;
  uint16_t maxRecAddr = 0;
  byte records = 0;

  nextRecID = 0;

  // read all the records in the EEPROM find min and max record ID
  for (uint16_t addr = 0; addr < eLength; addr += record_size) {
    if (readRecord(addr, recID)) {
      ++records;
      if (minRecID > recID) {
        minRecID = recID;
        minRecAddr = addr;
      }
      if (maxRecID < recID) {
        maxRecID = recID;
        maxRecAddr = addr;
      }
    } else {
      break;
    }
  }

  if (records == 0) {
    wAddr = rAddr = 0;
    can_write = true;
    return;
  }

  rAddr = maxRecAddr;
  if (records < (eLength / record_size)) {            // The EEPROM is not full
    wAddr = rAddr + record_size;
    if (wAddr > eLength) wAddr = 0;
  } else {
    wAddr = minRecAddr;
  }
  can_write = true;
}

void CONFIG::getConfig(struct cfg &Cfg) {
  memcpy(&Cfg, &Config, sizeof(struct cfg));
}

void CONFIG::updateConfig(struct cfg &Cfg) {
  memcpy(&Config, &Cfg, sizeof(struct cfg));
}

bool CONFIG::saveConfig(struct cfg &Cfg) {
  updateConfig(Cfg);
  return save();                                      // Save new data into the EEPROM
}

bool CONFIG::save(void) {
  if (!can_write) return can_write;
  if (nextRecID == 0) nextRecID = 1;

  uint16_t startWrite = wAddr;
  uint32_t nxt = nextRecID;
  byte summ = 0;
  for (byte i = 0; i < 4; ++i) {
    EEPROM.write(startWrite++, nxt & 0xff);
    summ <<=2; summ += nxt;
    nxt >>= 8;
  }
  byte* p = (byte *)&Config;
  for (byte i = 0; i < sizeof(struct cfg); ++i) {
    summ <<= 2; summ += p[i];
    EEPROM.write(startWrite++, p[i]);
  }
  summ ++;                                            // To avoid empty records
  EEPROM.write(wAddr+record_size-1, summ);

  rAddr = wAddr;
  wAddr += record_size;
  if (wAddr > EEPROM.length()) wAddr = 0;
  return true;
}

void CONFIG::load(void) {
  bool is_valid = readRecord(rAddr, nextRecID);
  nextRecID ++;
  if (!is_valid) defaultConfig();
  return;
}

bool CONFIG::readRecord(uint16_t addr, uint32_t &recID) {
  byte Buff[record_size];

  for (byte i = 0; i < record_size; ++i) 
    Buff[i] = EEPROM.read(addr+i);
  
  byte summ = 0;
  for (byte i = 0; i < sizeof(struct cfg) + 4; ++i) {

    summ <<= 2; summ += Buff[i];
  }
  summ ++;                                            // To avoid empty fields
  if (summ == Buff[record_size-1]) {                  // Checksumm is correct
    uint32_t ts = 0;
    for (char i = 3; i >= 0; --i) {
      ts <<= 8;
      ts |= Buff[byte(i)];
    }
    recID = ts;
    memcpy(&Config, &Buff[4], sizeof(struct cfg));
    return true;
  }
  return false;
}

void CONFIG::defaultConfig(void) {
  Config.ext_sensor_ID     =   0;                     // 0 means - any sensor
  Config.backlight_morning =  48;                     // 8:00
  Config.backlight_evening = 138;                     // 23:00
  Config.br_day            = 80;
  Config.br_night          = 10;
}


//------------------------------------------ class weatherLogger ----------------------------------------------------
class weatherLogger {
  public:
    weatherLogger(void) { canWriteSD = false; }
    void   init(byte sd_pin);
    char*  logName(time_t date=0);                  // Generate log file name based on date
    bool   writeData(byte sensorID, int t, int p, byte h);
    bool   openLog(time_t date=0);
    void   closeLog(void);
    bool   readData(byte &sensorID, int &t, int &p, byte &h, time_t &timestamp);
    void   rmOldLog(time_t date);                   // Remove the log files created before the date
  time_t calculateWriteLogTime(time_t start_time);// Write log at 0, 15, 30 and 45 minutes every hour
  private:
#if SD_SPI_CONFIGURATION >= 3                       // Must be set in SdFat/SdFatConfig.h to use software SPI
    SdFatSoftSpi<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> sd;
#else
    SdFat sd;
#endif
    void long2byte(byte* buff, long data, byte dsize = 4);
    long byte2long(byte* buff, byte dsize = 4);
    long checksum(byte *buff, byte bsize = 12);
    bool canWriteSD;
    char log_file_name[13];
    SdFile readLog;
};

void weatherLogger::init(byte sd_pin) {
   canWriteSD = sd.begin(sd_pin, SPI_HALF_SPEED);
}

char* weatherLogger::logName(time_t date) {
 
  if (date == 0) date = now();
  sprintf(log_file_name, "%04d%02d%02d.dat", year(date), month(date), day(date));
  return log_file_name;
}

bool weatherLogger::writeData(byte sensorID, int t, int p, byte h) {
  if (!canWriteSD) return false;
  logName();
  byte record[16];
  time_t now_t = now();
  long2byte(record, now_t);                         // the record begins with timestamp, 4 bytes
  long2byte(&record[4], t, 2);                      // the temperature, 2 bytes
  long2byte(&record[6], p, 2);                      // the pressure, 2 bytes
  record[8] = h;                                    // the humidity, a single byte
  record[9] = sensorID;
  record[10] = record[11] = 0;                      // Skip 10-th and 11-th bytes
  long crc = checksum(record);
  long2byte(&record[12], crc);                      // the checksum, 4 bytes

  SdFile lf;
  if (lf.open(log_file_name, O_RDWR | O_CREAT | O_AT_END )) {
    lf.write(record, 16);
    lf.close();
  }
  return true;
}

bool weatherLogger::readData(byte &sensorID, int &t, int &p, byte &h, time_t &timestamp) {
  byte record[16];
  int readb = readLog.read(record, 16);
  if (readb != 16) {
    closeLog();
    return false;
  }
  long crc = checksum(record);
  if (crc == byte2long(&record[12])) {              // Checksumm is correct
    timestamp = byte2long(record);
    t = byte2long(&record[4], 2);
    p = byte2long(&record[6], 2);
    h = record[8];
    sensorID = record[9];
    return true;
  }
  return false;
}

bool weatherLogger::openLog(time_t date) {
  logName(date);
  if (!readLog.open(log_file_name, O_READ))
    return false;
  return true;
}

void weatherLogger::closeLog(void) {
  readLog.close();
}

void weatherLogger::long2byte(byte* buff, long data, byte dsize) {
  for (byte i = 0; i < dsize; ++i) {
    buff[i] = data & 0xff;
    data >>= 8;
  }
}

long weatherLogger::byte2long(byte* buff, byte dsize) {
  long data = 0;
  for (char i = dsize-1; i >= 0; --i) {
    data <<= 8;
    data |= buff[byte(i)];
  }
  return data;
}

long weatherLogger::checksum(byte *buff, byte bsize) {
  long crc = 0;
  for (byte i = 0; i < bsize; ++i) {
    crc <<= 2;
    crc += buff[i];
  }
  return crc;
}

void weatherLogger::rmOldLog(time_t date) {
  tmElements_t tm;
  SdFile file;
  char f_name[13];

  sd.vwd()->rewind();
  while (file.openNext(sd.vwd(), O_READ)) {
    if (file.isDir()) {
      file.close();
      continue;
    }
    dir_t dir;
    file.dirEntry(&dir);
    memcpy(f_name, dir.name, 8); f_name[8] = '.';
    memcpy(&f_name[9], &dir.name[8], 3); f_name[12] = '\0';
    // Calculate last modification time of the file in date format
    uint16_t f_time = dir.lastWriteTime;
    tm.Second = FAT_SECOND(f_time);
    tm.Minute = FAT_MINUTE(f_time);
    tm.Hour   = FAT_HOUR(f_time);
    f_time    = dir.lastWriteDate;
    tm.Day    = FAT_DAY(f_time);
    tm.Month  = FAT_MONTH(f_time);
    tm.Year   = FAT_YEAR(f_time) - 1970;
    file.close();
    time_t f_date = makeTime(tm);
    if (f_date < date) {
      sd.remove(f_name);
    }
  }
}

time_t weatherLogger::calculateWriteLogTime(time_t start_time) {
  const byte period = 5;

  byte Sec = second(start_time);
  start_time -= Sec;
  byte Min = minute(start_time);
  Min %= period;
  Min = period - Min;
  return start_time + ((uint32_t)Min * 60);
}

//------------------------------------------ Heart beat class (Watch dog timer based on NE555 IC) --------------
class HB {
  public:
    HB(byte hb) {
      hb_pin = hb;
      pinMode(hb_pin, INPUT);
      period = 0;                                   // auto reset is disabled
      next_HB = 0;
    }
    void sendHeartbeat(void) {
      pinMode(hb_pin, OUTPUT);
      digitalWrite(hb_pin, LOW);                    // reset ne555 timer
      delay(200);
      pinMode(hb_pin, INPUT);
    }
    void setHBTimeout(uint16_t to = 0)                { period = to; }
    void autoHB(void) {
      if (period == 0) {                            // Automatic reset is disabled, reset now
        sendHeartbeat();
        return;
      }
      if (millis() > next_HB) {
        sendHeartbeat();
        next_HB = millis() + (long)period * 1000; 
      }
    }
  private:
    byte     hb_pin;                                // NE555 reset pin (heart beat)
    uint16_t period;                                // The period to send the heartbeat to the ne555 in seconds
    uint32_t next_HB;                               // time in ms to automatically reset the ne555
};

//------------------------------------------ class HISTORY ----------------------------------------------------
#define H_LENGTH 30
class HISTORY {
  public:
    HISTORY(void)                                   { len = 0; }
    void  init(void)                                { len = 0; }
    void  put(int item);
    bool  isFull(void)                              { return len == H_LENGTH; }
    int   last(void)                                { return queue[len-1]; }
    int   top(void)                                 { return queue[0]; }
    int   average(byte last_n = H_LENGTH);          // average value of the last last_n items
    long  dispersion(byte last_n = H_LENGTH);       // the math. dispersion * 100 of the last last_n items
    long  gradient(byte last_n = H_LENGTH);         // The gradient * 100 of the last last_n items
    void  dump(void);
  private:
    byte tailLength(byte last_n);
    byte startIndex(byte last_n);
    int queue[H_LENGTH];
    byte len;
};

void HISTORY::dump(void) {
  Serial.print(len, DEC);
  Serial.print(": ");
  for (byte i = 0; i < len; ++i) {
    Serial.print(queue[i]);
    Serial.print(", ");
  }
  Serial.println("");
}

void HISTORY::put(int item) {
  if (len < H_LENGTH) {
    queue[len++] = item;
  } else {
    for (byte i = 0; i < len-1; ++i) queue[i] = queue[i+1];
    queue[H_LENGTH-1] = item;
  }
}

int HISTORY::average(byte last_n) {
  long sum = 0;
  if (len == 0) return 0;
  if (len == 1) return queue[0];
  byte a_len = tailLength(last_n);
  byte si    = startIndex(last_n);
  for (byte i = si; i < len; ++i) sum += queue[i];
  sum += a_len >> 1;                                // round the average
  sum /= a_len;
  return (int)sum;
}

long HISTORY::dispersion(byte last_n) {
  if (len < 3) return 1000;
  long sum = 0;
  long avg = average(last_n);
  byte a_len = tailLength(last_n);
  byte si    = startIndex(last_n);
  for (byte i = si; i < len; ++i) {
    long q = queue[i];
    q -= avg;
    q *= q;
    sum += q;
  }
  sum += a_len << 1;
  float d = (float)sum * 100 / (float)a_len + 50;
  return long(d);
}

/* approfimating the last elements of the history with the line (y = Ax+B) using method of minimum squares.
 * The gradient is parameter A*100
*/
long HISTORY::gradient(byte last_n) {
  if (len < 2) return 0;
  byte a_len = tailLength(last_n);
  byte si    = startIndex(last_n);
  long sx, sx_sq, sxy, sy;
  sx = sx_sq = sxy = sy = 0;

  si += 1;
  for (byte i = si; i <= len; ++i) {
    sx    += i;
    sx_sq += i*i;
    sxy   += i*queue[i-1];
    sy    += queue[i-1];
  }

  long numerator   = a_len * sxy - sx * sy;
  long denominator = a_len * sx_sq - sx * sx;
  float a = (float)numerator * 100.0 / (float)denominator;
  return long(a);
}

byte HISTORY::tailLength(byte last_n) {
  if (last_n > H_LENGTH) last_n = H_LENGTH;
  byte a_len = len;
  if (len > last_n) a_len = last_n;
  return a_len;
}

byte HISTORY::startIndex(byte last_n) {
  if (last_n > H_LENGTH) last_n = H_LENGTH;
  byte start_index = 0;
  if (len > last_n) start_index = len - last_n;
  return start_index;
}

//------------------------------------------ class pressureSensor -----------------------------------------------------
class pressureSensor {
  public:
    pressureSensor(Adafruit_BMP280* Bmp) {
      pBmp  = Bmp;
    }
    void      init(void)                            { rh_press.init(); lh_press.init(); }
    void      update(void);                         // Read the pressure sensor data, save the data to the history
    void      load(time_t ts, int p);               // load history data from SD card
    NxtPage   forecast(bool is_cold = false, bool is_dark = false);
    int       press(void)                           { return rh_press.last(); }
    int       average(void)                         { return rh_press.average(15); }
  private:
    Adafruit_BMP280*  pBmp;                         // Pointer to the BMP sensor instance
    HISTORY           rh_press;                     // Recent history data for pressure (last 15 minutes) in mmHg.
    HISTORY           lh_press;                     // Long term pressure (15-minutes average) history data for last 6 hours
};

void pressureSensor::update(void) {
  float t = pBmp->readTemperature();
  float p = pBmp->readPressure();
  int pressure = int(p * 0.00750064);               // The pressure in mmHg
  rh_press.put(pressure);
  if ((minute() % 15) == 0) {                       // update long-term history data every 15 minutes
    int avg = rh_press.average(15);
    lh_press.put(avg);
  }
}

void pressureSensor::load(time_t ts, int p) {
  if ((ts % 900) == 0)
    lh_press.put(p);                                // Long-term history data saves every 15 minutes
  if ((now() - ts) < 600) {                         // recent data loaded
    rh_press.init();
    rh_press.put(p);
  }
}

NxtPage pressureSensor::forecast(bool is_cold , bool is_dark) {
  long grad_short    = rh_press.gradient(15);
  long grad_full     = lh_press.gradient();
  uint16_t pressure  = rh_press.last();       // The history data is pressure in mmHg
  byte f = 0;
  if (pressure > high_pressure) {                   // HIGH pressure means no rain
    if (grad_short < -8) {
      f = CLOUDY;
    } else {
      if (grad_full < -8)
        f = CLOUDY;
      else
        f = SUNNY;
    }
  } else if (pressure < low_pressure) {            // LOW pressure, means no sunny
    if (grad_full < 0) {
        f = RAIN;
    } else {
      if (grad_short < -4) {
        f = RAIN;
      } else if (grad_short <= 8) {
        f = CLOUDY;
      } else {
        f = SUNNY;
      }
    }
  } else if ((grad_short >= -5) && (grad_full >= 0)) { // NORMAL pressure
    f = SUNNY;
  } else {
    if (grad_short <= -8) {
        f = RAIN;
    } else
      f = CLOUDY;
  }

  if (is_cold && (f == RAIN))
    f = SNOW;

  if (is_dark) {
    switch (f) {
      case SUNNY:
        f = MOONY;
        break;
      case CLOUDY:
        f = N_CLOUDY;
        break;
      case RAIN:
        f = N_RAIN;
        break;
      case SNOW:
        f = N_SNOW;
        break;
      default:
        break;
    }
  }
  return NxtPage(f);
}

//------------------------------------------ class weatherSensor ------------------------------------------------------
class weatherSensor {
  public:
    weatherSensor(void)                             {}
    void   init(void);
    void   update(int temp, byte hum, bool batt, bool reset);   // Temperature in 10*cencegrees [-500; +500]
    int    temp(void)                               { return hTemp.last(); }
    byte   hum(void)                                { return hHum.last(); }
    int    aTemp(void)                              { return hTemp.average(); }
    byte   aHum(void)                               { return hHum.average(); }
    bool   isBattOK(void)                           { return battOK; }
    time_t lastUpdated(void)                        { return updated; }

  private:
    bool     battOK;                                // Whether the battery of the sensor is OK
    time_t   updated;                               // The time when the sensor data was updated
    HISTORY hTemp, hHum;                            // History data for temperature and humidity
};

void weatherSensor::init(void) {
  battOK   = false;
  updated  = 0;
  hTemp.init();
  hHum.init();
}

void weatherSensor::update(int temp, byte hum, bool batt, bool reset) {
  if (temp < -500) temp = -500;                     // lower temperature limit -50.0 cencegrees
  if (temp >  500) temp =  500;                     // upper temperature limit +50.0 cencegrees
  if (reset) {                                      // Used to load the sensor data from the SD card during start
    hTemp.init();
    hHum.init();
  }
  hTemp.put(temp);
  hHum.put(hum);
  battOK  = batt;
  updated = now();
}

//------------------------------------------ class sensorPool ---------------------------------------------------------
class sensorPool {
  public:
    sensorPool(void)                               { }
    void  init(void);
    void  update(byte ID, int temp, byte hum, bool batt, bool reset = false);
    byte  next(byte ID);                            // Next Sensor ID from the pool
    byte  previous(byte ID);                        // Previous Sensor ID from the pool
    bool  exists(byte indx);                        // Whether the sensor with ID exists
    void  freeStaled(void);                         // Free staled sensor (the one not updated for a long time)
    byte  numSensors(void);                         // Number of the sensors, registered in the pool
    byte  sensorList(byte ID[]);                    // Number of the sensors, and the sensor list from the pool
    int   temp(byte ID);                            // The temperature of the sensor ID
    byte  hum(byte ID);                             // The humidity of the sensor ID
    int   aTemp(byte ID);                           // The average temperature of the sensor ID
    byte  aHum(byte ID);                            // The average humidity of the sensor ID
    bool  isBattOK(byte ID);                        // Batery status of the sensor ID
  private:
    byte  registerSensor(byte ID);                  // Return sensor index of new registered sensor
    byte  index(byte ID);
    weatherSensor   s[max_sensors];                 // The sensors array
    byte            s_id[max_sensors];              // The sensor ID array, The internal sensor has ID = 0
    bool            in_use[max_sensors];            // Whether this sensor is currently in use
    const time_t    stale_time = 1800;              // timeout for the staled sensor data
};

void sensorPool::init(void) {
  for (byte i = 0; i < max_sensors; ++i) {
    s[i].init();
    in_use[i] = false;
  }
  // Register internal sensor with ID = 0
  s_id[0]   = 0;
  in_use[0] = true; 
}

void sensorPool::update(byte ID, int temp, byte hum, bool batt, bool reset) {
  byte indx = index(ID);
  if (indx >= max_sensors) {                        // The sensor is not in the list
    indx = registerSensor(ID);
    if (indx >= max_sensors) return;                // Failed to register new sensor, pool is full
  }
  s[indx].update(temp, hum, batt, reset);
}

byte sensorPool::next(byte ID) {
  byte indx = index(ID);
  if (indx >= max_sensors) return s_id[0];          // The sensor not found or this sensor is the last i the pool, return the 0-th sensor ID (internal one)
  ++indx;                                           // Start from the next sensor in the pool
  for (; indx < max_sensors; ++indx) {              // Check whether this sensor is in use
    if (in_use[indx]) return s_id[indx];
  }
  return s_id[0];
}

byte sensorPool::previous(byte ID) {
  char indx = index(ID);
  if (indx >= max_sensors) return s_id[0];          // The sensor not found or this sensor is the last i the pool, return the 0-th sensor ID (internal one)
  --indx;                                           // Start from the previous sensor in the pool
  if (indx < 0) indx = max_sensors-1;
  for (; indx >= 0; --indx) {                       // Check whether this sensor is in use
    if (in_use[byte(indx)]) return s_id[byte(indx)];
  }
  return s_id[0];
}

bool sensorPool::exists(byte ID) {
  byte indx = index(ID);
  return (indx < max_sensors);
}

void sensorPool::freeStaled(void) {
  time_t n = now();
  for (byte i = 0; i < max_sensors; ++i) {
    if (in_use[i]) {
      time_t lu = s[i].lastUpdated();
      if ((n - lu) >= stale_time) {                 // Mark the sensor free and clear the sensor data
        in_use[i] = false;
        s[i].init();
      }
    }
  }
}

byte sensorPool::numSensors(void) {
  byte n = 0;
  for (byte i = 0; i < max_sensors; ++i)
    if (in_use[i]) ++n;
  return n;
}

byte sensorPool::sensorList(byte ID[]) {
  for (byte i = 0; i < max_sensors; ++i) ID[i] = 0;
    
  byte n = 0;
  for (byte i = 0; i < max_sensors; ++i)
    if (in_use[i])
      ID[n++] = s_id[i];
  return n;
}

int sensorPool::temp(byte ID) {
  byte indx = index(ID);
  if (indx >= max_sensors) return 0;                // No sensor registered
  return s[indx].temp();
}

byte sensorPool::hum(byte ID) {
  byte indx = index(ID);
  if (indx >= max_sensors) return 0;                // No sensor registered
  return s[indx].hum();
}

int sensorPool::aTemp(byte ID) {
  byte indx = index(ID);
  if (indx >= max_sensors) return 0;                // No sensor registered
  return s[indx].aTemp();
}

byte sensorPool::aHum(byte ID) {
  byte indx = index(ID);
  if (indx >= max_sensors) return 0;                // No sensor registered
  return s[indx].aHum();
}

bool sensorPool::isBattOK(byte ID) {
  byte indx = index(ID);
  if (indx >= max_sensors) return false;            // No sensor registered
  return s[indx].isBattOK();
}

byte sensorPool::registerSensor(byte ID) {          // Return sensor index
  for (byte i = 0; i < max_sensors; ++i) {
    if (!in_use[i]) {
      s_id[i]   = ID;
      in_use[i] = true; 
      return i;
    }
  }
  return max_sensors;
}

byte sensorPool::index(byte ID) {                   // return the index of the sensor [0; max_sensors -1] or max_sensors if not found
  byte i = 0;
  for (; i < max_sensors; ++i)
    if (in_use[i] && s_id[i] == ID) break;
  return i;
}

//------------------------------------------ class Nextion screen via hardware serial port #2 ------------------
class NXT {
  public:
    NXT(void) { }
    void init(void);
    bool event(byte& page, byte& item);
    void turnPage(NxtPage p);
    void setBrightness(byte br);
    void showMainSensorData(int t, byte h);
    void showExtSensorData(byte ID, int t, byte h, bool battOK);
    void showIntSensorData(int t, byte h);
    void showPressure(int p);
    void showTime(time_t t = 0);
    void clearMainSensorData(void);
    void clearAll(void);
    void showSunData(time_t rise, time_t sset, byte mday);
    void showClockSetup(tmElements_t& tm, byte index);
    void clearGraphData(void);
    void showGraph(byte ID, byte mode, bool daily, byte sID[], byte s_num, byte hData[], int h_min, int h_max);
    void showConfig(struct cfg& Cfg);
    void showBrghtSetup(byte t_morning, byte t_evening, byte br_day, byte br_night, byte index);
    void showAllSensorData(byte s_num, byte s_id[], int s_temp[], byte s_humi[], byte main_id);
  private:
    byte verticalGrid(int interval);                // Calculate vertical grid space by value interval
    bool end_event(void);                           // Check than the message have been finished (3 0xff symbols received)
    void sendCmd(const char *cmd);                  // Send RAW command to nextion screen
    bool start_event;                               // Wether the press event was started (0x65 byte received)
    byte buff[7];                                   // Read buffer from nextion screen
    byte sym_indx;                                  // buff index for the next symbol
    char cmd_buff[30];
    byte page;
};

void NXT::init(void) {
  Serial2.begin(9600);
  start_event = false;
  sendCmd(" "); sendCmd(" ");
  page = NXT_MAXPAGE;
  turnPage(NXT_MAIN);
}

bool NXT::event(byte& page, byte& item) {
  while (Serial2.available() > 0) {

    byte c = Serial2.read();
    if (!start_event) {                             // No press event byte (0x65) have been received
      if (c == 0x65) {                              // Event press character received, start the event
        start_event = true;
        sym_indx = 0;
      }
    } else {
      buff[sym_indx] = c;
      if (c == 0xff) {                              // The finish sequence symbol
        if (end_event()) {
          page = buff[0];
          item = buff[1];
          start_event = false;
          return true;
        }
      }
      if (++sym_indx > 7) {
        start_event = false;
      }
    }

  }
  return false;
}

void NXT::turnPage(NxtPage p) {
  if ((page == p) || (p >= NXT_MAXPAGE)) return;
  sprintf(cmd_buff, "page %d", p);
  sendCmd(cmd_buff);
  page = p;
}

void NXT::showMainSensorData(int t, byte h) {
  // Display the temparature
  char sbuff[6];
  sbuff[0] = ' ';
  if (t < 0) {
    t *= -1;
    sbuff[0] = '-';
  }
  sprintf(&sbuff[1], "%d", t / 10);
  sprintf(cmd_buff, "m_temp.txt=\"%s.\"", sbuff);
  sendCmd(cmd_buff);
  sprintf(cmd_buff, "m_temp_dec.txt=\"%1d\"", t % 10);
  sendCmd(cmd_buff);

  // Display the humidity
  sprintf(cmd_buff, "m_hum.txt=\"%d%c\"", h, '%');
  sendCmd(cmd_buff);
}

void NXT::setBrightness(byte br) {
  if (br <= 100) {
    sprintf(cmd_buff, "dim=%d", br);
    sendCmd(cmd_buff);
  }
}

void NXT::showExtSensorData(byte ID, int t, byte h, bool battOK) {
  char batt = ' ';
  if (!battOK && ID) batt = '*';                    // Show only 'bad' battery status of the external sensor
  // The external sensor ID and battery status
  if (ID == 0)
    sprintf(cmd_buff, "extra_ID.txt=\"\"");
  else
    sprintf(cmd_buff, "extra_ID.txt=\"%x %c\"", ID, batt);
  sendCmd(cmd_buff);

  // The external sensor temparature
  char sbuff[6];
  sbuff[0] = ' ';
  if (t < 0) {
    t *= -1;
    sbuff[0] = '-';
  }
  sprintf(&sbuff[1], "%d.%1d", t / 10, t % 10);
  sprintf(cmd_buff, "extra_temp.txt=\"%s\"", sbuff);
  sendCmd(cmd_buff);

  // The external sensor humidity
  sprintf(cmd_buff, "extra_hum.txt=\"%d%c\"", h, '%');
  sendCmd(cmd_buff);
}

void NXT::showIntSensorData(int t, byte h) {
  // The internal sensor temparature
  char sbuff[6];
  sbuff[0] = ' ';
  if (t < 0) {
    t *= -1;
    sbuff[0] = '-';
  }
  sprintf(&sbuff[1], "%d.%1d", t / 10, t % 10);
  sprintf(cmd_buff, "int_temp.txt=\"%s\"", sbuff);
  sendCmd(cmd_buff);

  // The internal sensor humidity
  sprintf(cmd_buff, "int_hum.txt=\"%d%c\"", h, '%');
  sendCmd(cmd_buff);
}

void NXT::showTime(time_t t) {
  if (t == 0) t = now();
  tmElements_t  tm;                                 // time to be displayed
  breakTime(t, tm);

  sprintf(cmd_buff, "time.txt=\"%d:%02d\"", tm.Hour, tm.Minute);
  sendCmd(cmd_buff);

  sprintf(cmd_buff, "date.txt=\"%d.%02d\"", tm.Day, tm.Month);
  sendCmd(cmd_buff);
}

void NXT::showPressure(int p) {                     // Display the pressure
  sprintf(cmd_buff, "m_press.txt=\"%d\"", p);
  sendCmd(cmd_buff);
}

void NXT::clearMainSensorData(void) {
  sendCmd("m_temp.txt=\"\"");
  sendCmd("m_temp_dec.txt=\"\"");
  sendCmd("m_hum.txt=\"\"");
}

void NXT::clearAll(void) {
  clearMainSensorData();
  
  // clear external sensor data
  sendCmd("extra_ID.txt=\"\"");
  sendCmd("extra_temp.txt=\"\"");
  sendCmd("extra_hum.txt=\"\"");
  
  // clear internal sensor temparature
  sendCmd("int_temp.txt=\"\"");
  sendCmd("int_hum.txt=\"\"");

  // clear time and date
  sendCmd("time.txt=\"\"");
  sendCmd("date.txt=\"\"");

  // clear pressure
  sendCmd("m_press.txt=\"\"");
}

void NXT::showSunData(time_t rise, time_t sset, byte mday) {
  turnPage(NXT_SUNRISE);
  sprintf(cmd_buff, "wday.txt=\"%s,\"", wday[weekday()-1]);
  sendCmd(cmd_buff);
  sprintf(cmd_buff, "date.txt=\"%2d %s %4d\"", day(), months[(month()-1)], year());
  sendCmd(cmd_buff);
  sprintf(cmd_buff, "moon_day.txt=\"%2d\"", mday);
  sendCmd(cmd_buff);
  sprintf(cmd_buff, "rise.txt=\"%02d:%02d\"", hour(rise), minute(rise));
  sendCmd(cmd_buff);
  sprintf(cmd_buff, "set.txt=\"%02d:%02d\"", hour(sset), minute(sset));
  sendCmd(cmd_buff);
}

void NXT::showClockSetup(tmElements_t& tm, byte index) {
  // index: 0 - Hour, 1 - Minute, 2 - Day, 3 - Month, 4 - Year
  const char* elm[5] = {"hour", "minute", "day", "month", "year"};

  turnPage(NXT_CLOCK);

  char mon[4];
  for (byte i = 0; i < 3; ++i)
    mon[i] = months[tm.Month-1][i];
  mon[3] = '\0';
  
  for (byte i = 0; i < 5; ++i) {                    // Restore color of all elements, set color for selected element
    uint16_t color = 0xffff;
    if (i == index) color = 0x7e0;
    sprintf(cmd_buff, "%s.pco=%u", elm[i], color);
    sendCmd(cmd_buff);
  }

  sprintf(cmd_buff, "hour.txt=\"%02d\"", tm.Hour);
  sendCmd(cmd_buff);
  sprintf(cmd_buff, "minute.txt=\"%02d\"", tm.Minute);
  sendCmd(cmd_buff);

  sprintf(cmd_buff, "day.txt=\"%2d\"", tm.Day);
  sendCmd(cmd_buff);
  sprintf(cmd_buff, "month.txt=\"%s\"", mon);
  sendCmd(cmd_buff);
  sprintf(cmd_buff, "year.txt=\"%4d\"", tm.Year + 1970);
  sendCmd(cmd_buff);

  sprintf(cmd_buff, "wday.txt=\"%s\"", wday[tm.Wday-1]);
  sendCmd(cmd_buff);
}

void NXT::clearGraphData(void) {
    sendCmd("cle 1,255");                             // clear all data in the waveform
}

void NXT::showGraph(byte ID, byte mode, bool daily, byte sID[], byte s_num, byte hData[], int h_min, int h_max) {
  const char* mode_name[3] = {
    "\xC2\xD5\xDc\xDF\xD5\xE0\xD0\xE2\xE3\xE0\xD0", "\xB4\xD0\xD2\xDB\xD5\xDD\xD8\xD5",
    "\xB2\xDB\xD0\xD6\xDD\xDE\xE1\xE2\xEC"
  };

  byte p   = 1;                                     // Graph period in days
  byte gdw = 144;                                   // Show horizontal grid every 12 hours
  if (!daily) {
    p   = 8;
    gdw = 18;
  }
  byte gdh = verticalGrid(h_max-h_min);             // Vertical grid


  // Show the Garaph title
  sprintf(cmd_buff, "label.txt=\"%s %d%c.\"", mode_name[mode], p, '\xD4');
  sendCmd(cmd_buff);

//  sendCmd("cle 1,255");                             // clear all data in the waveform
  
  // Draw horizontal grid and vertical grid
  sprintf(cmd_buff, "data.gdw=%d%", gdw);
  sendCmd(cmd_buff);
  sprintf(cmd_buff, "data.gdh=%d%", gdh);
  sendCmd(cmd_buff);

  // Show sensors IDs
  for (byte i = 0; i < s_num; ++i) {
    uint16_t color = 0;
    if (ID == sID[i]) color = 31;                   // Hightlight the current sensor
    sprintf(cmd_buff, "s%d.pco=%u", i, color);
    sendCmd(cmd_buff);
    sprintf(cmd_buff, "s%d.txt=\"%x\"", i, sID[i]);
    sendCmd(cmd_buff);
  }
  for (byte i = s_num; i < max_sensors; ++i) {
    sprintf(cmd_buff, "s%d.txt=\"\"", i);
    sendCmd(cmd_buff);
  }

  for (uint16_t i = 0; i < h_samples; ++i) {        // Show the graph data
    sprintf(cmd_buff, "add 1,0,%d", hData[i]);
    sendCmd(cmd_buff);
  }

  // Show minimum and maximum values
  if (mode == 0) {                                  // The temperature in Celsius * 10
    sprintf(cmd_buff, "bottom.txt=\"%d.%d\"", h_min/10, abs(h_min) % 10);
    sendCmd(cmd_buff);
    sprintf(cmd_buff, "top.txt=\"%d.%d\"",    h_max/10, abs(h_max) % 10);
    sendCmd(cmd_buff);
  } else {                                          // Pressure or humidity
    sprintf(cmd_buff, "bottom.txt=\"%d\"", h_min);
    sendCmd(cmd_buff);
    sprintf(cmd_buff, "top.txt=\"%d\"",    h_max);
    sendCmd(cmd_buff);
  }
}

void NXT::showConfig(struct cfg& Cfg) {
  sprintf(cmd_buff, "t_brightness.txt=\"%d/%d\"", Cfg.br_day, Cfg.br_night);
  sendCmd(cmd_buff);
  sprintf(cmd_buff, "t_sensor.txt=\"%x\"", Cfg.ext_sensor_ID);
  sendCmd(cmd_buff);
  sprintf(cmd_buff, "t_time.txt=\"%d:%02d\"", hour(), minute());
  sendCmd(cmd_buff);
}

void NXT::showBrghtSetup(byte t_morning, byte t_evening, byte br_day, byte br_night, byte index) {
  const char* elm[4] = {"morning", "evening", "b_day", "b_night"};
  // Hightlight active item
  // index: 0 - morning, 1 - evening, 2 - daily brightness, 3 - nightly brightness
  for (byte i = 0; i < 4; ++i) {
    uint16_t c = 0xffff;                            // White
    if (i == index) c = 31;
    sprintf(cmd_buff, "%s.pco=%u", elm[i], c);
    sendCmd(cmd_buff);
  }
  
  // Show morning and evening times
  sprintf(cmd_buff, "morning.txt=\"%d:%02d\"", t_morning / 6, (t_morning %6) * 10);
  sendCmd(cmd_buff);
  sprintf(cmd_buff, "evening.txt=\"%d:%02d\"", t_evening / 6, (t_evening %6) * 10);
  sendCmd(cmd_buff);

  // Show day and night brightness
  sprintf(cmd_buff, "b_day.txt=\"%d\"", br_day);
  sendCmd(cmd_buff);
  sprintf(cmd_buff, "b_night.txt=\"%d\"", br_night);
  sendCmd(cmd_buff);

  if (index == 2) {
    sprintf(cmd_buff, "dim=%d", br_day);
    sendCmd(cmd_buff);
  } else if (index == 3) {
    sprintf(cmd_buff, "dim=%d", br_night);
    sendCmd(cmd_buff);
  }
  
}

void NXT::showAllSensorData(byte s_num, byte s_id[], int s_temp[], byte s_humi[], byte main_id) {
  for (byte i = 0; i < max_sensors - 1; ++i) {
    if (i < s_num) {
      uint16_t c = 0x0;                             // Black
      if (s_id[i] == main_id) c = 31;               // Blue
      sprintf(cmd_buff, "s%d.pco=%u", i+1, c);
      sendCmd(cmd_buff);
      sprintf(cmd_buff, "s%d.txt=\"%2x\"", i+1, s_id[i]);
      sendCmd(cmd_buff);
      sprintf(cmd_buff, "data%d.pco=%u", i+1, c);
      sendCmd(cmd_buff);
      sprintf(cmd_buff, "data%d.txt=\"%3d.%d, %d%c\"", i+1, s_temp[i]/10, abs(s_temp[i]) % 10, s_humi[i], '%');
      sendCmd(cmd_buff);
    } else {
      sprintf(cmd_buff, "s%d.txt=\"\"", i+1);
      sendCmd(cmd_buff);
      sprintf(cmd_buff, "data%d.txt=\"\"", i+1);
      sendCmd(cmd_buff);
    }
  }
}

void NXT::sendCmd(const char *cmd) {
  Serial2.print(cmd);
//Serial.print("cmd: "); Serial.println(cmd);
  Serial2.print("\xff\xff\xff");
}

bool NXT::end_event(void) {
  if (sym_indx != 5) return false;
  bool ok = true;
  for (char i = sym_indx; i > sym_indx - 3; --i) {
    if (buff[byte(i)] != 0xff) {
      ok = false;
      break;
    }
  }
  return ok;
}

byte NXT::verticalGrid(int interval) {
  if (interval <= 10) return (gh / interval);
  byte remainder[8];
  for (int i = 7; i >= 0; --i) {                   // Trying to calculate maximal divider of the interval. [3..13]
    int r = interval % (i+3);
    if (r == 0) {                                   // We have found the divider
      return (gh / (i+3));                          // Grid is multiple to the interval
    }
    remainder[i] = min(r, i+3 - r);
  }
  // If the interval is not multiple to any nomber [3..13], select the number with minimal remainder
  byte indx = 12;
  byte min_val = 255;
  for (byte i = 0; i < 7; ++i) {
    if (remainder[i] <= min_val) {
      min_val = remainder[i];
      indx = i + 3; 
    }
  }
  return (gh / indx);
}

//------------------------------------------ class SCREEN ------------------------------------------------------
class SCREEN {
  public:
    SCREEN() {
      update_screen  = 0;
      scr_timeout    = 0;
      time_to_return = 0;
    }
    virtual void    init(void)                  { }
    virtual void    show(void)                  { }
    virtual NxtPage event(byte page, byte item) { return NXT_MAIN; }    // By default, return to the main screen
    virtual bool      returnToMain(void);
    bool              isSetup(void)             { return (scr_timeout != 0); }
    void              forceRedraw(void)         { update_screen = 0; }
    void              resetTimeout(void);
    bool              wasRecentlyReset(void);
    void              setSCRtimeout(uint16_t t);
  protected:
    uint32_t update_screen;                     // The time (ms) when the screen was updated last time
    uint32_t scr_timeout;                       // Timeout is sec. to return to the main screen, canceling all changes
    uint32_t time_to_return;                    // Time in ms to return to main screen
    NxtPage  nxt_index;                         // The nextion screen index
};

bool SCREEN::returnToMain(void) {
  if (scr_timeout && (millis() >= time_to_return)) {
    scr_timeout = 0;
    return true;
  }
  return false;
}

void SCREEN::resetTimeout(void) {
  if (scr_timeout > 0)
    time_to_return = millis() + scr_timeout*1000;
}

void SCREEN::setSCRtimeout(uint16_t t) {
  scr_timeout = t;
  resetTimeout();
}

bool SCREEN::wasRecentlyReset(void) {
  uint32_t to = (time_to_return - millis()) / 1000;
  return((scr_timeout - to) < 15);
}

//------------------------------------------ class histSCREEN, show the data history ----------------------------------
class histSCREEN : public SCREEN {
  public:
    histSCREEN(NxtPage NXT_index, NXT* nxt, weatherLogger* WL, sensorPool* pSP) {
      nxt_index   = NXT_index;
      pD          = nxt;
      pWl         = WL;
      pSp         = pSP;
      ID          = 0;
      mode        = 0;
    }
    virtual void    init(void);
    virtual void    show(void);
    virtual NxtPage event(byte page, byte item);
    void            setGraph(byte graph_sensor, byte graph_mode);
  private:
    bool loadHistoryData(byte &sensorID, byte &t, byte &p, byte &h, time_t &timestamp);
    void rebuildHistory(void);
    NXT*           pD;                                // Pointer to the screen instance
    weatherLogger* pWl;                               // Pointer to the weatherLogger instance
    sensorPool*    pSp;                               // Pointer to the sensor pool instance
    byte           sID[max_sensors];                  // All available sensors ID
    byte           s_num;                             // Number of the registered sensors 
    byte           hData[h_samples];                  // Sensor values saved every 15 minutes, 96 samples per day
    byte           hdn[h_samples];                    // The number of samples per graph point
    byte           h_min;                             // minimal values for temperature, pressure or humidity
    byte           h_max;                             // maximal values for temperature, pressure or humidity
    uint16_t       period;                            // the interval bettwen history data (in seconds) 900 or 7200
    byte           ID;                                // the sensor ID to display history
    byte           mode;                              // The graph mode: 0 - temperature, 1 - pressure, 2 - humidity
    time_t         start_time;                        // the time of the first history sample
    const uint32_t refresh     = 40000;               // The period in ms to refresh the screen
    const uint16_t day_period  = 86400/h_samples;     // Period (s) between two samples in the day statistics. (h_samples = 288, day_period = 5 minutes)
    const uint16_t week_period = 8*day_period;        // Period (s) between two samples in the 8-days statistics.
};

void histSCREEN::init(void) {
  period = day_period;                                // Show the daily statistic, 5 - minutes saples
  rebuildHistory();
  setSCRtimeout(30);
  update_screen = 0;
  pD->turnPage(nxt_index);
  s_num = pSp->sensorList(sID);                       // To display on the graph
}

void histSCREEN::setGraph(byte graph_sensor, byte graph_mode) {
  ID   = graph_sensor;
  mode = graph_mode;
}

void histSCREEN::show(void) {
  uint32_t nowMS = millis();
  if (update_screen && ((nowMS - update_screen) < refresh)) return;
  update_screen = nowMS;

  int h_bot, h_top;
  switch (mode) {
    case 0:                                       // The temperature. The data in the history is the temperature * 5 plus 500
      h_bot = (int(h_min) << 2) - 500;            // Minimum temperature * 10
      h_top = (int(h_max) << 2) - 500;            // Maximum temperature * 10
      break;
    case 1:                                       // The pressure. The data in the history is the pressure - 600
      h_bot = int(h_min) + 600;                   // Minimum pressure
      h_top = int(h_max) + 600;                   // Maximum pressure
      break;
    case 2:
    default:
      h_bot = h_min;                              // Minimum humidity
      h_top = h_max;                              // Maximum humidity
      break;
  }
  pD->showGraph(ID, mode, (period == day_period), sID, s_num, hData, h_bot, h_top);
}

void histSCREEN::rebuildHistory(void) {
  time_t ts;

    // Clear the history data
  pD->clearGraphData();
  h_min = 255;
  h_max = 0;
  for (uint16_t i = 0; i < h_samples; ++i) {
    hData[i] = 0;
    hdn[i]   = 0;
  }

  // Calculate the history start time
  start_time = now() - (uint32_t)period * h_samples;
  uint16_t next = start_time % period;
  start_time -= next;

  byte rdata[3];                                      // temporary buffer to read the data: temp, pressure and humidity
  byte sensorID = 0;
  // New log file created at midninghts; Several log files can be used to display the history data 
  for (time_t curr_log = start_time; curr_log < now(); curr_log += 86400) {
    if (!pWl->openLog(curr_log)) continue;            // Skip the day history period, because no data file opened for this day
    while (loadHistoryData(sensorID, rdata[0], rdata[1], rdata[2], ts) ) {
    if (sensorID != ID) continue;
      ts -= second(ts);                               // Round the history data to the minute
      // Calculate position of the data in the graph; Data must fit to the time grid
      if (ts < start_time) continue;                  // This data time stamp is before start time
 
      uint16_t indx = (ts - start_time) / period;         // Grid index of the read data
      if (indx >= h_samples) continue;                // Only 96 values fit the graph

      // If several data fit timestamp of the grid, use avarage value (on week graph)
      uint16_t d = uint16_t(hData[indx]) * hdn[indx] + uint16_t(rdata[mode]);
      d /= (hdn[indx] + 1);
      hData[indx] = byte(d);
      if (d < h_min) h_min = d;
      if (d > h_max) h_max = d;
      ++hdn[indx];
    }
  }                                                   // The end of the 'day loop'

  if (h_min > h_max) {                                // There was no history data read
    h_min = h_max = 0;
    return;
  }

  // Fill the empty data in the begining of the graph with average value
  uint16_t l = 0;
  for ( ; ((l < h_samples) && (hdn[l] == 0)); ++l)
    hData[l] = ((uint16_t)h_min + (uint16_t)h_max) >> 1;
  // Fill the empty data inside the graph with the average data
  for ( ; l < h_samples-1; ++l) {
    if (hdn[l+1] != 0) continue;                      // Next sample loaded, move the left border
    uint16_t r = l+1;                                 // Looking for next non-empty data
    for ( ; ((r < h_samples) && (hdn[r] == 0)); ++r);
    if (r < h_samples) {                              // The right boarder is insede the sample interval
      float k = (float(hData[r]) - float(hData[l])) / float(r - l);
      for (int i = 1; i < r - l; ++i) {               // Linear approximation between left and right boarders
        float v = float(hData[l]) + k * i + 0.5;
        hData[l+i] = byte(v);
      }
      l = r-1;
    } else {                                          // No more samples till the end of interval
      byte v = hData[l];
      for ( ; l < h_samples; ++l)
        hData[l] = v;
    }
  }

  // Normalize the data [min; max] -> [0; gh]
  if (h_max > h_min) {
    float k = (float)gh / ((float)h_max - (float)h_min);
    for (uint16_t i = 0; i < h_samples; ++i) {
      float nd = (float)hData[i] - (float)h_min;
      nd *= k;
      nd += 0.5;
      int ndi = (int) nd;
      if (ndi < 0) ndi = 0; else if (ndi > gh) ndi = gh;
      hData[i] = ndi;
    }
  } else {                                          // Constant data
    for (uint16_t i = 0; i < h_samples; ++i) {
      hData[i] = gh>>1;
    }
  }
}

bool histSCREEN::loadHistoryData(byte &sensorID, byte &t, byte &p, byte &h, time_t &timestamp) {
  int t1, p1;
  if (pWl->readData(sensorID, t1, p1, h, timestamp)) {
    t1 += 500;                                        // t1 was: -500 <= t1 <= 500 (in centegrees * 10)
    t = t1>>2;                                        // t is in centegrees * 2.5
    p = p1 - 600;
    return true;
  }
  return false;
}

NxtPage histSCREEN::event(byte page, byte item) {
  if ((page == nxt_index) && (item <= 10)) {
    switch (item) {
      case 1:                                         // The graph itself, change period
        if (period == day_period) {                   // switch between day and week period
          period = week_period;
        } else {
          period = day_period;
        }
        break;
      case 2:                                         // Title, return to the main screen
        return NXT_MAIN;
      case 3:                                         // pressure
        ID   = 0;
        mode = 1;
        break;
      case 4:                                         // temperature
        mode = 0;
        break;
      case 5:                                         // humidity
        mode = 2;
        break;
      default:                                        // 6-10 - sensor index
        if ((item < 6) || (item >= s_num+6)) {
          SCREEN::resetTimeout();
          return nxt_index;
        }
        item -= 6;
        ID = sID[item];
        if (mode == 1) mode = 0;
        break;
    }
    rebuildHistory();
    update_screen = 0;                                // Force refresh the screen
  }
  SCREEN::resetTimeout();
  return nxt_index;
}

//------------------------------------------ class mainSCREEN ---------------------------------------------------------
class mainSCREEN : public SCREEN {
  public:
    mainSCREEN(NxtPage NXT_index, NXT* nxt, sensorPool* pSP, pressureSensor* pPS, sunMoon* pSM, histSCREEN* HS, CONFIG* CFG) {
      nxt_index   = NXT_index;
      pD          = nxt;
      pSp         = pSP;
      pPs         = pPS;
      pSm         = pSM;
      ph          = HS;
      pCfg        = CFG;
      main_sensor = 0;
      main_sensor_manual = false;
    }
    virtual void    init(void);
    virtual void    show(void);
    virtual NxtPage event(byte page, byte item);

  private:
    void  showExtSensor(void);                        // Show external sensor data (right side of the screen)
    bool  getMainSensorData(int& temp, byte& humidity);
    bool  isDark(void);
    NXT*             pD;                              // Pointer to the Nextion display
    sensorPool*      pSp;                             // Pointer to the sensor pool instance
    sunMoon*         pSm;                             // Pointer to the sunMoon class instance
    pressureSensor*  pPs;                             // Pointer to the Pressure sensor based on BMP280
    histSCREEN*      ph;                              // Pointer to the History screen
    CONFIG*          pCfg;                            // Pointer to the configuration
    byte             main_sensor;                     // The ID of the main external sensor
    bool             main_sensor_manual;              // Whether the main sensor ID was setup manually
    byte             ext_sensor;                      // external sensor ID to be displayed
    byte             screen_brightness;               // The screen brightness has been setup
    const uint16_t   period = 15000;                  // The screen update period
};

void mainSCREEN::init(void) {
  update_screen = 0;
  ext_sensor = 0;
  screen_brightness = 101;                            // Invalid brightness indicates, the brightness should be setup once more
  struct cfg conf;
  pCfg->getConfig(conf);
  if (conf.ext_sensor_ID) {
    main_sensor = conf.ext_sensor_ID;
    main_sensor_manual = true;
  } else {
    main_sensor = 0;
    main_sensor_manual = false;
  }
  pD->turnPage(nxt_index);                            // Nextion index for main scren depands on weather forecast. [0..7]
}

NxtPage mainSCREEN::event(byte page, byte item) {
  bool show_graph = false;
  byte graph_sensor = 0;
  byte graph_mode   = 1;                              // 0 - temperature, 1 - pressure, 2 - humidity

  if ((page == nxt_index) && (item <= 11)) {
    switch (item) {
      case 1:                                         // Pressure selected
        show_graph = true;
        break;
      case 2:                                         // Main sensor temperature selected
        if (pSp->exists(main_sensor)) {
          show_graph   = true;
          graph_sensor = main_sensor;
          graph_mode   = 0;
        }
        break;
      case 4:                                         // Main sensor humidity selected
        if (pSp->exists(main_sensor)) {
          show_graph   = true;
          graph_sensor = main_sensor;
          graph_mode   = 2;
        }
        break;
      case 5:                                         // External sensor temperature selected
        show_graph   = true;
        graph_sensor = ext_sensor;
        graph_mode   = 0;
        break;
      case 6:                                         // External sensor humidity selected
        show_graph   = true;
        graph_sensor = ext_sensor;
        graph_mode   = 2;
        break;
      case 8:                                         // Internal temterature selected
        show_graph   = true;
        graph_mode   = 0;
        break;
      case 9:                                         // Internal humidity selected
        show_graph   = true;
        graph_mode   = 2;
        break;  
      case 10:                                        // Time selecterd, run main setup
        return NXT_MENU;
      case 11:                                        // Date selected, show sinrise ans sinset times
        return NXT_SUNRISE;
      default:
        break;
    }
  }

  if (show_graph) {
    ph->setGraph(graph_sensor, graph_mode);
    return NXT_GRAPH;
  }
  return nxt_index;
}

void mainSCREEN::show(void) {
  
  uint32_t nowMS = millis();
  if (update_screen && ((nowMS - update_screen) < period)) return;
  update_screen = nowMS;

  // Switch nextion screen accordingly with the weather forecast
  bool is_cold   = (pSp->temp(main_sensor) < 0);
  nxt_index = pPs->forecast(is_cold, isDark());
  pD->turnPage(nxt_index);
  
  pD->showTime();                                     // Show current time and date

  // Show the main sensor data
  int  t = 0;
  byte h = 0;
  if (getMainSensorData(t, h))
    pD->showMainSensorData(t, h);
  else
    pD->clearMainSensorData();

  pD->showPressure(pPs->press());                     // Show the current pressure

  // Show internal sensor data
  if (pSp->exists(0)) {
    t = pSp->temp(0);
    h = pSp->hum(0);
    pD->showIntSensorData(t, h);
  }

  showExtSensor();                                    // Show external sensor data in the right side of the screen

  // Setup screen brightness
  struct cfg conf;
  pCfg->getConfig(conf);
  time_t tm = now();
  tm %= 86400;                                         // Time since midnight
  uint32_t morning = uint32_t(conf.backlight_morning) * 600;
  uint32_t evening = uint32_t(conf.backlight_evening) * 600;
  byte br = conf.br_night;
  if ((morning < tm) && (tm < evening))
    br = conf.br_day;
  if (br != screen_brightness) {
    pD->setBrightness(br);
    screen_brightness = br;
  }
}

bool mainSCREEN::getMainSensorData(int& temp, byte& humidity) {
  if (!main_sensor_manual && main_sensor == 0) {      // Main sensor ID was not set nor selected, select the sensor from the pool
    byte n = pSp->numSensors();
    if (n < 2) return false;                          // Only internal sensor exists; Failed to load sensor data
    for (byte i = 0; i < n; ++i) {
      main_sensor = pSp->next(main_sensor);
      if (main_sensor > 0) break;                     // Select external sensor automatically
    }
  }
  if (pSp->exists(main_sensor)) {
    temp     = pSp->temp(main_sensor);
    humidity = pSp->hum(main_sensor);
    return true;
  } else {
    if (!main_sensor_manual)
      main_sensor = 0;
  }
  return false;
}

void mainSCREEN::showExtSensor(void) {
  byte num_sensors = pSp->numSensors();
  
  switch (num_sensors) {
    case 0:
      return;

    case 1:                                           // Internal sensor only, show internal sensor data in the right side
      ext_sensor     = 0;
      break;

    case 2:                                           // Internal + 1 external sensor, show internal sensor data also
      ext_sensor = pSp->next(ext_sensor);
      break;

    default:                                           // do not show internal sensor
      for (byte i = 0; i < num_sensors; ++i) {
        ext_sensor = pSp->next(ext_sensor);
        if (ext_sensor) break;
      }
      break;
  }

  int  t    = pSp->temp(ext_sensor);
  byte h    = pSp->hum(ext_sensor);
  bool batt = pSp->isBattOK(ext_sensor);
  pD->showExtSensorData(ext_sensor, t, h, batt);
}

bool mainSCREEN::isDark(void) {
  static time_t last_day = 0;
  static time_t sr = 0;
  static time_t ss = 0;

  time_t t = now();
  time_t ld = t / 86400;                              // 24 hours in seconds
  if (ld != last_day) {
    last_day = ld;
    sr = pSm->sunRise();
    ss = pSm->sunSet();
  }
  return ((t < sr) || (t > ss));
}

//------------------------------------------ class menuSCREEN show main menu ------------------------------------------
class menuSCREEN: public SCREEN {
  public:
    menuSCREEN(NxtPage NXT_index, NXT* nxt, CONFIG* CFG) : SCREEN() {
      nxt_index   = NXT_index;
      pD          = nxt;
      pCfg        = CFG;
    };
    virtual void    init(void);
    virtual NxtPage event(byte page, byte item);
  private:
    NXT*          pD;                                 // Pointer to the Screen instance
    CONFIG*       pCfg;                               // Pointer to the configuration instance
    const uint32_t period = 40000;
};

void menuSCREEN::init(void) {     
  update_screen = 0;
  setSCRtimeout(20);
  pD->turnPage(nxt_index);
  struct cfg Cfg;
  pCfg->getConfig(Cfg);
  pD->showConfig(Cfg);
}

NxtPage menuSCREEN::event(byte page, byte item) {
  
  if ((page == nxt_index) && (item <= 6)) {
    switch (item) {
      case 2:                                         // select main sensor
        return NXT_SENSOR;
      case 3:                                         // configure brightness
        return NXT_BRIGHT;
      case 4:                                         // setup the clock
        return NXT_CLOCK;
      case 5:                                         // Apply the changes into the config
        pCfg->save();                                 // Write current config to the EEPROM 
        return NXT_MAIN;
        break;
      case 6:                                         // Cancel the changes, reload previous config
        pCfg->init();
        pCfg->load();
        return NXT_MAIN;
      default:
        break;
    }
  }
  SCREEN::resetTimeout();
  return nxt_index;
}

//------------------------------------------ class menuSCREEN show main menu ------------------------------------------
class brghSCREEN: public SCREEN {
  public:
    brghSCREEN(NxtPage NXT_index, NXT* nxt, CONFIG* CFG) : SCREEN() {
      nxt_index   = NXT_index;
      pD          = nxt;
      pCfg        = CFG;
    };
    virtual void    init(void);
    virtual void    show(void);
    virtual NxtPage event(byte page, byte item);
  private:
    void          changeValue(bool increment);        // Change time elment selected by index
    NXT*          pD;                                 // Pointer to the Screen instance
    CONFIG*       pCfg;                               // Pointer to the config
    byte          index;                              // 0 - morning, 1 - evening, 2 - daily brightness, 3 - nightly brightness
    struct cfg    conf;                               // Configuration data
    const uint32_t period = 40000;
};

void brghSCREEN::init(void) {     
  pCfg->getConfig(conf);
  index = 0;
  update_screen = 0;
  setSCRtimeout(20);
  pD->turnPage(nxt_index);
}

void brghSCREEN::show(void) {

  uint32_t nowMS = millis();
  if (update_screen && ((nowMS - update_screen) < period)) return;
  update_screen = nowMS;

  pD->showBrghtSetup(conf.backlight_morning, conf.backlight_evening, conf.br_day, conf.br_night, index);
}

NxtPage brghSCREEN::event(byte page, byte item) {
  if ((page == nxt_index) && (item <= 13)) {
    switch (item) {
      case 2:                                         // minus selected
        changeValue(false);
        break;
      case 3:                                         // plus selected
        changeValue(true);
        break;
      case 6:                                         // morning time selected
        index = 0;
        break;
      case 7:                                         // evening time selected
        index = 1;
        break;
      case 12:                                        // nightly brightness selected
        index = 3;
        break;
      case 13:                                        // daily brightness selected
        index = 2;
        break;
      case 8:                                         // Cancel
        return NXT_MENU;
      case 9:                                         // Apply the changes
        pCfg->updateConfig(conf);
        return NXT_MENU;
      default:
        break;
    }
  }
  update_screen = 0;                                  // Force refresh the screen
  SCREEN::resetTimeout();
  return nxt_index;
}

void brghSCREEN::changeValue(bool increment) {        // index: 0 - morning, 1 - evening, 2 - daily brightness, 3 - nightly brightness
  switch (index) {
    case 0:
      if (increment && (conf.backlight_morning < 144)) ++conf.backlight_morning;
      else if (conf.backlight_morning > 0) --conf.backlight_morning;
      break;
    case 1:
      if (increment && (conf.backlight_evening < 144)) ++conf.backlight_evening;
      else if (conf.backlight_evening > 0) --conf.backlight_evening;
      break;
    case 2:
      if (increment && (conf.br_day < 100)) ++conf.br_day;
      else if (conf.br_day > 0) --conf.br_day;
      break;
    case 3:
      if (increment && (conf.br_night < 100)) ++conf.br_night;
      else if (conf.br_night > 0) --conf.br_night;
      break;
    default:
      break;
  }
}

//------------------------------------------ class sensSCREEN show select main sensor menu ----------------------------
class sensSCREEN: public SCREEN {
  public:
    sensSCREEN(NxtPage NXT_index, NXT* nxt, CONFIG* CFG, sensorPool* SPool) : SCREEN() {
      nxt_index   = NXT_index;
      pD          = nxt;
      pCfg        = CFG;
      pSp         = SPool;
    };
    virtual void    init(void);
    virtual void    show(void);
    virtual NxtPage event(byte page, byte item);
  private:
    NXT*          pD;                                 // Pointer to the Screen instance
    CONFIG*       pCfg;                               // Pointer to the config instance
    sensorPool*   pSp;                                // Pointer to the sensor pool
    byte          s_num;                              // The number of the external sensors
    byte          s_main;                             // Main sensor ID
    byte          s_id[max_sensors];                  // Available sensor ID
    int           s_temp[max_sensors];                // The temperature of the sensors
    byte          s_humi[max_sensors];                // The humidity of the sensors
    const uint32_t period = 40000;
};

void sensSCREEN::init(void) {    
  update_screen = 0;
  setSCRtimeout(20);
  s_num = pSp->numSensors() - 1;                      // Exclude internal sensor from the list
  byte id = 0;
  for (byte i = 0; i < s_num; ++i) {
    if (id = pSp->next(id)) {
      s_id[i]   = id;
      s_temp[i] = pSp->temp(id);
      s_humi[i] = pSp->hum(id);
    } else break;
  }
  struct cfg Cfg;
  pCfg->getConfig(Cfg);
  s_main = Cfg.ext_sensor_ID;
  pD->turnPage(nxt_index);
}

void sensSCREEN::show(void) {

  uint32_t nowMS = millis();
  if (update_screen && ((nowMS - update_screen) < period)) return;
  update_screen = nowMS;

  pD->showAllSensorData(s_num, s_id, s_temp, s_humi, s_main);
}

NxtPage sensSCREEN::event(byte page, byte item) {
  struct cfg Cfg;
  
  if ((page == nxt_index) && (item <= 10)) {
    switch (item) {
      case 10:                                        // 'auto' selected
        pCfg->getConfig(Cfg);
        Cfg.ext_sensor_ID = 0;
        pCfg->updateConfig(Cfg);
        return NXT_MENU;
      default:
        if ((item >= 1) && (item <= s_num)) {         // Sensor index selected
          pCfg->getConfig(Cfg);
          Cfg.ext_sensor_ID = s_id[item-1];
          pCfg->updateConfig(Cfg);
          return NXT_MENU;
        }
        break;
    }
  }
  update_screen = 0;                                  // Force refresh the screen
  SCREEN::resetTimeout();
  return nxt_index;
}

//------------------------------------------ class timeSerSCR, setup clock --------------------------------------------
class timeSetSCR: public SCREEN {
  public:
    timeSetSCR(NxtPage NXT_index, NXT* nxt) : SCREEN() {
      nxt_index   = NXT_index;
      pD          = nxt;
      index = 0;
    };
    virtual void    init(void);
    virtual void    show(void);
    virtual NxtPage event(byte page, byte item);
  private:
    void          changeValue(bool increment);        // Change time elment selected by index
    NXT*          pD;                                 // Pointer to the Screen instance
    byte          index;                              // 0 - Hour, 1 - Minute, 2 - Day, 3 - Month, 4 - Year
    tmElements_t  tm;                                 // time to be set-up
    const uint32_t period = 30000;
};

void timeSetSCR::init(void) {  
  RTC.read(tm);                                       // Initialize the time to be set up with the current time
  index = 0;
  update_screen = 0;
  setSCRtimeout(20);
  pD->turnPage(nxt_index);
}

void timeSetSCR::show(void) {

  uint32_t nowMS = millis();
  if (update_screen && ((nowMS - update_screen) < period)) return;
  update_screen = nowMS;

  pD->showClockSetup(tm, index);
}

NxtPage timeSetSCR::event(byte page, byte item) {
  if ((page == nxt_index) && (item <= 10)) {
    switch (item) {
      case 1:                                         // day selected
        index = 2;
        break;
      case 2:                                         // month selected
        index = 3;
        break;
      case 3:                                         // year selected
        index = 4;
        break;
      case 4:                                         // Hour selected
        index = 0;
        break;
      case 5:                                         // Minute selected
        index = 1;
        break;
      case 7:                                         // Apply selected, setup new time
        tm.Second = 0;
        RTC.write(tm);
        setTime(RTC.get());
        return NXT_MAIN;
      case 8:                                         // Cancel selected, return to main screen
        return NXT_MAIN;
      case 9:                                         // Minus selecterd, decrement current value
        changeValue(false);
        break;
      case 10:                                        // Plus selected, increment current value
        changeValue(true);
        break;
      default:
        break;
    }
  }
  update_screen = 0;                                  // Force refresh the screen
  SCREEN::resetTimeout();
  return nxt_index;
}

void timeSetSCR::changeValue(bool increment) {        // index: 0 - Hour, 1 - Minute, 2 - Day, 3 - Month, 4 - Year
  byte *p = 0;
  switch (index) {
    case 0:
      p = &tm.Hour;
      break;
    case 1:
      p = &tm.Minute;
      break;
    case 2:
      p = &tm.Day;
      break;
    case 3:
      p = &tm.Month;
      break;
    case 4:
      p = &tm.Year;
      break;
    default:
      break;
  }
  if (p) {
    if (increment)
      (*p)++;
    else
      (*p)--;
    time_t nt = makeTime(tm);
    breakTime(nt, tm);
  }
}

//------------------------------------------ class sunSCREEN, show sunrise and sunset times ---------------------------
class sunSCREEN : public SCREEN {
  public:
    sunSCREEN(NxtPage NXT_index, NXT* nxt, sunMoon* pSM) {
      nxt_index = NXT_index;
      pD        = nxt;
      pSm       = pSM;
    }
    virtual void show(void);
    virtual void init(void);
  private:
    NXT*     pD;                                      // Pointer to the Screen instance
    sunMoon* pSm;                                     // Pointer to the sunMoon instance
    const uint32_t period = 60000;
};

void sunSCREEN::init(void) {
  setSCRtimeout(30);
  update_screen = 0;
  pD->turnPage(nxt_index);
}

void sunSCREEN::show(void) {

  uint32_t nowMS = millis();
  if (update_screen && ((nowMS - update_screen) < period)) return;
  update_screen = nowMS;

  time_t rise = pSm->sunRise(); rise += 30;
  time_t sset = pSm->sunSet();  sset += 30;
  byte   mday = pSm->moonDay();
  pD->showSunData(rise, sset, mday);
}

//============================================ End of class declarations =======================================
NXT                     nxt;
DHT                     dht(DHT_PIN, DHTTYPE);
Adafruit_BMP280         bmp;
OregonDecoderV2         orscV2;
sunMoon                 sm;
pressureSensor          pSensor(&bmp);
weatherLogger           wLogger;
sensorPool              sPool;
HB                      hb(hbPIN);
CONFIG                  stationCfg;

histSCREEN              histScr(NXT_GRAPH, &nxt, &wLogger, &sPool);
mainSCREEN              mainScr(NXT_MAIN, &nxt, &sPool, &pSensor, &sm, &histScr, &stationCfg);
menuSCREEN              menuScr(NXT_MENU, &nxt, &stationCfg);
brghSCREEN              brightScr(NXT_BRIGHT, &nxt, &stationCfg);
sensSCREEN              sensorScr(NXT_SENSOR, &nxt, &stationCfg, &sPool);
timeSetSCR              clockScr(NXT_CLOCK, &nxt);
sunSCREEN               sunScr(NXT_SUNRISE, &nxt, &sm);

SCREEN* nxt_scr[7] =    {&mainScr, &sunScr, &histScr, &menuScr, &brightScr, &sensorScr, &clockScr};

SCREEN* pScreen = &mainScr;

// call back iroutine for file timestamps
void SDfileDate(uint16_t* date, uint16_t* time) {
 *date = FAT_DATE(year(), month(), day());
 *time = FAT_TIME(hour(), minute(), second());
}

void loadSensorData(void) {
  time_t current_time = now();
  if (wLogger.openLog(current_time)) {              // Read current date log file from the SD card
    byte id, h;
    int t, p;
    time_t ts;
    while (wLogger.readData(id, t, p, h, ts)) {
      if (id == 0)
        pSensor.load(ts, p);
      if ((h > 0) && (current_time - ts) <= 600)
        sPool.update(id, t, h, true, true);         // Save the data from remote sensor to the sensor pool
    }
  }
}

void setup() {
  //Serial.begin(9600);
  time_t now_t = RTC.get();                         // RTC (DS3231 clock) declared as an external variable
  setTime(now_t);
  randomSeed(now_t);
  bmp.begin(0x76);
  dht.begin();
  pSensor.init();
  sPool.init();
  wLogger.init(SD_PIN);
  SdFile::dateTimeCallback(SDfileDate);
  delay(1500);

  sm.init(OUR_timezone, OUR_latitude, OUR_longtitude);
  orscV2.begin(w433_INTR);
  nxt.init();
  hb.setHBTimeout(30);                              // Send heartbeat to NE555 timer every half of minute

  // Load configuration data
  stationCfg.init();
  stationCfg.load();
  
  nxt.clearAll();
  loadSensorData();                                 // Load previous data from SD card
  pScreen->init();
}

void w433Update(void) {
  int  temp = 0;
  byte hum  = 0;
  byte ch   = 0;
  byte id   = 0;
  bool battOK = false;
  if (orscV2.receiveData(ch, id, temp, hum, battOK)) {
    if ((id > 0) && (hum > 0)) {
      sPool.update(id, temp, hum, battOK);            // Save the data from remote sensor to the pool
    }
  }
  sPool.freeStaled();
}

void loop() {
 static uint32_t updated_data = 0;                    // Time in seconds when the internal sensors data was updated
 static time_t   write_log = 0;
 const uint32_t  period = 60000;                      // The period (ms) to update the internal sensor data

  hb.autoHB();                                        // Periodically send heartbeat to the NE555 timer (not every time)
  
  uint32_t nowMS = millis();
  if (!updated_data || ((nowMS - updated_data) >= period)) {
    updated_data = nowMS;
    pSensor.update();
    float fh = dht.readHumidity();
    float ft = dht.readTemperature();
    byte  h  = byte(fh + 0.5);
    int   t  = int(ft * 10);                          // The temperature in 10*centegrees
    sPool.update(0, t, h, true);                      // Save the data from local DHT sensor to the pool with sensor ID = 0
  }

  w433Update();                                       // Check wireless 433MHz sensors for new data
  
  time_t now_t = now();
  if (write_log == 0)                                 // System was rebooted, set Write log time
    write_log = wLogger.calculateWriteLogTime(now_t);

  // Save average data from all the sensors to the file in SD card
  if (now_t >= write_log) {
    write_log = wLogger.calculateWriteLogTime(now_t + 120);
    int aP = pSensor.average();
    byte id = 0;
    do {                                              // Save average data from all available sensors
      int  aT = sPool.aTemp(id);
      byte aH = sPool.aHum(id);
      wLogger.writeData(id, aT, aP, aH);              // Put the pressure data to the log witn every sensor data
      id = sPool.next(id);
    } while (id > 0);
    setTime(RTC.get());                               // Synchronize local time with DS3231 clock
  }

  // Remove old logs at 2:00 am
  if ((hour() == 2) && (minute() == 0) && (second() == 0)) { 
    wLogger.rmOldLog(now_t - 691200);                 // 8 days in seconds
    delay(1000);
  }
  
  byte page, item;
  if (nxt.event(page, item)) {
    byte nxt_index = pScreen->event(page, item);
    if (nxt_index >= 8) nxt_index -= 7;
    SCREEN* nxt = nxt_scr[nxt_index];
    if (nxt && (nxt != pScreen)) {
      pScreen = nxt;
      pScreen->init();
    }
  }

  if (pScreen->returnToMain()) {                       // return to the main screen by timeout
    pScreen = nxt_scr[NXT_MAIN];
    pScreen->init();
  }
  
  pScreen->show();
}

