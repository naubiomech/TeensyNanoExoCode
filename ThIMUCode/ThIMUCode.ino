/**
 * @file ThIMUCode.ino
 * @author Chance Cuddeback
 * @brief Uses the Nexgen AHRS to get global thigh angle. Transmits over I2C. Please define if left or right.
 * @date 2022-11-21
 * 
 */

#include <Wire.h>
#include <NexgenAHRS.h>

#define PERIOD 7 //ms
#define LEFT 1
#define LOG_LEVEL 0 // 0 = none, 1 = errors, 2 = warnings, 3 = debug
typedef enum {
    LOG_NONE = 0,
    LOG_ERROR = 1,
    LOG_WARN = 2,
    LOG_DEBUG = 3, 
    LOG_DISABLE,
} app_log_level_t;


static LSM9DS1 imu;
static EulerAngles angles;

static const uint8_t left_addr = 0x01;
static const uint8_t right_addr = 0x02;
static const int led_pins[] = {22, 23, 24};

/**
 * @brief Prints a message to the serial port if the log level is high enough
 * 
 * @param msg C String to print
 * @param level Log level of message
 */
inline static void logger(const char *msg, app_log_level_t level) {
    if (level <= LOG_LEVEL) {
        logger::print(msg);
    }
}
inline static void logger(const int msg, app_log_level_t level) {
    if (level <= LOG_LEVEL) {
        logger::print(msg);
    }
}
inline static void logger(const float msg, app_log_level_t level) {
    if (level <= LOG_LEVEL) {
        logger::print(msg);
    }
}

/**
 * @brief Writes RGB values to the LED
 * 
 * @param r Red 0-255
 * @param g Green 0-255
 * @param b Blue 0-255
 */
inline static void led_helper(bool r, bool g, bool b) {
  // LED is active low, must subtract by one
  digitalWrite(led_pins[0], !r);
  digitalWrite(led_pins[1], !g);
  digitalWrite(led_pins[2], !b);
};

/**
 * @brief Triggers a blocking error
 * 
 * @param msg Error Message
 */
inline static void app_error(const char *msg) {
    logger(msg, LOG_ERROR);
    while (1) {
        led_helper(1, 0, 0);
        delay(1000);
        led_helper(0, 0, 0);
        delay(1000);
    }
}


static const uint8_t ENABLE_CMD = 0x01;
static const uint8_t ANGLE_CMD = 0x02;

static volatile uint8_t working_cmd = 0x00;

inline static void on_receive(int bytes) {
    logger("\nOn Receive with: ", LOG_DEBUG);
    logger(bytes, LOG_DEBUG);

    if (bytes != 1) {
        logger("\nReceived invalid number of bytes", LOG_WARN);
        return;
    }

    uint8_t cmd = Wire.read();
    logger("\nReceived: ", LOG_DEBUG);
    logger(cmd, LOG_DEBUG);
    switch (cmd) {
        case ENABLE_CMD:
            logger("\nReceived command to turn on LED", LOG_DEBUG);
            led_helper(0, 1, 0);
            break;
        case ANGLE_CMD:
            logger("\nReceived command to get angle", LOG_DEBUG);
            break;
        default:
            logger("\nReceived invalid command", LOG_WARN);
            return;
    }
    working_cmd = cmd;
}

inline static void on_request() {
    logger("\nOn Request with: ", LOG_DEBUG);
    logger(working_cmd, LOG_DEBUG);
    
    if (working_cmd == 0) {
      logger("\nWorking command has not been initialized", LOG_WARN);
      return;
    }
    
    switch (working_cmd) {
      case ENABLE_CMD:
        logger("\nSending handshake", LOG_DEBUG);
        Wire.write(0x01);
        break;
      case ANGLE_CMD:
        logger("\nSending Roll angle: ", LOG_DEBUG);
        logger((uint8_t)angles.roll, LOG_DEBUG);
        Wire.write((uint8_t)angles.roll);
        break;
      default:
        logger("\nRequest on invalid command", LOG_WARN);
        return;
    }
    
    
}


void setup() {
    //  Start Serial
    Serial.begin(115200);
    delay(100);
    if (LOG_LEVEL == LOG_DEBUG) {
      while(!Serial);
    }

    // Initialize RGB LED
    for (int i = 0; i < 3; i++) {
        pinMode(led_pins[i], OUTPUT);
    }
    led_helper(0, 0, 0);

    // Initialise the LSM9DS1 IMU
    imu.begin();

    //  Positive magnetic declination - Sydney, AUSTRALIA
    imu.setDeclination(12.717);
    imu.setFusionAlgorithm(SensorFusion::FUSION);
    imu.setFusionPeriod(float(PERIOD*1000));   // Estimated sample period = 0.01 s = 100 Hz
    imu.setFusionThreshold(0.5f); // Stationary threshold = 0.5 degrees per second
    imu.setFusionGain(7.5);       // Fusion Filter Gain

    if (imu.connected()) {
      logger("\nIMU connected", LOG_DEBUG);
      imu.start();
    } else {
      app_error("\nIMU not connected");
    }


    // Initialize I2C communication, and set up callbacks
    if (LEFT == 1) {
        logger("\nLeft I2C started", LOG_DEBUG);
        Wire.begin(left_addr);
    } else {
        logger("\nRight I2C started", LOG_DEBUG);
        Wire.begin(right_addr);
    }

    Wire.onReceive(on_receive);
    Wire.onRequest(on_request);

    // Indicate that the device is on but I2C has not been validated
    led_helper(1, 1, 0);
}

void loop() { 
    static unsigned long previousMillis = 0;
    if (millis() - previousMillis >= PERIOD) {
      angles = imu.update();
      logger("\nRoll: ", LOG_DISABLE);
      logger(angles.roll, LOG_DISABLE);
      previousMillis = millis();
    }
}
