
// Add the ESP32 as a Bluetooth keyboard of your computer
// or mobile phone:
//
// 1. Go to your computers/phones settings
// 2. Ensure Bluetooth is turned on
// 3. Scan for Bluetooth devices
// 4. Connect to the device called "ESP32 Keyboard"
// 5. Open an empty document in a text editor
// 6. Press the button attached to the ESP32

#define US_KEYBOARD 1
#define USEWIFI 1

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEHIDDevice.h>
#include <HIDTypes.h>
#include <HIDKeyboardTypes.h>

#if USEWIFI
#include <esp_wifi.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#endif






// Change the below values if desired
#define POT_PEDAL_PIN 3
#define POT_EXT_PIN 4
#define LED_PIN 10
#define POT_RESOLUTION 12 // Number of bits

#define PRINT_DBG_INTERVAL  800UL //Milliseconds

#define WIFI_SSID "Abschirmdienst"
#define WIFI_PASS "frank123"

#define DEVICE_NAME "Pedal"




// Forward declarations
#if USEWIFI
void handleWiFi();
void WiFiCallback(WiFiEvent_t event, WiFiEventInfo_t info);
#endif
void printdebug(float pedal, float ext, float period);
void handleBle(float pedal, float ext);
void bluetoothTask(void*);
void typeKey(char key);
void typeText(const char* text);



unsigned long now;
volatile bool isBleConnected = false;
volatile bool isOTAinProgress = false;

enum LEDMODE {boot, notconnected, connected};
LEDMODE ledMode = boot;

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, 1);

    Serial.begin(115200);
    Serial.printf("Flash: %dMB CPU: %dMHz\n", ESP.getFlashChipSize() / 1000000, ESP.getCpuFreqMHz());

    analogReadResolution(POT_RESOLUTION);

    Serial.printf("Attempting to connect to SSID: %s\n", WIFI_SSID);

#if USEWIFI
    WiFi.useStaticBuffers(true);
    WiFi.mode(WIFI_STA);
    WiFi.onEvent(WiFiCallback, WiFiEvent_t::ARDUINO_EVENT_MAX);
    WiFi.setHostname(DEVICE_NAME);

    ArduinoOTA.onStart([]() {
        isOTAinProgress = true;
        digitalWrite(LED_PIN, 1);
    });
    ArduinoOTA.setHostname(DEVICE_NAME);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
#endif

    // start Bluetooth task
    xTaskCreate(bluetoothTask, "bluetooth", 20000, NULL, 5, NULL);
    ledMode = notconnected;

}

void loop() {

    now = millis();
#if USEWIFI
    handleWiFi();
    vTaskDelay(5 / portTICK_PERIOD_MS);
#endif
    if (isOTAinProgress) return;

    float pot_pedal = 100.0f * analogRead(POT_PEDAL_PIN) / (1 << POT_RESOLUTION);
    float pot_ext = 100.0f * analogRead(POT_EXT_PIN) / (1 << POT_RESOLUTION);
    handleBle(pot_pedal, pot_ext);

}

#if USEWIFI
static unsigned long timeWiFiConnected;

void handleWiFi()
{

    if (!WiFi.isConnected()) return;
    ArduinoOTA.handle();
    if (isOTAinProgress) return;
/*
    //Switch off WiFI after WIFI_MINUTES:
    if (now - timeWiFiConnected > WIFI_MINUTES * 60UL * 1000UL) {
        WiFi.disconnect();
        WiFi.mode(WIFI_OFF);
        return;
    }
*/
}

void WiFiCallback(WiFiEvent_t event, WiFiEventInfo_t info)
{

 switch(event) {

    case WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP:
            timeWiFiConnected = millis();
            ArduinoOTA.begin();
            Serial.printf("WiFi Connected.\nSSID: %s IP Address: %s signal strength (RSSI):%d dBm\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(), WiFi.RSSI());
            break;

    case WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Serial.printf("WiFi stopped.\n");
            break;
 }

}
#endif

void handleBle(float pedal, float ext)
{
    static unsigned long t;
    if (!isBleConnected) return;

    if (pedal < 10.0f) pedal = 0;
    if (ext < 1.0f) ext = 0;

    long period = pedal * 20; //milliseconds

    printdebug(pedal, ext, period);
    if (period > 0 && millis() - t >= period ) {
        t = millis();
        typeKey(DOWN_ARROW);
        digitalWrite(LED_PIN, 1);
        vTaskDelay(25 / portTICK_PERIOD_MS);
        digitalWrite(LED_PIN, 0);
    }


}

void printdebug(float pedal, float ext, float period)
{
    static unsigned long t;
    if (millis() - t > PRINT_DBG_INTERVAL ) {
        t = millis();
        Serial.printf("Pedal: %3.2f %% Ext: %3.2f %% => Period: %d ms\n", pedal, ext, (int)period);
    }
}

// Message (report) sent when a key is pressed or released
struct InputReport {
    uint8_t modifiers;	     // bitmask: CTRL = 1, SHIFT = 2, ALT = 4
    uint8_t reserved;        // must be 0
    uint8_t pressedKeys[6];  // up to six concurrenlty pressed keys
};

// Message (report) received when an LED's state changed
struct OutputReport {
    uint8_t leds;            // bitmask: num lock = 1, caps lock = 2, scroll lock = 4, compose = 8, kana = 16
};


// The report map describes the HID device (a keyboard in this case) and
// the messages (reports in HID terms) sent and received.
static const uint8_t REPORT_MAP[] = {
    USAGE_PAGE(1),      0x01,       // Generic Desktop Controls
    USAGE(1),           0x06,       // Keyboard
    COLLECTION(1),      0x01,       // Application
    REPORT_ID(1),       0x01,       //   Report ID (1)
    USAGE_PAGE(1),      0x07,       //   Keyboard/Keypad
    USAGE_MINIMUM(1),   0xE0,       //   Keyboard Left Control
    USAGE_MAXIMUM(1),   0xE7,       //   Keyboard Right Control
    LOGICAL_MINIMUM(1), 0x00,       //   Each bit is either 0 or 1
    LOGICAL_MAXIMUM(1), 0x01,
    REPORT_COUNT(1),    0x08,       //   8 bits for the modifier keys
    REPORT_SIZE(1),     0x01,
    HIDINPUT(1),        0x02,       //   Data, Var, Abs
    REPORT_COUNT(1),    0x01,       //   1 byte (unused)
    REPORT_SIZE(1),     0x08,
    HIDINPUT(1),        0x01,       //   Const, Array, Abs
    REPORT_COUNT(1),    0x06,       //   6 bytes (for up to 6 concurrently pressed keys)
    REPORT_SIZE(1),     0x08,
    LOGICAL_MINIMUM(1), 0x00,
    LOGICAL_MAXIMUM(1), 0x65,       //   101 keys
    USAGE_MINIMUM(1),   0x00,
    USAGE_MAXIMUM(1),   0x65,
    HIDINPUT(1),        0x00,       //   Data, Array, Abs
    REPORT_COUNT(1),    0x05,       //   5 bits (Num lock, Caps lock, Scroll lock, Compose, Kana)
    REPORT_SIZE(1),     0x01,
    USAGE_PAGE(1),      0x08,       //   LEDs
    USAGE_MINIMUM(1),   0x01,       //   Num Lock
    USAGE_MAXIMUM(1),   0x05,       //   Kana
    LOGICAL_MINIMUM(1), 0x00,
    LOGICAL_MAXIMUM(1), 0x01,
    HIDOUTPUT(1),       0x02,       //   Data, Var, Abs
    REPORT_COUNT(1),    0x01,       //   3 bits (Padding)
    REPORT_SIZE(1),     0x03,
    HIDOUTPUT(1),       0x01,       //   Const, Array, Abs
    END_COLLECTION(0)               // End application collection
};


BLEHIDDevice* hid;
BLECharacteristic* input;
BLECharacteristic* output;

const InputReport NO_KEY_PRESSED = { };


/*
 * Callbacks related to BLE connection
 */
class BleKeyboardCallbacks : public BLEServerCallbacks {

    void onConnect(BLEServer* server) {
        isBleConnected = true;

        // Allow notifications for characteristics
        BLE2902* cccDesc = (BLE2902*)input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
        cccDesc->setNotifications(true);

        Serial.println("Bluetooth client has connected.");
        digitalWrite(LED_PIN, 0);
        //typeText(MESSAGE);
    }

    void onDisconnect(BLEServer* server) {
        isBleConnected = false;

        // Disallow notifications for characteristics
        BLE2902* cccDesc = (BLE2902*)input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
        cccDesc->setNotifications(false);

        Serial.println("Bluetooth client has disconnected.");
        digitalWrite(LED_PIN, 1);
    }
};


/*
 * Called when the client (computer, smart phone) wants to turn on or off
 * the LEDs in the keyboard.
 *
 * bit 0 - NUM LOCK
 * bit 1 - CAPS LOCK
 * bit 2 - SCROLL LOCK
 */
 class OutputCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* characteristic) {
        OutputReport* report = (OutputReport*) characteristic->getData();
        /*
        Serial.print("LED state: ");
        Serial.print((int) report->leds);
        Serial.println();
        */
    }
};


void bluetoothTask(void*) {

    // initialize the device
    BLEDevice::init(DEVICE_NAME);
    BLEServer* server = BLEDevice::createServer();
    server->setCallbacks(new BleKeyboardCallbacks());

    // create an HID device
    hid = new BLEHIDDevice(server);
    input = hid->inputReport(1); // report ID
    output = hid->outputReport(1); // report ID
    output->setCallbacks(new OutputCallbacks());

    // set manufacturer name
    hid->manufacturer()->setValue("Maker Community");
    // set USB vendor and product ID
    hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
    // information about HID device: device is not localized, device can be connected
    hid->hidInfo(0x00, 0x02);

    // Security: device requires bonding
    BLESecurity* security = new BLESecurity();
    security->setAuthenticationMode(ESP_LE_AUTH_BOND);

    // set report map
    hid->reportMap((uint8_t*)REPORT_MAP, sizeof(REPORT_MAP));
    hid->startServices();

    // set battery level to 100%
    hid->setBatteryLevel(100);

    // advertise the services
    BLEAdvertising* advertising = server->getAdvertising();
    advertising->setAppearance(HID_KEYBOARD);
    advertising->addServiceUUID(hid->hidService()->getUUID());
    advertising->addServiceUUID(hid->deviceInfo()->getUUID());
    advertising->addServiceUUID(hid->batteryService()->getUUID());
    advertising->start();

    Serial.println("BLE ready");
    vTaskDelay(portMAX_DELAY / portTICK_PERIOD_MS);
};

void typeKey(char key)
{
    // translate character to key combination
    uint8_t val = (uint8_t)key;
    if (val > KEYMAP_SIZE)
        return; // character not available on keyboard - return
    KEYMAP map = keymap[val];

    // create input report
    InputReport report = {
        .modifiers = map.modifier,
        .reserved = 0,
        .pressedKeys = {
            map.usage,
            0, 0, 0, 0, 0
        }
    };

    // send the input report
    input->setValue((uint8_t*)&report, sizeof(report));
    input->notify();

    //delay(5);
    //vTaskDelay(1 / portTICK_PERIOD_MS);
    // release all keys between two characters; otherwise two identical
    // consecutive characters are treated as just one key press
    input->setValue((uint8_t*)&NO_KEY_PRESSED, sizeof(NO_KEY_PRESSED));
    input->notify();

    //delay(5);
    //vTaskDelay(5 / portTICK_PERIOD_MS);
    vTaskDelay(1 / portTICK_PERIOD_MS);
}

void typeText(const char* text) {

    while (*text != 0) {
        typeKey(*text++);
    }

}
