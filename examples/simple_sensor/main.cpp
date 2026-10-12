#include "SensorMesh.h"
#include <Preferences.h>

#ifndef BUS_SENSE_PIN
  #define BUS_SENSE_PIN  7    // opto-isolator output -> GPIO 7
#endif

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

class MyMesh : public SensorMesh {
public:
  MyMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : SensorMesh(board, radio, ms, rng, rtc, tables), 
       battery_data(12*24, 5*60)    // 24 hours worth of battery data, every 5 minutes
  {
  }

protected:
  /* ========================== custom logic here ========================== */
  Trigger low_batt, critical_batt;
  Trigger bus_lost, bus_restored;
  TimeSeriesData  battery_data;
  bool bus_invert = false;     // false: opto output LOW = bus live (typical PC817 module)
  bool bus_seen_loss = false;
  int  bus_last_state = -1;    // -1 = not yet read since boot
  mesh::GroupChannel alert_ch;
  uint8_t alert_key_len = 0;   // 0 = no channel key set

  void setAlertKey(const uint8_t* key, uint8_t len) {
    memset(alert_ch.secret, 0, sizeof(alert_ch.secret));
    memcpy(alert_ch.secret, key, len);
    mesh::Utils::sha256(alert_ch.hash, sizeof(alert_ch.hash), alert_ch.secret, len);
    alert_key_len = len;
  }

  bool sendChannelAlert(const char* text) {
    if (alert_key_len == 0) return false;
    uint8_t temp[5 + 160];
    uint32_t ts = getRTCClock()->getCurrentTimeUnique();
    memcpy(temp, &ts, 4);
    temp[4] = 0;  // TXT_TYPE_PLAIN
    int n = snprintf((char*)&temp[5], 160, "%s: %s", getNodePrefs()->node_name, text);
    if (n < 0) return false;
    if (n > 159) n = 159;
    auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, alert_ch, temp, 5 + n);
    if (!pkt) return false;
    sendFlood(pkt);
    return true;
  }

  bool busLive() {
    bool pin_low = (digitalRead(BUS_SENSE_PIN) == LOW);
    return bus_invert ? !pin_low : pin_low;
  }

  void initBusSense() {
    pinMode(BUS_SENSE_PIN, INPUT_PULLUP);
    Preferences p; p.begin("bussense", true);
    bus_invert = p.getBool("invert", false);
    uint8_t key[32];
    size_t klen = p.getBytes("chkey", key, sizeof(key));
    if (klen == 16 || klen == 32) setAlertKey(key, (uint8_t)klen);
    p.end();
  }

  void onSensorDataRead() override {
    float batt_voltage = getVoltage(TELEM_CHANNEL_SELF);

    battery_data.recordData(getRTCClock(), batt_voltage);   // record battery
    alertIf(batt_voltage < 3.4f, critical_batt, HIGH_PRI_ALERT, "Battery is critical!");
    alertIf(batt_voltage < 3.6f, low_batt, LOW_PRI_ALERT, "Battery is low");

    bool live = busLive();
    if (!live) bus_seen_loss = true;
    alertIf(!live, bus_lost, HIGH_PRI_ALERT, "Main bus: voltage LOST");
    alertIf(live && bus_seen_loss, bus_restored, LOW_PRI_ALERT, "Main bus: RESTORED");

    // channel alerts: shown reliably in the app's channel view
    int state = live ? 1 : 0;
    if (bus_last_state == -1) {
      sendChannelAlert(live ? "Sensor started, main bus LIVE" : "Sensor started, main bus DOWN");
    } else if (state != bus_last_state) {
      sendChannelAlert(live ? "Main bus: RESTORED" : "Main bus: voltage LOST");
    }
    bus_last_state = state;
  }

  void addCustomTelemetry(CayenneLPP& lpp) override {
    lpp.addDigitalInput(TELEM_CHANNEL_SELF, busLive() ? 1 : 0);   // 1 = bus live
  }

  int querySeriesData(uint32_t start_secs_ago, uint32_t end_secs_ago, MinMaxAvg dest[], int max_num) override {
    battery_data.calcMinMaxAvg(getRTCClock(), start_secs_ago, end_secs_ago, &dest[0], TELEM_CHANNEL_SELF, LPP_VOLTAGE);
    return 1;
  }

  bool handleCustomCommand(uint32_t sender_timestamp, char* command, char* reply) override {
    if (strcmp(command, "bus") == 0) {
      sprintf(reply, "bus %s (pin %s, invert %s)", busLive() ? "LIVE" : "DOWN",
              digitalRead(BUS_SENSE_PIN) ? "HIGH" : "LOW", bus_invert ? "on" : "off");
      return true;
    }
    if (strcmp(command, "bus.invert on") == 0 || strcmp(command, "bus.invert off") == 0) {
      bus_invert = (command[11] == 'o' && command[12] == 'n');
      Preferences p; p.begin("bussense", false); p.putBool("invert", bus_invert); p.end();
      sprintf(reply, "OK - invert %s, bus now %s", bus_invert ? "on" : "off", busLive() ? "LIVE" : "DOWN");
      return true;
    }
    if (strncmp(command, "alert.key", 9) == 0) {
      const char* hex = command + 9;
      while (*hex == ' ') hex++;
      if (*hex == 0) {
        sprintf(reply, "alert channel key %s", alert_key_len ? "SET" : "NOT set");
        return true;
      }
      int hl = strlen(hex);
      uint8_t key[32];
      if ((hl == 32 || hl == 64) && mesh::Utils::fromHex(key, hl / 2, hex)) {
        setAlertKey(key, hl / 2);
        Preferences p; p.begin("bussense", false); p.putBytes("chkey", key, hl / 2); p.end();
        strcpy(reply, "OK - alert channel key saved");
      } else {
        strcpy(reply, "ERR: key must be 32 or 64 hex chars");
      }
      return true;
    }
    if (strcmp(command, "alert.test") == 0) {
      strcpy(reply, sendChannelAlert("Test alert") ? "OK - test sent to channel" : "ERR: no alert channel key set");
      return true;
    }
    return false;  // not handled
  }

public:
  void beginBusSense() { initBusSense(); }

protected:
  /* ======================================================================= */
};

StdRNG fast_rng;
SimpleMeshTables tables;

MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

void halt() {
  while (1) ;
}

static char command[160];

void setup() {
  Serial.begin(115200);
  delay(1000);

  board.begin();

#ifdef HAS_EXTERNAL_WATCHDOG
  external_watchdog.begin();
#endif

#ifdef DISPLAY_CLASS
  if (display.begin()) {
    display.startFrame();
    display.print("Please wait...");
    display.endFrame();
  }
#endif

  if (!radio_init()) { halt(); }

  fast_rng.begin(radio_driver.getRngSeed());

  FILESYSTEM* fs;
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  InternalFS.begin();
  fs = &InternalFS;
  IdentityStore store(InternalFS, "");
#elif defined(ESP32)
  SPIFFS.begin(true);
  fs = &SPIFFS;
  IdentityStore store(SPIFFS, "/identity");
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  fs = &LittleFS;
  IdentityStore store(LittleFS, "/identity");
  store.begin();
#else
  #error "need to define filesystem"
#endif
  if (!store.load("_main", the_mesh.self_id)) {
    MESH_DEBUG_PRINTLN("Generating new keypair");
    the_mesh.self_id = radio_new_identity();   // create new random identity
    int count = 0;
    while (count < 10 && (the_mesh.self_id.pub_key[0] == 0x00 || the_mesh.self_id.pub_key[0] == 0xFF)) {  // reserved id hashes
      the_mesh.self_id = radio_new_identity(); count++;
    }
    store.save("_main", the_mesh.self_id);
  }

  Serial.print("Sensor ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE); Serial.println();

  command[0] = 0;

  sensors.begin();
  the_mesh.beginBusSense();

  the_mesh.begin(fs);

#ifdef DISPLAY_CLASS
  ui_task.begin(the_mesh.getNodePrefs(), FIRMWARE_BUILD_DATE, FIRMWARE_VERSION);
#endif

  // send out initial zero hop Advertisement to the mesh
#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvertisement(16000, false);
#endif
}

void loop() {
  int len = strlen(command);
  while (Serial.available() && len < sizeof(command)-1) {
    char c = Serial.read();
    if (c != '\n') {
      command[len++] = c;
      command[len] = 0;
    }
    Serial.print(c);
  }
  if (len == sizeof(command)-1) {  // command buffer full
    command[sizeof(command)-1] = '\r';
  }

  if (len > 0 && command[len - 1] == '\r') {  // received complete line
    command[len - 1] = 0;  // replace newline with C string null terminator
    char reply[160];
    the_mesh.handleCommand(0, command, reply);  // NOTE: there is no sender_timestamp via serial!
    if (reply[0]) {
      Serial.print("  -> "); Serial.println(reply);
    }

    command[0] = 0;  // reset command buffer
  }

  the_mesh.loop();
  sensors.loop();
#ifdef DISPLAY_CLASS
  ui_task.loop();
#endif
  rtc_clock.tick();
#ifdef HAS_EXTERNAL_WATCHDOG
  external_watchdog.loop();
#endif
}
