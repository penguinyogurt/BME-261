// ESP-NOW link test — flash this SAME sketch to BOTH ESP32-WROOM-32 boards.
//
// Each board broadcasts a small packet once per second and prints every
// packet it hears (sender MAC, sequence number, RSSI). The packet also
// carries how many packets the sender has HEARD so far ("they-heard"),
// so watching one board's serial monitor proves the link both ways:
//   - "RX from ..." lines           -> other board's TX works, this RX works
//   - "they-heard" counting up      -> this board's TX reaches the other side
//
// No per-board configuration needed: broadcast requires no peer MAC.
// Known board: servo/receiver = 7C:87:CE:30:FA:88. The other board's MAC
// will appear in the RX lines — record it for the later unicast pairing.
//
// Serial: 115200 baud.

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

typedef struct __attribute__((packed)) {
  uint32_t seq;       // this board's send counter
  uint32_t heard;     // packets this board has received so far
  uint32_t uptimeMs;  // sender uptime, sanity check
} LinkPacket;

static const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

volatile uint32_t rxCount = 0;
uint32_t txSeq = 0;
uint32_t nextSendMs = 0;

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != (int)sizeof(LinkPacket)) return;  // ignore foreign ESP-NOW traffic
  LinkPacket p;
  memcpy(&p, data, sizeof p);
  rxCount++;

  char mac[18];
  snprintf(mac, sizeof mac, "%02X:%02X:%02X:%02X:%02X:%02X",
           info->src_addr[0], info->src_addr[1], info->src_addr[2],
           info->src_addr[3], info->src_addr[4], info->src_addr[5]);
  int rssi = (info->rx_ctrl != nullptr) ? info->rx_ctrl->rssi : 0;

  Serial.printf("RX from %s  seq=%lu  they-heard=%lu  rssi=%d dBm\n",
                mac, (unsigned long)p.seq, (unsigned long)p.heard, rssi);
}

void onSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  // Broadcast frames report success once transmitted (no ACK exists for
  // broadcast), so a failure here means a radio-level problem, not a
  // missing peer.
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("TX radio failure");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  WiFi.mode(WIFI_STA);

  Serial.println();
  Serial.println("=== ESP-NOW link test ===");
  // Read the MAC from the driver directly: WiFi.macAddress() returns zeros
  // if the STA interface hasn't finished starting yet.
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.printf("This board's STA MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  if (esp_now_init() != ESP_OK) {
    Serial.println("FATAL: esp_now_init failed");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = 0;            // 0 = use current WiFi channel (both boards default to 1)
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("FATAL: esp_now_add_peer failed");
    while (true) delay(1000);
  }

  Serial.println("Broadcasting one packet per second...");
}

void loop() {
  uint32_t now = millis();
  if (now >= nextSendMs) {
    nextSendMs = now + 1000;

    LinkPacket p;
    p.seq = ++txSeq;
    p.heard = rxCount;
    p.uptimeMs = now;
    esp_now_send(BROADCAST, (const uint8_t *)&p, sizeof p);

    Serial.printf("TX seq=%lu  (heard so far: %lu)\n",
                  (unsigned long)txSeq, (unsigned long)rxCount);
  }
}
