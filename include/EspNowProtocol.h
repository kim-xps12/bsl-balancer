#pragma once

#include <stdint.h>

namespace espnow_protocol {

constexpr uint8_t WIFI_CHANNEL = 1;
constexpr uint32_t PACKET_MAGIC = 0x42534C4A;  // "BSLJ"
constexpr uint8_t PACKET_VERSION = 1;

enum class PacketType : uint8_t {
  Discovery = 1,
  JoystickPosition = 2,
};

// Keep one fixed-size, naturally aligned wire representation for every packet
// type. Reserved fields allow compatible additions without changing its size.
struct __attribute__((packed)) Packet {
  uint32_t magic;
  uint32_t sender_millis;
  uint16_t sequence;
  int8_t pos_x;
  int8_t pos_y;
  uint8_t version;
  PacketType type;
  uint16_t reserved;
};

static_assert(sizeof(Packet) == 16, "ESP-NOW packet layout changed");

inline Packet makePacket(PacketType type, uint16_t sequence,
                         uint32_t sender_millis, int8_t pos_x = 0,
                         int8_t pos_y = 0) {
  return {PACKET_MAGIC, sender_millis, sequence, pos_x, pos_y,
          PACKET_VERSION, type, 0};
}

inline bool isValidPacket(const Packet& packet, PacketType expected_type) {
  return packet.magic == PACKET_MAGIC && packet.version == PACKET_VERSION &&
         packet.type == expected_type;
}

}  // namespace espnow_protocol
