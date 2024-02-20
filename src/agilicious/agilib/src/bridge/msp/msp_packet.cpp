#include "agilib/bridge/msp/msp_packet.hpp"


namespace agi {

MspPacket::MspPacket(const char payload_bytes) {
  data = (unsigned char*)malloc(payload_bytes + 6);
  memset(data, 0, payload_bytes + 6);
  data[0] = '$';
  data[1] = 'M';
  data[3] = payload_bytes;
}

MspPacket::~MspPacket() { free(data); }

void MspPacket::finalize() {
  checksum() = data[3] ^ data[4];
  const unsigned char* const tmp = payload();
  for (int i = 0; i < payload_length(); ++i) {
    checksum() ^= tmp[i];
  }
}

std::ostream& operator<<(std::ostream& os, const MspPacket& p) {
  std::string dir;
  switch (p.direction()) {
    case msp::SEND:
      dir = "to flight controller";
      break;
    case msp::RECV:
      dir = "from flight controller";
      break;
    default:
      dir = "unknown - error";
  }
  os << "Direction    " << dir << std::endl
     << "Payload Size " << (int)p.payload_length() << " bytes" << std::endl
     << "Message Code " << (int)p.code() << std::endl
     << "Checksum     " << (int)p.checksum() << std::endl
     << "Payload      ";
  for (int i = 0; i < p.payload_length(); ++i) {
    char tmp[3];
    sprintf(tmp, "%02X", p.payload()[i]);
    os << tmp << " ";
  }
  os << std::endl;
  return os;
}


bool MspPacket::copy_to_buffer(char* const buffer) const {
  std::memcpy(buffer, message(), message_size());
  return true;
}


}  // namespace agi
