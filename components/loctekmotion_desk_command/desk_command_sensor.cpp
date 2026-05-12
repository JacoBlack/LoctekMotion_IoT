#include "desk_command_sensor.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace loctekmotion_desk_command {

static const char *const TAG = "loctekmotion_desk_command.sensor";

// ========== INTERNAL METHODS ==========
void DeskCommandSensor::loop() {
  uint8_t incomingByte;
  while (this->available() > 0) {
    if (this->read_byte(&incomingByte)) {
      // ESP_LOGD("DEBUG", "Incoming byte is: %08x", incomingByte);

      // First byte, start of a packet
      if (incomingByte == 0x9b) {
        // Reset message length and type
        this->msg_len = 0;
        this->msg_type = 0;
      }

      // Second byte defines the message length
      if (this->history[0] == 0x9b) {
        this->msg_len = incomingByte;
      }

      // Third byte is message type
      if (this->history[1] == 0x9b) {
        this->msg_type = incomingByte;
      }

      // Do processing when 5 bytes recieved, incomingByte is byte 5, history[0] is byte 4
      if (this->history[3] == 0x9b) {
        if (this->msg_type == 0x02 &&
            ((this->msg_len == 6) || (this->msg_len == 5))) { // valid desk command message
          if (history[0] != 0) { // non zero byte 4 - command is in a bit in this byte
            this->value = log2(history[0] * 2); // refactor 2^n to 1-7
          } else if (incomingByte != 0){ // non zero byte 5 - command is in a bit in this byte
            this->value = 8 + log2(incomingByte * 2); // refactor 2^n to 9-15
          } else {  // both bytes 4 and 5 are empty - empty desk command
            this->value = 8;
          }
        }
      }

      // Save byte buffer to history arrary
      this->history[3] = this->history[2];
      this->history[2] = this->history[1];
      this->history[1] = this->history[0];
      this->history[0] = incomingByte;

      // End byte
      if (incomingByte == 0x9d) {
        if (this->value && (this->value != this->lastPublished)) {
          this->publish_state(this->value);
          this->lastPublished = this->value;
        }
      }
    }
  }
}

void DeskCommandSensor::dump_config() {
  LOG_SENSOR("", "LoctekMotion Desk Command Sensor", this);
}

} // namespace loctekmotion_desk_command
} // namespace esphome
