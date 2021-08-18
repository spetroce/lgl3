#include <iostream>
#include <bitset>

/// bit 14 - Channel Group Select, 1 = HB [8:7] | 0 = HB [6:1]
#define NCV7719_HBSEL 0x4000

uint8_t g_set_digit_toggle_cnt = 0;

int main(int argc, char *argv[]) {
  if (argc < 3) {
    return -1;
  }
  const uint8_t curr_digit = atoi(argv[1]),
                next_digit = atoi(argv[2]);
  #define DIGIT_CFG_ARRAY_LEN 12
  if (curr_digit >= DIGIT_CFG_ARRAY_LEN ||
      next_digit >= DIGIT_CFG_ARRAY_LEN) {
    return -1;
  }
  std::cout << "curr_digit: " << static_cast<int>(curr_digit) << ", " << "next_digit: " << static_cast<int>(next_digit) << std::endl;
  #define NUM_SEGMENT 7
  const uint32_t seg_mask[NUM_SEGMENT] =
  //          xg    XG0   fedcbaFEDCBA0
    {0b00000000000000000000000000000010,  // A
     0b00000000000000000000000000000100,  // B
     0b00000000000000000000000000001000,  // C
     0b00000000000000000000000000010000,  // D
     0b00000000000000000000000000100000,  // E
     0b00000000000000000000000001000000,  // F
     0b00000000000000100000000000000000}; // G
  const uint32_t digit_cfg[DIGIT_CFG_ARRAY_LEN] =
  //          xg    XG0   fedcbaFEDCBA0
    {0b00000000000000000000000001111110,  // 0
     0b00000000000000000000000000001100,  // 1
     0b00000000000000100000000000110110,  // 2
     0b00000000000000100000000000011110,  // 3
     0b00000000000000100000000001001100,  // 4
     0b00000000000000100000000001011010,  // 5
     0b00000000000000100000000001111010,  // 6
     0b00000000000000000000000000001110,  // 7
     0b00000000000000100000000001111110,  // 8
     0b00000000000000100000000001011110,  // 9
     0b00000000000000100000000001111110,  // all
     0b00000000000000000000000000000000}; // none
                                        //       xg    XG    fedcbaFEDCBA
  const uint32_t reset_pin_bit_cfg_on = 0b00000000000001000000000000000000;  // X
  const uint32_t reset_pin_bit_en =  reset_pin_bit_cfg_on << 6;  // x
  // Also use digit_xor to specify which half-bridges need to be enabled
  const uint32_t digit_xor = digit_cfg[curr_digit] ^ digit_cfg[next_digit];
  const uint32_t all_bits_to_turn_on = digit_cfg[next_digit] & digit_xor,
                 all_bits_to_turn_off = digit_cfg[curr_digit] & digit_xor;
  uint32_t bits_to_turn_on[8] = {0},
           bits_to_turn_off[8] = {0};
  uint8_t bits_to_turn_on_len = 0,
          bits_to_turn_off_len = 0;
  uint8_t i, j, k;
  for (i = 0, j = 0, k = 0; i < NUM_SEGMENT; ++i) {
    if (all_bits_to_turn_on & seg_mask[i]) {
      bits_to_turn_on[bits_to_turn_on_len] |= seg_mask[i];
      ++j;
      if (j == 2) {
        j = 0;
        ++bits_to_turn_on_len;
      }
    } else if (all_bits_to_turn_off & seg_mask[i]) {
      bits_to_turn_off[bits_to_turn_off_len] |= seg_mask[i];
      ++k;
      if (k == 2) {
        ++bits_to_turn_off_len;
        k = 0;
      }
    }
  }
  if (j % 2 != 0) {
    ++bits_to_turn_on_len;
  }
  if (k % 2 != 0) {
    ++bits_to_turn_off_len;
  }
  const uint32_t all_bits_to_turn_on_en = all_bits_to_turn_on << 6,
                 all_bits_to_turn_off_en = all_bits_to_turn_off << 6;
  uint32_t tx_data_handle_on_bits = all_bits_to_turn_on | all_bits_to_turn_on_en | reset_pin_bit_en;
  uint32_t tx_data_handle_off_bits = all_bits_to_turn_off_en | reset_pin_bit_cfg_on | reset_pin_bit_en;
  std::cout << "                             xg    XG    fedcbaFEDCBA" << std::endl;
  std::cout << "          curr_digit: " << std::bitset<32>(digit_cfg[curr_digit]) << std::endl;
  std::cout << "          next_digit: " << std::bitset<32>(digit_cfg[next_digit]) << std::endl;
  std::cout << "                 xor: " << std::bitset<32>(digit_xor) << std::endl;
  std::cout << " all_bits_to_turn_on: " << std::bitset<32>(all_bits_to_turn_on) << std::endl;
  for (size_t i = 0; i < bits_to_turn_on_len; ++i) {
  std::cout << "     bits_to_turn_on: " << std::bitset<32>(bits_to_turn_on[i]) << std::endl;
  }
  std::cout << "all_bits_to_turn_off: " << std::bitset<32>(all_bits_to_turn_off) << std::endl;
  for (size_t i = 0; i < bits_to_turn_off_len; ++i) {
  std::cout << "    bits_to_turn_off: " << std::bitset<32>(bits_to_turn_off[i]) << std::endl;
  }
  std::cout << "                                xg    XG    fedcbaFEDCBA" << std::endl;
  std::cout << " all_bits_to_turn_on_en: " << std::bitset<32>(all_bits_to_turn_on_en) << std::endl;
  std::cout << "all_bits_to_turn_off_en: " << std::bitset<32>(all_bits_to_turn_off_en) << std::endl;
  return 0;
}
