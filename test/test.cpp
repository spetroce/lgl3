#include <iostream>
#include <bitset>

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
  const uint32_t digit_cfg[DIGIT_CFG_ARRAY_LEN] =
  //          xg    XG    fedcbaFEDCBA
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
     0b00000000000000100000000001100000,  // init_prev
     0b00000000000000000000000000011110}; // init_next
                                        //       xg    XG    fedcbaFEDCBA
  const uint32_t reset_pin_bit_cfg_on = 0b00000000000001000000000000000000;  // X
  const uint32_t reset_pin_bit_en =  reset_pin_bit_cfg_on << 6;  // x
  // Also use digit_xor to specify which half-bridges need to be enabled
  const uint32_t digit_xor = digit_cfg[curr_digit] ^ digit_cfg[next_digit];
  const uint32_t bits_to_turn_on = digit_cfg[next_digit] & digit_xor,
                 bits_to_turn_off = digit_cfg[curr_digit] & digit_xor;
  const uint32_t bits_to_turn_on_en = bits_to_turn_on << 6,
                 bits_to_turn_off_en = bits_to_turn_off << 6;
  uint32_t tx_data_handle_on_bits = bits_to_turn_on | bits_to_turn_on_en | reset_pin_bit_en;
  uint32_t tx_data_handle_off_bits = bits_to_turn_off_en | reset_pin_bit_cfg_on | reset_pin_bit_en;
  std::cout << "                         xg    XG    fedcbaFEDCBA" << std::endl;
  std::cout << "      curr_digit: " << std::bitset<32>(digit_cfg[curr_digit]) << std::endl;
  std::cout << "      next_digit: " << std::bitset<32>(digit_cfg[next_digit]) << std::endl;
  std::cout << "             xor: " << std::bitset<32>(digit_xor) << std::endl;
  std::cout << " bits_to_turn_on: " << std::bitset<32>(bits_to_turn_on) << std::endl;
  std::cout << "bits_to_turn_off: " << std::bitset<32>(bits_to_turn_off) << std::endl;
  std::cout << "                            xg    XG    fedcbaFEDCBA" << std::endl;
  std::cout << " bits_to_turn_on_en: " << std::bitset<32>(bits_to_turn_on_en) << std::endl;
  std::cout << "bits_to_turn_off_en: " << std::bitset<32>(bits_to_turn_off_en) << std::endl;
  uint16_t * tx_data_word = (uint16_t*)&tx_data_handle_on_bits;
  std::cout << std::hex << std::uppercase;
  std::cout << " txd_handle_on_bits: " << std::bitset<32>(tx_data_handle_on_bits) << ", " <<
                                          tx_data_word[0] << ", " << tx_data_word[1] << std::endl;
  tx_data_word = (uint16_t*)&tx_data_handle_off_bits;
  std::cout << "txd_handle_off_bits: " << std::bitset<32>(tx_data_handle_off_bits) << ", " <<
                                          tx_data_word[0] << ", " << tx_data_word[1] << std::endl;
  return 0;
}
