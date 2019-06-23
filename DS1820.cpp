#include "DS1820.h"
#include "mbed.h"

bool DS1820::search_done = false;
int DS1820::last_discrepancy = 0;
uint8_t DS1820::search_ROM_data[8] = {0};
 
DS1820::DS1820(PinName data_pin, PinName power_pin)
    : datapin_(data_pin), parasitepin_(power_pin), parasite_power_(power_pin != NC) {
    memset(ROM, 0xFF, sizeof(ROM));
    memset(RAM, 0x00, sizeof(RAM));
}
 
bool DS1820::onewire_reset() {
    datapin_.output();
    datapin_ = 0; // bring low for 500 us
    wait_us(500);
    datapin_.input(); // let the data line float high
    wait_us(90);
    // see if any devices are pulling the data line low
    bool presence = datapin_.read() == 0;
    wait_us(410);
    return presence; // are there devices on bus?
}
 
void DS1820::onewire_bit_out(bool bit_data) {
    datapin_.output();
    datapin_ = 0;
    wait_us(3); // DXP modified from 5
    if (bit_data) {
        datapin_.input(); // bring data line high
        wait_us(55);
    } else {
        wait_us(55); // keep data line low
        datapin_.input();
        wait_us(10); // DXP added to allow bus to float high before next bit_out
    }
}
 
void DS1820::onewire_byte_out(uint8_t data) {
    // output data character (LSB first)
    for (uint8_t n = 0; n < 8; ++n) {
        onewire_bit_out(data & 0x01);
        data >>= 1; // shift next bit to LSB position
    }
}

bool DS1820::onewire_bit_in() {
    datapin_.output();
    datapin_ = 0;
    wait_us(3); // DXP modified from 5
    datapin_.input();
    wait_us(10); // DXP modified from 5
    bool answer = datapin_.read();
    wait_us(45); // DXP modified from 50
    return answer;
}
 
uint8_t DS1820::onewire_byte_in() {
    // read byte, least sig byte first
    uint8_t answer = 0x00;
    for (uint8_t i = 0; i < 8; i++) {
        answer >>= 1; // shift over to make room for the next bit
        if (onewire_bit_in()) answer |= 0x80; // if the data port is high, make this bit a 1
    }
    return answer;
}
 
bool DS1820::search_ROM() {
    return search_ROM_routine(0xF0); // Search ROM command
}
 
bool DS1820::search_alarm() {
    return search_ROM_routine(0xEC); // Search Alarm command
}
 
bool DS1820::search_ROM_routine(uint8_t command) {
    if (DS1820::search_done) return false;
    uint8_t discrepancy_marker = 0, byte_counter = 0, bit_mask = 0x01;
    if (not onewire_reset()) goto error_exit;
    onewire_byte_out(command); // Search ROM command or Search Alarm command
    for (uint8_t i = 1; i <= 64; ++i) {
        switch ((onewire_bit_in() << 1) | onewire_bit_in()) {
            case 0x03: // 0b11
                // data read error, this should never happen
                goto error_exit;
            case 0x02: // 0b10
                // Set ROM bit to one
                DS1820::search_ROM_data[byte_counter] |= bit_mask;
                break;
            case 0x01: // 0b01
                // Set ROM bit to zero
                DS1820::search_ROM_data[byte_counter] &= ~bit_mask;
                break;
            case 0x00: // 0b00
                // there are two or more devices present
                if (i == DS1820::last_discrepancy) {
                    // Set ROM bit to one
                    DS1820::search_ROM_data[byte_counter] |= bit_mask;
                } else if (i > DS1820::last_discrepancy) {
                    // Set ROM bit to zero
                    DS1820::search_ROM_data[byte_counter] &= ~bit_mask;
                    discrepancy_marker = i;
                } else if (not (DS1820::search_ROM_data[byte_counter] & bit_mask)) {
                    discrepancy_marker = i;
                }
        }
        onewire_bit_out(DS1820::search_ROM_data[byte_counter] & bit_mask);
        if (bit_mask & 0x80) {
            byte_counter++;
            bit_mask = 0x01;
        } else {
            bit_mask <<= 1;
        }
    }
    DS1820::last_discrepancy = discrepancy_marker;
    memcpy(ROM, DS1820::search_ROM_data, sizeof(DS1820::search_ROM_data));
    // abort search if CRC error detected
    if (ROM_checksum_error()) goto error_exit;
    if (DS1820::last_discrepancy == 0) DS1820::search_done = true;
    return true; // normal termination
    
error_exit:
    DS1820::last_discrepancy = 0;
    DS1820::search_done = true;
    return false;
}
 
void DS1820::read_ROM() {
    // NOTE: This command can only be used when there is one DS1820 on the bus. If this command
    // is used when there is more than one slave present on the bus, a data collision will occur
    // when all the DS1820s attempt to respond at the same time.
    onewire_reset();
    onewire_byte_out(0x33);   // Read ROM id
    for (int i = 0; i < 8; ++i) ROM[i] = onewire_byte_in();
}
 
void DS1820::match_ROM() {
    // Used to select a specific device
    onewire_reset();
    onewire_byte_out(0x55); // Match ROM command
    for (int i = 0; i < 8; ++i) onewire_byte_out(ROM[i]);
}
 
void DS1820::skip_ROM() {
    onewire_reset();
    onewire_byte_out(0xCC); // Skip ROM command
}
 
bool DS1820::ROM_checksum_error() {
    char CRC = 0x00;
     // Only going to shift the lower 7 bytes
    for (uint8_t i = 0; i < 7; ++i) CRC = CRC_byte(CRC, ROM[i]);
    // After 7 bytes CRC should equal the 8th byte (ROM CRC)
    return CRC != ROM[7]; // will return true if there is a CRC checksum mis-match         
}
 
bool DS1820::RAM_checksum_error() {
    uint8_t CRC = 0x00;
    // Only going to shift the lower 8 bytes
    for (uint8_t i = 0; i < 8; ++i) CRC = CRC_byte(CRC, RAM[i]);
    // After 8 bytes CRC should equal the 9th byte (RAM CRC)
    return CRC != RAM[8]; // will return true if there is a CRC checksum mis-match        
}
 
uint8_t DS1820::CRC_byte(uint8_t CRC, uint8_t byte) {
    for (uint8_t j = 0; j < 8; ++j) {
        if ((byte & 0x01) ^ (CRC & 0x01)) {
            // DATA ^ LSB CRC = 1
            CRC >>= 1;
            // Set the MSB to 1
            CRC |= 0x80;
            // Check bit 3
            if (CRC & 0x04) {
                CRC &= 0xFB; // Bit 3 is set, so clear it
            } else {
                CRC |= 0x04; // Bit 3 is clear, so set it
            }
            // Check bit 4
            if (CRC & 0x08) {
                CRC &= 0xF7; // Bit 4 is set, so clear it
            } else {
                CRC |= 0x08; // Bit 4 is clear, so set it
            }
        } else {
            // DATA ^ LSB CRC = 0
            CRC >>= 1;
            // clear MSB
            CRC &= 0x7F;
            // No need to check bits, with DATA ^ LSB CRC = 0, they will remain unchanged
        }
        byte >>= 1;
    }
    return CRC;
}
 
int DS1820::convert_temperature(bool wait, devices device) {
    // Convert temperature into scratchpad RAM for all devices at once
    int delay_time = 750; // Default delay time
    // Skip ROM command, will convert for ALL devices
    if (device == all_devices) {
        skip_ROM();
    } else {
        match_ROM();
        if (ROM[0] == FAMILY_CODE_DS18B20) {
            switch (RAM[4] & 0x60) {
                case 0x00: // 9 bits
                    delay_time = 94;
                    break;
                case 0x20: // 10 bits
                    delay_time = 188;
                    break;
                case 0x40: // 11 bits. Note 12bits uses the 750ms default
                    delay_time = 375;
                    break;
            }
        }
    }
    onewire_byte_out(0x44); // perform temperature conversion
    if (parasite_power_) {
        parasitepin_ = 1; // Parasite power strong pullup
        wait_ms(delay_time);
        parasitepin_ = 0;
        delay_time = 0;
    } else if (wait) {
        wait_ms(delay_time);
        delay_time = 0;
    }
    return delay_time;
}
 
void DS1820::read_RAM() {
    // This will copy the DS1820's 9 bytes of RAM data
    // into the objects RAM array. Functions that use
    // RAM values will automaticly call this procedure.
    match_ROM(); // Select this device
    onewire_byte_out(0xBE); // Read Scratchpad command
    for (uint8_t i=0; i < 9; ++i) RAM[i] = onewire_byte_in();
}

bool DS1820::set_configuration_bits(uint8_t resolution) {
    resolution -= 9;
    if (resolution < 4) {
        resolution <<= 5; // align the bits
        RAM[4] = (RAM[4] & 0x60) | resolution; // mask out old data, insert new
        write_scratchpad((RAM[2] << 8) + RAM[3]);
        // store_scratchpad(DS1820::this_device); // Need to test if this is required
        return true;
    }
    return false;
}
 
int DS1820::read_scratchpad() {
    read_RAM();
    return (RAM[2]<<8) + RAM[3];
}
 
void DS1820::write_scratchpad(int data) {
    RAM[3] = data;
    RAM[2] = data >> 8;
    match_ROM();
    onewire_byte_out(0x4E); // Copy scratchpad into DS1820 ram memory
    onewire_byte_out(RAM[2]); // T(H)
    onewire_byte_out(RAM[3]); // T(L)
    if (ROM[0] == FAMILY_CODE_DS18B20) {
        onewire_byte_out(RAM[4]); // Configuration register
    }
}
 
void DS1820::store_scratchpad(devices device) {
    // Skip ROM command, will store for ALL devices
    device == all_devices ? skip_ROM() : match_ROM();
    onewire_byte_out(0x48); // Write scratchpad into E2 command
    if (parasite_power_) parasitepin_ = 1;
    wait_ms(10); // Parasite power strong pullup for 10ms
    if (parasite_power_) parasitepin_ = 0;
}
 
int DS1820::recall_scratchpad(devices device) {
    // This copies the E2 values into the DS1820's memory.
    // If you specify all_devices this will return zero, otherwise
    // it will return the value of the scratchpad memory.
    // Skip ROM command, will store for ALL devices
    device == all_devices ? skip_ROM() : match_ROM();
    onewire_byte_out(0xB8); // Recall E2 data to scratchpad command
    wait_ms(10); // poll for completion instead?
    if (device == DS1820::this_device) {
        read_RAM();
        return read_scratchpad();
    }
    return 0;
}    
 
float DS1820::temperature(temperature_scale scale) {
    // The data specs state that count_per_degree should be 0x10 (16), I found my devices
    // to have a count_per_degree of 0x4B (75). With the standard resolution of 1/2 deg C
    // this allowed an expanded resolution of 1/150th of a deg C. I wouldn't rely on this
    // being super acurate, but it does allow for a smooth display in the 1/10ths of a
    // deg C or F scales.
    read_RAM();
    if (RAM_checksum_error()) return FP_NAN; // Indicate we got a CRC error
    int reading = (RAM[1] << 8) + RAM[0];
    if (reading & 0x8000) { // negative degrees C
        reading = 0 - ((reading ^ 0xffff) + 1); // 2's comp then convert to signed int
    }
    float answer = reading; // convert to float
    if (ROM[0] == FAMILY_CODE_DS18B20) {
        answer = answer / 8.0;
    } else {
        const float remaining_count = RAM[6];
        const float count_per_degree = RAM[7];
        answer = answer - 0.25 + (count_per_degree - remaining_count) / count_per_degree;
    }
    if (scale == celsius) {
        answer = answer / 2.0;
    } else {
        answer = answer * 9.0 / 10.0 + 32.0;
    }
    return answer;
}
 
bool DS1820::read_power_supply(devices device) {
    // This will return true if the device (or all devices) are Vcc powered
    // This will return false if the device (or ANY device) is parasite powered
    // Skip ROM command, will poll for any device using parasite power
    device == all_devices ? skip_ROM() : match_ROM();
    onewire_byte_out(0xB4); // Read power supply command
    return onewire_bit_in();
}

const char *DS1820::get_family() const {
    switch (ROM[0]) {
        case FAMILY_CODE_DS1820: // same for DS18S20
            return "DS1820/DS18S20";
        case FAMILY_CODE_DS18B20:
            return "DS18B20";
        default:
            return "UNKNOWN";
    }
}
