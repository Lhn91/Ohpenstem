/**
 * @file main.cpp
 * @brief Firmware C++ cho Yolo UNO (Control Hub), chỉ hỗ trợ OpenBot, không sử dụng gyro.
 *
 * PHIÊN BẢN SIMPLIFIED:
 * - Xử lý lệnh từ OpenBot App qua BLE
 * - OpenBot: BLE với lệnh "c<drive>,<turn>"
 * - Đã loại bỏ tính năng gyro và PID, điều khiển trực tiếp không hiệu chỉnh
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <algorithm>
#include <map>

// --- CẤU HÌNH ---
#define I2C_MOTOR_DRIVER_ADDRESS 0x54
#define DEVICE_NAME "OpenBot-YoloUNO"

// I2C Pin definitions
#define SDA_PIN 11
#define SCL_PIN 12

// --- HẰNG SỐ ---
const uint8_t M1 = 1, M2 = 2, E1 = 16, E2 = 32;
const uint8_t MOTOR_PORT_FL = M1, MOTOR_PORT_FR = M2, MOTOR_PORT_BL = E1, MOTOR_PORT_BR = E2;

#define REG_MOTOR_IDX      0x10
#define REG_MOTOR_SPEED    0x12
#define REG_ENCODER_SETUP  0x20

// UUIDs Chuẩn UART - Tương thích với OpenBot
#define SERVICE_UUID             "61653dc3-4021-4d1e-ba83-8b4eec61d613" // UART service UUID
#define CHARACTERISTIC_UUID_RX   "06386c14-86ea-4d71-811c-48f97c58f8c9" // RX: Nhận lệnh từ điện thoại
#define CHARACTERISTIC_UUID_TX   "9bf1103b-834c-47cf-b149-c9e4bcf778a7" // TX: Gửi dữ liệu lên điện thoại

// --- BIẾN TOÀN CỤC ---
BLEServer* pServer = NULL;
bool deviceConnected = false;
bool motor_driver_ok = false;  // Thêm biến kiểm tra motor driver

// Biến để theo dõi thời gian nhận lệnh cuối cùng từ mỗi nguồn
unsigned long last_openbot_cmd_time = 0;
const unsigned long CMD_TIMEOUT = 2000; // Coi như mất kết nối nếu không có lệnh mới sau 2 giây

// Biến cho OpenBot
volatile int g_drive_y = 0;
volatile int g_turn_z = 0;
int last_valid_drive_y = 0;  // Giữ lệnh cuối cùng hợp lệ
int last_valid_turn_z = 0;

// Biến debug
unsigned long last_debug_output = 0;
const unsigned long DEBUG_INTERVAL = 5000; // Debug mỗi 5 giây

// --- KHAI BÁO HÀM ---
void set_i2c_motor_speed(uint8_t motor_mask, int speed);
void set_i2c_encoder(uint8_t motorMask, uint16_t rpm, uint8_t ppr, uint8_t gear);
void drive_mecanum(int y, int x, int z);
void stop_motors();
void unified_parser(const std::string& command);
void control_loop();
bool test_motor_driver();
void scan_i2c_devices();
void debug_output();
void test_motors();

// --- LỚP CALLBACK ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { 
        deviceConnected = true; 
        Serial.println(">>> Client connected <<<"); 
        Serial.flush();
    }
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        stop_motors();
        Serial.println(">>> Client disconnected <<<");
        Serial.flush();
        pServer->getAdvertising()->start();
        Serial.println("Restart advertising...");
        Serial.flush();
    }
};

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            Serial.printf("[BLE_RECEIVED] %s\n", value.c_str());
            Serial.flush();
            unified_parser(value);
        }
    }
};

// --- SETUP VÀ LOOP ---
void setup() {
    Serial.begin(115200);
    delay(1000); 
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.println("\n=== YOLO UNO CONTROL HUB - SIMPLIFIED VERSION ===");
    Serial.println("Firmware: OpenBot Only, No Gyro/PID");
    Serial.flush();

    // Initialize I2C with specific pins
    Wire.begin(SDA_PIN, SCL_PIN);
    delay(100);
    
    Serial.println("\n--- I2C INITIALIZATION ---");
    scan_i2c_devices();
    motor_driver_ok = test_motor_driver();
    
    if (motor_driver_ok) {
        // Configure encoders if needed
        set_i2c_encoder(E1, 100, 20, 30);
        set_i2c_encoder(E2, 100, 20, 30);
    }
    
    stop_motors();
    
    if (motor_driver_ok) {
        Serial.println("*** MOTOR TEST - Rotating each motor for 1 second ***");
        test_motors();
    } else {
        Serial.println("!!! WARNING: Motor driver not working - motors will not move !!!");
    }
    
    Serial.println("\n--- BLE INITIALIZATION ---");
    Serial.flush();
    
    BLEDevice::init(DEVICE_NAME);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    // Tạo characteristic cho OpenBot (WRITE + NOTIFY)
    BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
    );
    pRxCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
    pRxCharacteristic->addDescriptor(new BLE2902());
    
    pService->start();
    
    // Cấu hình advertising chuẩn OpenBot
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // Giúp kết nối với thiết bị iOS
    pAdvertising->setMaxPreferred(0x12);
    
    BLEDevice::startAdvertising();
    
    // In ra địa chỉ MAC để debug
    String btAddress = BLEDevice::getAddress().toString().c_str();
    Serial.printf(">>> BLE Setup Complete. Device: %s <<<\n", DEVICE_NAME);
    Serial.printf(">>> MAC Address: %s <<<\n", btAddress.c_str());
    Serial.println(">>> Waiting for OpenBot connection... <<<");
    Serial.println("\n=== SYSTEM READY ===");
    Serial.flush();
}

void loop() {
    // Chạy control_loop để xử lý cả PS4 và các nguồn khác
    control_loop();

    // Cập nhật trạng thái LED dựa trên kết nối
    if (deviceConnected) {
        digitalWrite(LED_BUILTIN, HIGH);
    } else {
        // Nhấp nháy LED để báo hiệu đang chờ kết nối BLE
        static unsigned long last_led_toggle = 0;
        if (millis() - last_led_toggle >= 200) {
            static bool led_state = false;
            led_state = !led_state;
            digitalWrite(LED_BUILTIN, led_state);
            last_led_toggle = millis();
        }
    }
    
    // Debug output định kỳ
    static unsigned long last_debug_output = 0;
    if (millis() - last_debug_output >= 5000) { // Mỗi 5 giây
        debug_output();
        last_debug_output = millis();
    }
    
    // Delay ngắn để không làm quá tải CPU
    delay(10);
}

// --- HÀM KIỂM TRA VÀ DEBUG ---
void scan_i2c_devices() {
    Serial.println("Scanning I2C devices...");
    Serial.flush();
    
    bool found_any = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  Found device at 0x%02X", addr);
            if (addr == I2C_MOTOR_DRIVER_ADDRESS) Serial.print(" (Motor Driver)");
            Serial.println();
            Serial.flush();
            found_any = true;
        }
    }
    
    if (!found_any) {
        Serial.println("  No I2C devices found!");
        Serial.flush();
    }
}

bool test_motor_driver() {
    Serial.print("Testing motor driver communication... ");
    Serial.flush();
    
    Wire.beginTransmission(I2C_MOTOR_DRIVER_ADDRESS);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
        Serial.println("OK!");
        Serial.flush();
        return true;
    } else {
        Serial.printf("FAILED! Error code: %d\n", error);
        Serial.flush();
        return false;
    }
}

void test_motors() {
    // Tốc độ cao hơn để kiểm tra rõ ràng hơn
    const int test_speed = 70;  // Tăng tốc độ test từ 50 lên 70
    const int test_duration = 1500;  // Tăng thời gian test từ 1000ms lên 1500ms
    
    Serial.println("\n*** MOTOR TEST WITH NEW PROTOCOL ***");
    Serial.println("Testing all motors at once...");
    
    // Test tất cả động cơ cùng lúc
    set_i2c_motor_speed(MOTOR_PORT_FL | MOTOR_PORT_FR | MOTOR_PORT_BL | MOTOR_PORT_BR, test_speed);
    delay(test_duration);
    stop_motors();
    delay(1000);
    
    Serial.println("Testing Front Left motor...");
    set_i2c_motor_speed(MOTOR_PORT_FL, test_speed);
    delay(test_duration);
    set_i2c_motor_speed(MOTOR_PORT_FL, 0);
    delay(500);
    
    Serial.println("Testing Front Right motor...");
    set_i2c_motor_speed(MOTOR_PORT_FR, test_speed);
    delay(test_duration);
    set_i2c_motor_speed(MOTOR_PORT_FR, 0);
    delay(500);
    
    Serial.println("Testing Back Left motor...");
    set_i2c_motor_speed(MOTOR_PORT_BL, test_speed);
    delay(test_duration);
    set_i2c_motor_speed(MOTOR_PORT_BL, 0);
    delay(500);
    
    Serial.println("Testing Back Right motor...");
    set_i2c_motor_speed(MOTOR_PORT_BR, test_speed);
    delay(test_duration);
    set_i2c_motor_speed(MOTOR_PORT_BR, 0);
    delay(500);
    
    // Test quay trái, quay phải
    Serial.println("Testing rotation (left)...");
    drive_mecanum(0, 0, -test_speed);
    delay(test_duration);
    stop_motors();
    delay(500);
    
    Serial.println("Testing rotation (right)...");
    drive_mecanum(0, 0, test_speed);
    delay(test_duration);
    stop_motors();
    delay(500);
    
    Serial.println("Motor test complete!\n");
    Serial.flush();
}

void debug_output() {
    Serial.println("\n--- DEBUG STATUS ---");
    Serial.printf("Time: %lu ms\n", millis());
    Serial.printf("BLE Connected: %s\n", deviceConnected ? "YES" : "NO");
    Serial.printf("Motor Driver: %s\n", motor_driver_ok ? "OK" : "FAILED");
    Serial.printf("OpenBot Commands: Drive=%d, Turn=%d\n", g_drive_y, g_turn_z);
    Serial.printf("Last Valid Commands: Drive=%d, Turn=%d\n", last_valid_drive_y, last_valid_turn_z);
    Serial.println("--- END DEBUG ---\n");
    Serial.flush();
}

// --- HÀM ĐIỀU KHIỂN CHÍNH ---
void control_loop() {
    int final_drive_y = 0;
    int final_turn_z = 0;
    
    // Xác định trạng thái OpenBot
    unsigned long now = millis();
    bool openbot_active = (now - last_openbot_cmd_time < CMD_TIMEOUT);
    
    // Xử lý lệnh OpenBot - chỉ lấy giá trị trực tiếp không qua xử lý
    if (openbot_active) {
        // Lấy giá trị trực tiếp từ OpenBot
        final_drive_y = g_drive_y;
        final_turn_z = g_turn_z;
        
        // Lưu lệnh hợp lệ cuối cùng
        if (final_drive_y != 0 || final_turn_z != 0) {
            last_valid_drive_y = final_drive_y;
            last_valid_turn_z = final_turn_z;
        } else {
            last_valid_drive_y = 0;
            last_valid_turn_z = 0;
        }
    } else {
        // Khi không có nguồn active, sử dụng lệnh cuối cùng hợp lệ
        // nếu thời gian chưa quá lâu
        if ((now - last_openbot_cmd_time < CMD_TIMEOUT * 3) && 
            (last_valid_drive_y != 0 || last_valid_turn_z != 0)) {
            final_drive_y = last_valid_drive_y;
            final_turn_z = last_valid_turn_z;
            
            static unsigned long last_keep_debug = 0;
            if (millis() - last_keep_debug >= 1000) {
                Serial.printf("[KEEP_COMMAND] Using last valid: Y=%d, Z=%d\n", 
                            final_drive_y, final_turn_z);
                Serial.flush();
                last_keep_debug = millis();
            }
        }
    }
    
    // Điều khiển motor - sử dụng giá trị trực tiếp không qua hiệu chỉnh
    static unsigned long last_final_debug = 0;
    if (millis() - last_final_debug >= 1000) {
        Serial.printf("[FINAL_VALUES] Drive_Y: %d, Turn_Z: %d (direct value)\n", 
                     final_drive_y, final_turn_z);
        Serial.flush();
        last_final_debug = millis();
    }
    
    if (final_drive_y == 0 && final_turn_z == 0) {
        static bool was_moving = false;
        if (was_moving) {
            Serial.println("[MOTOR] Stopping motors");
            Serial.flush();
            was_moving = false;
        }
        stop_motors();
    } else {
        // Log lệnh motor
        static unsigned long last_motor_debug = 0;
        if (millis() - last_motor_debug >= 500) {
            Serial.printf("[MOTOR_CMD] Y: %d, Z: %d (no PID correction)\n", 
                        final_drive_y, final_turn_z);
            Serial.flush();
            last_motor_debug = millis();
        }
        drive_mecanum(final_drive_y * 3, 0, final_turn_z * 3);
        static bool was_stopped = true;
        if (was_stopped) {
            Serial.println("[MOTOR] Starting motors");
            Serial.flush();
            was_stopped = false;
        }
    }
}

// --- BỘ PHÂN TÍCH LỆNH ĐA NĂNG ---
void unified_parser(const std::string& command) {
    // Phân tích theo định dạng OpenBot ("c<left_speed>,<right_speed>")
    if (command.length() > 0 && command[0] == 'c') {
        last_openbot_cmd_time = millis(); // Cập nhật thời gian nhận lệnh
        size_t comma_pos = command.find(',');
        if (comma_pos != std::string::npos) {
            // Parse left_speed và right_speed từ OpenBot
            int raw_left = atoi(command.substr(1, comma_pos - 1).c_str());
            int raw_right = atoi(command.substr(comma_pos + 1).c_str());
            
            // Chuyển đổi từ raw values (-255 to +255) sang percentage (-100 to +100)
            int left_speed = map(raw_left, -255, 255, -100, 100);
            int right_speed = map(raw_right, -255, 255, -100, 100);
            
            // CHUYỂN ĐỔI THÀNH DRIVE_Y VÀ TURN_Z CHO MECANUM ROBOT
            // drive_y = tốc độ trung bình (tiến/lùi)
            // turn_z = độ chênh lệch (xoay trái/phải)
            g_drive_y = (left_speed + right_speed) / 2;      // Tốc độ trung bình
            g_turn_z = (left_speed - right_speed) / 2;       // Sửa: left - right để đúng chiều xoay
            
            // Lệnh được chuyển trực tiếp (không qua gyro/PID) đến động cơ
            Serial.printf("[OPENBOT] Raw: L=%d,R=%d | Converted: L=%d,R=%d | Drive=%d,Turn=%d (direct)\n", 
                          raw_left, raw_right, left_speed, right_speed, g_drive_y, g_turn_z);
            Serial.flush();
        }
        return;
    }
    
    Serial.printf("[UNKNOWN] Command: %s\n", command.c_str());
    Serial.flush();
}

// --- CÁC HÀM TIỆN ÍCH ---
void set_i2c_motor_speed(uint8_t motor_mask, int speed) {
    if (!motor_driver_ok) {
        static unsigned long last_error_message = 0;
        if (millis() - last_error_message >= 5000) {  // Chỉ in lỗi mỗi 5 giây
            Serial.println("[ERROR] Motor driver not available!");
            last_error_message = millis();
        }
        return;
    }
    
    // Giới hạn tốc độ trong khoảng -100 đến 100
    speed = constrain(speed, -100, 100);
    
    // *** SỬA LỖI QUAN TRỌNG: Nhân tốc độ với 10 để có dải giá trị từ -1000 đến 1000 ***
    int16_t speed_value = speed * 10;
    
    // Gửi lệnh theo cách tương thích với thư viện Python
    const uint8_t MDV2_REG_MOTOR_INDEX_AND_SPEED = 0x10; // Thanh ghi 16 (0x10)
    
    // Tạo một buffer 4 byte để gửi đi một lần, mô phỏng chính xác hàm _write_16_array của Python
    // [byte 0: motor_mask_low, byte 1: motor_mask_high, byte 2: speed_low, byte 3: speed_high]
    uint8_t data_buffer[4] = {
        (uint8_t)(motor_mask & 0xFF),        // Byte thấp của motor_mask
        0,                                   // Byte cao của motor_mask (luôn là 0)
        (uint8_t)(speed_value & 0xFF),       // Byte thấp của tốc độ
        (uint8_t)((speed_value >> 8) & 0xFF) // Byte cao của tốc độ
    };
    
    Wire.beginTransmission(I2C_MOTOR_DRIVER_ADDRESS);
    Wire.write(MDV2_REG_MOTOR_INDEX_AND_SPEED); // Báo cho driver biết chúng ta sẽ ghi từ thanh ghi 16
    Wire.write(data_buffer, 4);                // Ghi toàn bộ 4 byte
    uint8_t error = Wire.endTransmission();
    
    if (error != 0) {
        Serial.printf("[I2C_ERROR] Motor command failed, error: %d\n", error);
        Serial.flush();
    } else {
        // Log successful motor commands more frequently during active movement
        static unsigned long last_motor_debug = 0;
        if (millis() - last_motor_debug >= 1000) {  // Debug mỗi 1 giây
            if (speed != 0) {
                Serial.printf("[MOTOR_SUCCESS] Mask=0x%02X, Speed=%d, Scaled=%d\n", 
                             motor_mask, speed, speed_value);
                Serial.flush();
                last_motor_debug = millis();
            }
        }
    }
}

// New function to setup encoder if needed
void set_i2c_encoder(uint8_t motorMask, uint16_t rpm, uint8_t ppr, uint8_t gear) {
    // Tạo buffer cho encoder setup theo định dạng của thư viện Python
    uint8_t data_buffer[6] = {
        (uint8_t)(motorMask & 0xFF),  // Byte thấp của motor_mask
        0,                            // Byte cao của motor_mask (luôn là 0)
        ppr,                          // Pulses per revolution
        gear,                         // Gear ratio
        (uint8_t)(rpm >> 8),          // Byte cao của RPM
        (uint8_t)(rpm & 0xFF)         // Byte thấp của RPM
    };
    
    Wire.beginTransmission(I2C_MOTOR_DRIVER_ADDRESS);
    Wire.write(REG_ENCODER_SETUP);
    Wire.write(data_buffer, 6);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
        Serial.printf("[Encoder Setup] Motor mask 0x%02X configured (RPM=%d, PPR=%d, Gear=%d)\n", 
                     motorMask, rpm, ppr, gear);
    } else {
        Serial.printf("[I2C_ERROR] Encoder setup failed, error: %d\n", error);
    }
    Serial.flush();
}

void drive_mecanum(int y, int x, int z) {
    int speed_fl = y + x + z;
    int speed_fr = y - x - z;
    int speed_bl = y - x + z;
    int speed_br = y + x - z;

    int max_val = std::max({abs(speed_fl), abs(speed_fr), abs(speed_bl), abs(speed_br)});
    
    if (max_val > 100) {
        speed_fl = map(speed_fl, -max_val, max_val, -100, 100);
        speed_fr = map(speed_fr, -max_val, max_val, -100, 100);
        speed_bl = map(speed_bl, -max_val, max_val, -100, 100);
        speed_br = map(speed_br, -max_val, max_val, -100, 100);
    }
    
    // Debug output cho motor speeds - chỉ in mỗi 2 giây (giá trị thực tế gửi tới motor)
    if (max_val > 0) {
        static unsigned long last_mecanum_debug = 0;
        if (millis() - last_mecanum_debug >= 2000) {  // Chỉ in mỗi 2 giây
            Serial.printf("[MECANUM] FL=%d, FR=%d, BL=%d, BR=%d (actual values sent)\n", 
                          -speed_fl, speed_fr, -speed_bl, speed_br);
            Serial.flush();
            last_mecanum_debug = millis();
        }
    }
    
    // SỬA LỖI: Đảo dấu cho motor bên phải để phù hợp với cấu trúc phần cứng
    set_i2c_motor_speed(MOTOR_PORT_FL, speed_fl);  
    set_i2c_motor_speed(MOTOR_PORT_FR, -speed_fr);   
    set_i2c_motor_speed(MOTOR_PORT_BL, speed_bl);  
    set_i2c_motor_speed(MOTOR_PORT_BR, -speed_br);
}

void stop_motors() {
    static unsigned long last_stop_debug = 0;
    if (millis() - last_stop_debug >= 1000) {
        Serial.println("[MOTOR] stop_motors() called");
        Serial.flush();
        last_stop_debug = millis();
    }
    set_i2c_motor_speed(M1 | M2 | E1 | E2, 0);
}
