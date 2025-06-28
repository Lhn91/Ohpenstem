/**
 * @file main.cpp
 * @brief Firmware C++ đa năng cho Yolo UNO (Control Hub), hỗ trợ OpenBot, OhStem Gamepad và PS4 Controller.
 *
 * PHIÊN BẢN ALL-IN-ONE PLUS PS4 VỚI DEBUG:
 * - Tự động nhận diện và xử lý lệnh từ ba nguồn: OpenBot App, OhStem Gamepad và PS4 Controller.
 * - OpenBot: BLE với lệnh "c<drive>,<turn>"
 * - OhStem Gamepad: BLE với format "KEY:VALUE"  
 * - PS4 Controller: I2C qua PS4 Receiver module (địa chỉ 0x55)
 * - Tích hợp logic PID và cảm biến góc (gyro) để tự động giữ hướng đi thẳng.
 * - Hỗ trợ đầy đủ các tính năng của PS4: analog stick, D-pad, L1/R1 trượt ngang, R2 turbo.
 * - THÊM CHỨC NĂNG DEBUG CHO VIỆC KIỂM TRA MOTOR VÀ I2C
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <algorithm>
#include <map>

// --- CẤU HÌNH ---
#define I2C_MOTOR_DRIVER_ADDRESS 0x54
#define PS4_RECEIVER_ADDRESS 0x55  // Địa chỉ I2C của PS4 receiver
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

// UUIDs Chuẩn UART - Tương thích với cả OpenBot và các app BLE Terminal/Gamepad
#define SERVICE_UUID             "61653dc3-4021-4d1e-ba83-8b4eec61d613" // UART service UUID
#define CHARACTERISTIC_UUID_RX   "06386c14-86ea-4d71-811c-48f97c58f8c9" // RX: Nhận lệnh từ điện thoại
#define CHARACTERISTIC_UUID_TX   "9bf1103b-834c-47cf-b149-c9e4bcf778a7" // TX: Gửi dữ liệu lên điện thoại


enum ControlMode { MODE_OPENBOT, MODE_GAMEPAD, MODE_PS4 };
enum Direction { 
    DIR_FW=0, DIR_RF=1, DIR_R=2, DIR_RB=3, DIR_BW=4, 
    DIR_LB=5, DIR_L=6, DIR_LF=7, DIR_SL=8, DIR_SR=9, DIR_NONE=-1 
};

// --- CẤU TRÚC DỮ LIỆU PS4 ---
struct PS4Data {
    bool is_connected = false;
    uint8_t dpad = 0;
    int32_t aLX = 0, aLY = 0, aRX = 0, aRY = 0;
    int32_t al2 = 0, ar2 = 0;
    uint16_t buttons = 0;
    uint16_t misc_buttons = 0;
    
    // Giá trị trực tiếp từ I2C
    int8_t y_value = 0;
    int8_t turn_value = 0;
    
    // Parsed buttons
    bool dpad_up = false, dpad_down = false, dpad_left = false, dpad_right = false;
    bool square = false, triangle = false, cross = false, circle = false;
    bool l1 = false, r1 = false, l2 = false, r2 = false;
    bool thumbl = false, thumbr = false;
    bool share = false, options = false, ps = false;
};

// --- BIẾN TOÀN CỤC ---
BLEServer* pServer = NULL;
bool deviceConnected = false;
Adafruit_MPU6050 mpu;
bool mpu_ok = false;
bool motor_driver_ok = false;  // Thêm biến kiểm tra motor driver
volatile ControlMode current_mode = MODE_OPENBOT;

// Biến để theo dõi thời gian nhận lệnh cuối cùng từ mỗi nguồn
unsigned long last_openbot_cmd_time = 0;
unsigned long last_gamepad_cmd_time = 0;
unsigned long last_ps4_cmd_time = 0;
const unsigned long CMD_TIMEOUT = 500; // Coi như mất kết nối nếu không có lệnh mới sau 500ms

// Biến cho OpenBot
volatile int g_drive_y = 0;
volatile int g_turn_z = 0;

// Biến cho Gamepad
std::map<std::string, int> gamepadState;
Direction last_gamepad_dir = DIR_NONE;
int gamepad_speed = 0;
const int MIN_SPEED = 40, MAX_SPEED = 100, ACCEL_STEPS = 5;

// Biến cho PS4 Controller
PS4Data ps4;
Direction last_ps4_dir = DIR_NONE;
int ps4_speed = 0;
unsigned long last_ps4_update = 0;
const unsigned long PS4_UPDATE_INTERVAL = 50; // 20Hz

// Biến cho Gyro & PID
float current_heading = 0.0;
unsigned long last_update_time_micros = 0;
double Kp = 2.5, Ki = 0.1, Kd = 0.2;
double pid_integral = 0, pid_last_error = 0;

// Biến debug
unsigned long last_debug_output = 0;
const unsigned long DEBUG_INTERVAL = 5000; // Debug mỗi 5 giây

// --- KHAI BÁO HÀM ---
void set_i2c_motor_speed(uint8_t motor_mask, int speed);
void set_i2c_encoder(uint8_t motorMask, uint16_t rpm, uint8_t ppr, uint8_t gear);
void drive_mecanum(int y, int x, int z);
void run_direction(Direction dir, int speed);
void stop_motors();
void unified_parser(const std::string& command);
void setup_imu();
void control_loop();
void reset_heading();
bool test_motor_driver();
void scan_i2c_devices();
void debug_output();
void test_motors();

// PS4 Controller functions
bool update_ps4_data();
void parse_ps4_buttons();
int32_t read_32bit_i2c(uint8_t* data, int start_index);
uint16_t read_16bit_i2c(uint8_t* data, int start_index);
Direction get_ps4_direction();

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
    Serial.println("\n=== YOLO UNO CONTROL HUB DEBUG VERSION ===");
    Serial.println("Firmware: Universal (OpenBot + Gamepad + PS4) with DEBUG");
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
    
    setup_imu();
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
    Serial.println(">>> Waiting for OpenBot/Gamepad connection... <<<");
    Serial.println("\n=== SYSTEM READY ===");
    Serial.flush();
}

void loop() {
    // Luôn chạy control_loop để xử lý cả PS4 và các nguồn khác, bất kể BLE có kết nối hay không
    control_loop();

    if (deviceConnected) {
        digitalWrite(LED_BUILTIN, HIGH);
    } else {
        // Nhấp nháy LED để báo hiệu đang chờ kết nối BLE
        digitalWrite(LED_BUILTIN, LOW); delay(200);
        digitalWrite(LED_BUILTIN, HIGH); delay(200);
    }
    
    // Debug output định kỳ
    if (millis() - last_debug_output >= DEBUG_INTERVAL) {
        debug_output();
        last_debug_output = millis();
    }
    
    delay(20);
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
            if (addr == PS4_RECEIVER_ADDRESS) Serial.print(" (PS4 Receiver)");
            if (addr == 0x68) Serial.print(" (MPU6050)");
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
    Serial.printf("MPU6050: %s\n", mpu_ok ? "OK" : "FAILED");
    Serial.printf("Control Mode: %s\n", 
        current_mode == MODE_OPENBOT ? "OpenBot" :
        current_mode == MODE_GAMEPAD ? "Gamepad" : "PS4");
    Serial.printf("OpenBot Commands: Drive=%d, Turn=%d\n", g_drive_y, g_turn_z);
    Serial.printf("PS4 Connected: %s\n", ps4.is_connected ? "YES" : "NO");
    
    if (current_mode == MODE_GAMEPAD && !gamepadState.empty()) {
        Serial.print("Gamepad State: ");
        for (auto& pair : gamepadState) {
            if (pair.second != 0) {
                Serial.printf("%s=%d ", pair.first.c_str(), pair.second);
            }
        }
        Serial.println();
    }
    
    Serial.println("--- END DEBUG ---\n");
    Serial.flush();
}

// --- HÀM ĐIỀU KHIỂN CHÍNH ---
void control_loop() {
    int final_drive_y = 0;
    int final_turn_z = 0;

    // 1. Cập nhật PS4 controller nếu đã đến thời gian
    if (millis() - last_ps4_update >= PS4_UPDATE_INTERVAL) {
        update_ps4_data(); // Hàm này chỉ thay đổi mode khi có lệnh thực sự
        last_ps4_update = millis();
    }
    
    // 2. Xác định nguồn điều khiển hiện tại dựa trên thời gian lệnh gần nhất
    unsigned long now = millis();
    const unsigned long REAL_CMD_TIMEOUT = 1000; // Thời gian timeout dài hơn
    
    // Kiểm tra từng nguồn và thời gian lệnh cuối
    bool openbot_active = (now - last_openbot_cmd_time < REAL_CMD_TIMEOUT);
    bool gamepad_active = (now - last_gamepad_cmd_time < REAL_CMD_TIMEOUT);
    bool ps4_active = (now - last_ps4_cmd_time < REAL_CMD_TIMEOUT);
    
    // Chọn nguồn điều khiển ưu tiên dựa trên timestamp gần nhất
    ControlMode active_mode = MODE_OPENBOT; // Mặc định không làm gì
    unsigned long latest_time = 0;
    
    // So sánh thời gian để tìm nguồn điều khiển mới nhất
    if (openbot_active && last_openbot_cmd_time > latest_time) {
        latest_time = last_openbot_cmd_time;
        active_mode = MODE_OPENBOT;
    }
    
    if (gamepad_active && last_gamepad_cmd_time > latest_time) {
        latest_time = last_gamepad_cmd_time;
        active_mode = MODE_GAMEPAD;
    }
    
    if (ps4_active && last_ps4_cmd_time > latest_time) {
        latest_time = last_ps4_cmd_time;
        active_mode = MODE_PS4;
    }
    
    // Nếu không có nguồn nào active, cho motor dừng và thoát
    if (!openbot_active && !gamepad_active && !ps4_active) {
        stop_motors();
        return;
    }
    
    // Cập nhật biến global cho debug
    current_mode = active_mode;
    
    // 3. Xử lý lệnh dựa trên nguồn được chọn
    if (active_mode == MODE_PS4) {
        // --- CODE PS4 ĐƠN GIẢN HÓA ---
        // Sử dụng giá trị đã lưu trong ps4.y_value và ps4.turn_value
        final_drive_y = ps4.y_value * 2; // Nhân 2 để tăng gain (phạm vi: +/- 100)
        final_turn_z = ps4.turn_value * 2;
        
        // Giới hạn giá trị trong phạm vi -100 đến 100
        final_drive_y = constrain(final_drive_y, -100, 100);
        final_turn_z = constrain(final_turn_z, -100, 100);
    } 
    else if (active_mode == MODE_OPENBOT) {
        // Giữ nguyên logic OpenBot
        final_drive_y = g_drive_y; 
        final_turn_z = g_turn_z;
    } 
    else if (active_mode == MODE_GAMEPAD) {
        // Đơn giản hóa logic Gamepad
        if (gamepadState["U"]) final_drive_y = 100;
        else if (gamepadState["D"]) final_drive_y = -100;
        
        if (gamepadState["L"]) final_turn_z = -100;
        else if (gamepadState["R"]) final_turn_z = 100;
        
        // Xử lý analog stick nếu cần
        if (abs(gamepadState["ALY"]) > 20) {
            final_drive_y = gamepadState["ALY"];
        }
        if (abs(gamepadState["ALX"]) > 20) {
            final_turn_z = gamepadState["ALX"];
        }
    }

    // 4. Áp dụng ổn định Gyro nếu đang đi thẳng
    if (mpu_ok && final_turn_z == 0 && final_drive_y != 0) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        unsigned long now = micros();
        float dt = (now - last_update_time_micros) / 1000000.0F;
        last_update_time_micros = now;

        current_heading += degrees(g.gyro.z * dt);

        float error = 0 - current_heading;
        pid_integral += error * dt;
        float derivative = (error - pid_last_error) / dt;
        final_turn_z = Kp * error + Ki * pid_integral + Kd * derivative;
        pid_last_error = error;
    } else {
        reset_heading();
    }

    // 5. Log và gửi lệnh đến động cơ
    static unsigned long last_motor_debug = 0;
    if (millis() - last_motor_debug >= 1000) {
        Serial.printf("[MOTOR_CMD] Y=%d, Z=%d (Mode: %s)\n", 
                     final_drive_y, final_turn_z,
                     current_mode == MODE_OPENBOT ? "OpenBot" :
                     current_mode == MODE_GAMEPAD ? "Gamepad" : "PS4");
        Serial.flush();
        last_motor_debug = millis();
    }

    // Điều khiển mọi động cơ dừng khi không có lệnh nào
    if (final_drive_y == 0 && final_turn_z == 0) {
        stop_motors();
    } else {
        drive_mecanum(final_drive_y, 0, final_turn_z);
    }
}

// --- BỘ PHÂN TÍCH LỆNH ĐA NĂNG ---
void unified_parser(const std::string& command) {
    // Thử phân tích theo định dạng Gamepad ("KEY:VALUE")
    size_t colon_pos = command.find(':');
    if (colon_pos != std::string::npos) {
        current_mode = MODE_GAMEPAD;
        last_gamepad_cmd_time = millis(); // Cập nhật thời gian nhận lệnh
        std::string key = command.substr(0, colon_pos);
        int value = atoi(command.substr(colon_pos + 1).c_str());

        // Xử lý analog stick (AL hoặc AR)
        if (key == "AL" || key == "AR") {
            int x = (value >> 8) & 0xFF;
            int y = value & 0xFF;
            if (y > 100) y = -(256 - y);  // Chuyển đổi từ unsigned sang signed
            if (x > 100) x = -(256 - x);
            gamepadState[key+"X"] = x;
            gamepadState[key+"Y"] = y;
            Serial.printf("[GAMEPAD] %s: X=%d, Y=%d\n", key.c_str(), x, y);
            Serial.flush();
        } else {
            gamepadState[key] = value;
            if (value > 0) {
                Serial.printf("[GAMEPAD] %s: %d\n", key.c_str(), value);
                Serial.flush();
            }
        }
        return;
    }
    
    // Nếu không phải Gamepad, thử phân tích theo định dạng OpenBot ("c<left_speed>,<right_speed>")
    if (command.length() > 0 && command[0] == 'c') {
        current_mode = MODE_OPENBOT;
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
            
            Serial.printf("[OPENBOT] Raw: L=%d,R=%d | Converted: L=%d,R=%d | Drive=%d,Turn=%d\n", 
                          raw_left, raw_right, left_speed, right_speed, g_drive_y, g_turn_z);
            Serial.flush();
        }
        return;
    }
    
    Serial.printf("[UNKNOWN] Command: %s\n", command.c_str());
    Serial.flush();
}


// --- CÁC HÀM TIỆN ÍCH ---
void setup_imu() {
    Serial.print("Testing MPU6050... ");
    Serial.flush();
    if (!mpu.begin()) {
        Serial.println("FAILED! Gyro stabilization will be disabled.");
        Serial.flush();
        mpu_ok = false;
    } else {
        Serial.println("OK!");
        Serial.flush();
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        mpu_ok = true;
    }
}

void reset_heading() {
    current_heading = 0;
    pid_integral = 0;
    pid_last_error = 0;
    last_update_time_micros = micros();
}

// Updated motor control function using the working protocol
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
    }
    
    // Debug output cho tốc độ motor
    static unsigned long last_motor_debug = 0;
    if (millis() - last_motor_debug >= 5000) {  // Chỉ in debug mỗi 5 giây
        if (speed != 0) {
            Serial.printf("[MOTOR] Mask=0x%02X, Speed=%d, Scaled=%d\n", 
                         motor_mask, speed, speed_value);
            Serial.flush();
            last_motor_debug = millis();
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
    
    // Debug output cho motor speeds - chỉ in mỗi 2 giây
    if (max_val > 0) {
        static unsigned long last_mecanum_debug = 0;
        if (millis() - last_mecanum_debug >= 2000) {  // Chỉ in mỗi 2 giây
            Serial.printf("[MECANUM] FL=%d, FR=%d, BL=%d, BR=%d\n", 
                          -speed_fl, speed_fr, -speed_bl, speed_br);
            Serial.flush();
            last_mecanum_debug = millis();
        }
    }
    
    // Đảo dấu cho motor bên trái để phù hợp với cấu trúc phần cứng
    set_i2c_motor_speed(MOTOR_PORT_FL, speed_fl);  // Đảo dấu
    set_i2c_motor_speed(MOTOR_PORT_FR, -speed_fr);
    set_i2c_motor_speed(MOTOR_PORT_BL, speed_bl);  // Đảo dấu  
    set_i2c_motor_speed(MOTOR_PORT_BR, -speed_br);
}

void stop_motors() {
    set_i2c_motor_speed(M1 | M2 | E1 | E2, 0);
}

// --- CÁC HÀM PS4 CONTROLLER ---
int32_t read_32bit_i2c(uint8_t* data, int start_index) {
    // Đọc 32-bit signed little endian từ 4 bytes
    int32_t raw = (data[start_index] << 24) | (data[start_index+1] << 16) | 
                  (data[start_index+2] << 8) | data[start_index+3];
    return raw;
}

uint16_t read_16bit_i2c(uint8_t* data, int start_index) {
    // Đọc 16-bit little endian từ 2 bytes
    return (data[start_index] << 8) | data[start_index+1];
}

bool update_ps4_data() {
    // Request just y and turn values (2 bytes) from PS4 receiver
    Wire.beginTransmission(PS4_RECEIVER_ADDRESS);
    if (Wire.endTransmission() != 0) {
        if (ps4.is_connected) {
            Serial.println("[PS4] Disconnected");
            Serial.flush();
        }
        ps4.is_connected = false;
        return false;
    }
    
    Wire.requestFrom(PS4_RECEIVER_ADDRESS, 2);  // Only request 2 bytes
    if (Wire.available() >= 2) {
        int8_t y = (int8_t)Wire.read();      // First byte is Y value
        int8_t turn = (int8_t)Wire.read();   // Second byte is turn value
        
        // Update PS4 data structure with these values
        ps4.is_connected = true;
        ps4.aLY = -y * 5;  // Scale and invert for compatibility
        ps4.aLX = turn * 5;  // Scale for compatibility
        
        // Set appropriate direction flags based on y/turn values
        ps4.dpad_up = (y > 20);
        ps4.dpad_down = (y < -20);
        ps4.dpad_right = (turn > 20);
        ps4.dpad_left = (turn < -20);
        
        // Áp dụng ngưỡng lọc tín hiệu nhiễu/chờ từ PS4 controller
        const int PS4_THRESHOLD = 10;  // Ngưỡng cao hơn để bỏ qua tín hiệu nhiễu/offset
        bool is_active_command = (abs(y) > PS4_THRESHOLD || abs(turn) > PS4_THRESHOLD);
        
        // Chỉ in log nếu có lệnh thực sự hoặc định kỳ
        static unsigned long last_ps4_log = 0;
        if (is_active_command || millis() - last_ps4_log >= 5000) {
            Serial.printf("[PS4] Y=%d, Turn=%d %s\n", y, turn, 
                         is_active_command ? "(ACTIVE)" : "");
            Serial.flush();
            last_ps4_log = millis();
        }
        
        // Lưu lệnh PS4 vào biến global cho control_loop xử lý
        ps4.y_value = y;
        ps4.turn_value = turn;
        
        // Chỉ thay đổi mode và cập nhật timestamp khi có lệnh thực sự 
        if (is_active_command) {
            current_mode = MODE_PS4;
            last_ps4_cmd_time = millis();
        }
        
        return true;
    }
    
    if (ps4.is_connected) {
        Serial.println("[PS4] Communication error");
        Serial.flush();
    }
    ps4.is_connected = false;
    return false;
}

void parse_ps4_buttons() {
    // Parse D-pad
    ps4.dpad_up = (ps4.dpad >> 0) & 1;
    ps4.dpad_down = (ps4.dpad >> 1) & 1;
    ps4.dpad_right = (ps4.dpad >> 2) & 1;
    ps4.dpad_left = (ps4.dpad >> 3) & 1;
    
    // Parse main buttons
    ps4.cross = (ps4.buttons >> 0) & 1;     // A button
    ps4.circle = (ps4.buttons >> 1) & 1;    // B button
    ps4.square = (ps4.buttons >> 2) & 1;    // X button
    ps4.triangle = (ps4.buttons >> 3) & 1;  // Y button
    ps4.l1 = (ps4.buttons >> 4) & 1;
    ps4.r1 = (ps4.buttons >> 5) & 1;
    ps4.l2 = (ps4.buttons >> 6) & 1;
    ps4.r2 = (ps4.buttons >> 7) & 1;
    ps4.thumbl = (ps4.buttons >> 8) & 1;
    ps4.thumbr = (ps4.buttons >> 9) & 1;
    
    // Parse misc buttons
    ps4.ps = (ps4.misc_buttons >> 0) & 1;     // PS/Home button
    ps4.share = (ps4.misc_buttons >> 1) & 1;  // Share/Select button
    ps4.options = (ps4.misc_buttons >> 2) & 1; // Options/Start button
}

Direction get_ps4_direction() {
    // Ưu tiên kiểm tra analog stick trái
    if (abs(ps4.aLY) > 50 || abs(ps4.aLX) > 50) {
        int x = map(ps4.aLX, -512, 512, -100, 100);
        int y = map(-ps4.aLY, -512, 512, -100, 100); // Đảo Y
        
        int distance = sqrt(x*x + y*y);
        if (distance < 20) return DIR_NONE;
        
        float angle = atan2(y, x) * 180.0 / PI;
        if (angle < 0) angle += 360;
        
        if (angle >= 350 || angle < 10) return DIR_R;
        else if (angle >= 15 && angle < 75) return DIR_RF;
        else if (angle >= 80 && angle < 110) return DIR_FW;
        else if (angle >= 115 && angle < 165) return DIR_LF;
        else if (angle >= 170 && angle < 190) return DIR_L;
        else if (angle >= 195 && angle < 255) return DIR_LB;
        else if (angle >= 260 && angle < 280) return DIR_BW;
        else if (angle >= 285 && angle < 345) return DIR_RB;
    }
    
    // Fallback to D-pad
    if (ps4.dpad_up) return DIR_FW;
    if (ps4.dpad_down) return DIR_BW;
    if (ps4.dpad_left) return DIR_L;
    if (ps4.dpad_right) return DIR_R;
    
    return DIR_NONE;
}
