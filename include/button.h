#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>

// Định nghĩa trạng thái nút
#define NORMAL_STATE HIGH
#define PRESSED_STATE LOW

// Thời gian debounce và long press (ms)
#define DEBOUNCE_DURATION 30
#define LONG_PRESS_DURATION 200

// Số lượng nút
#define NUM_BUTTONS 2

// Trạng thái nút
enum ButtonState {
    IDLE,
    DEBOUNCE,
    PRESSED,
    LONG_PRESSED
};

// Khai báo các biến và hàm
extern int buttonState[NUM_BUTTONS];
extern int button_flag[NUM_BUTTONS];
extern const int buttonPins[NUM_BUTTONS];
int isButtonPressed(int index);
int isButtonLongPressed(int index);
void getKeyInput();

#ifdef __cplusplus
}
#endif

#endif /* INC_BUTTON_H_ */