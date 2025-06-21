#include "button.h"

// Khai báo các biến toàn cục
int buttonState[NUM_BUTTONS] = { IDLE, IDLE };
int TimeOutForKeyPress[NUM_BUTTONS] = { 0, 0 };
int button_flag[NUM_BUTTONS] = { 0, 0 };
int KeyReg0[NUM_BUTTONS] = { NORMAL_STATE, NORMAL_STATE };
int KeyReg1[NUM_BUTTONS] = { NORMAL_STATE, NORMAL_STATE };
int KeyReg2[NUM_BUTTONS] = { NORMAL_STATE, NORMAL_STATE };
int KeyRegb[NUM_BUTTONS];

// Chân GPIO của các nút
const int buttonPins[NUM_BUTTONS] = { GPIO_NUM_8, GPIO_NUM_9}; // Thay đổi theo thiết kế của bạn

// Hàm kiểm tra nút nhấn
int isButtonPressed(int index) {
    if (button_flag[index] == 1) {
        button_flag[index] = 0;
        return 1;
    }
    return 0;
}

int isButtonLongPressed(int index) {
    if (button_flag[index] == 2) {
        button_flag[index] = 0;
        return 1;
    }
    return 0;
}

// Hàm xử lý trạng thái nút
void subKeyProcess(int index) {
    if (buttonState[index] == PRESSED) {
        button_flag[index] = 1;
    } else if (buttonState[index] == LONG_PRESSED) {
        button_flag[index] = 2;
    }
}

// Hàm đọc trạng thái nút
void getKeyInput() {
    for (int i = 0; i < NUM_BUTTONS; i++) {
        KeyRegb[i] = digitalRead(buttonPins[i]); // Đọc trạng thái nút từ GPIO

        KeyReg2[i] = KeyReg1[i];
        KeyReg1[i] = KeyReg0[i];
        KeyReg0[i] = KeyRegb[i];

        switch (buttonState[i]) {
        case IDLE:
            if (KeyReg0[i] == PRESSED_STATE && KeyReg1[i] == KeyReg0[i] && KeyReg2[i] == KeyReg1[i]) {
                buttonState[i] = DEBOUNCE;
                TimeOutForKeyPress[i] = DEBOUNCE_DURATION;
            }
            break;

        case DEBOUNCE:
            if (TimeOutForKeyPress[i] > 0) {
                TimeOutForKeyPress[i]--;
            } else {
                if (KeyReg0[i] == PRESSED_STATE) {
                    buttonState[i] = PRESSED;
                    TimeOutForKeyPress[i] = LONG_PRESS_DURATION;
                    subKeyProcess(i);
                } else {
                    buttonState[i] = IDLE;
                }
            }
            break;

        case PRESSED:
            if (KeyReg0[i] == NORMAL_STATE) {
                buttonState[i] = IDLE;
            } else {
                if (TimeOutForKeyPress[i] > 0) {
                    TimeOutForKeyPress[i]--;
                } else {
                    buttonState[i] = LONG_PRESSED;
                    subKeyProcess(i);
                }
            }
            break;

        case LONG_PRESSED:
            if (KeyReg0[i] == NORMAL_STATE) {
                buttonState[i] = IDLE;
            } else {
                subKeyProcess(i);
                TimeOutForKeyPress[i] = LONG_PRESS_DURATION;
            }
            break;

        default:
            buttonState[i] = IDLE;
            break;
        }
    }
}