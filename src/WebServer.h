#pragma once
#include <Arduino.h>

class WebServerControl {
public:
    void begin();
    void handleClient();
    String getCommand();
    bool isBalanceEnabled();
private:
    String lastCommand;
};
