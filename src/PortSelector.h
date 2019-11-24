#pragma once

/*
 * Modbus.cpp
 * 
 * Copyright 2019 PeerGum
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// This will load the definition for common Particle variable types
#include "Particle.h"

#ifndef __MODBUS_PORT_SELECTOR_H
#define __MODBUS_PORT_SELECTOR_H

class PortSelector {
    public:
        PortSelector(uint8_t pinCount, bool activeHigh, ...);
        void begin();
        void selectPort(uint8_t address);

    private:
        uint8_t _pinCount = 0;
        uint16_t *_pins;
        bool _activeHigh = true;
}

#endif