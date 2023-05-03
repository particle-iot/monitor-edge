/*
 * Copyright (c) 2023 Particle Industries, Inc.
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

#pragma once

#include "cloud_service.h"

using ShippingModeCb = std::function<int(void)>;

class EdgeShipping
{
    public:
        static EdgeShipping& instance() {
            if(!_instance)
            {
                _instance = new EdgeShipping();
            }
            return *_instance;
        }

        void init();

        int enter(bool checkPower = false);

        int regShutdownBeginCallback(ShippingModeCb begin);
        int regShutdownIoCallback(ShippingModeCb io);
        int regShutdownFinalCallback(ShippingModeCb final);
    private:
        EdgeShipping() :
            _beginCallback(nullptr),
            _ioCallback(nullptr),
            _finalCallback(nullptr),
            _checkPower(false),
            _pmicFire(false) {}
        static EdgeShipping* _instance;

        ShippingModeCb _beginCallback;
        ShippingModeCb _ioCallback;
        ShippingModeCb _finalCallback;
        bool _checkPower;
        bool _pmicFire;

        int enter_cb(JSONValue *root);
        void shutdown();
        static void pmicHandler();
};

// These are here for compatibility
using TrackerShipping = EdgeShipping;
