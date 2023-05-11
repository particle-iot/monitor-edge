/*
 * Copyright (c) 2023 Particle Industries, Inc.  All rights reserved.
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

#include "dct.h"
#include "deviceid_hal.h"
#include "exrtc_hal.h"

#include "tracker_config.h"

#include "cloud_service.h"
#include "config_service.h"
#include "edge_gnss_abstraction.h"
#include "edge_motion.h"

#include "edge_sleep.h"
#include "edge_location.h"
#include "edge_motion_configuration.h"
#include "edge_shipping.h"
#include "edge_temperature.h"
#include "mcp_can.h"
#ifdef EDGE_USE_MEMFAULT
#include "memfault.h"
#endif // EDGE_USE_MEMFAULT
#include "IEdgePlatformConfiguration.h"

// !!!TODO!!!
// * replace this header with more general purpose Monitor One one
// * Handle the GnssLedEnable() and GnssLedInit() calls in model specific sources
#include "monitor_one_gnss_led.h"

//
// Default configuration
//
#ifndef TRACKER_CONFIG_ENABLE_IO
// Enable or disable IO/CAN power at initialization, see TrackerConfiguration below
#define TRACKER_CONFIG_ENABLE_IO              (true)
#endif

#ifndef TRACKER_CONFIG_ENABLE_IO_SLEEP
// Enable or disable IO/CAN power shutdown prior to sleep, see TrackerConfiguration below
#define TRACKER_CONFIG_ENABLE_IO_SLEEP        (false)
#endif

#ifndef TRACKER_CONFIG_DISABLE_CHARGING
// Enable or disable LiPo charging. Also available to the user application. See TrackerConfiguration below
#define TRACKER_CONFIG_DISABLE_CHARGING       (false)
#endif

#ifndef TRACKER_CONFIG_ENABLE_FAST_LOCK
// Enable or disable faster GNSS lock based on HDOP, see TrackerConfiguration below
#define TRACKER_CONFIG_ENABLE_FAST_LOCK       (false)
#endif

#ifndef TRACKER_CONFIG_GNSS_RETRY_COUNT
// GNSS initialization retry count, see TrackerConfiguration below
#define TRACKER_CONFIG_GNSS_RETRY_COUNT       (1)
#endif


struct EdgeCloudConfig {
    bool UsbCommandEnable;
};

enum class EdgeChargeState {
    CHARGE_INIT,
    CHARGE_DONT_CARE,
    CHARGE_CARE,
};

struct EdgeChargeStatus {
    unsigned int uptime;
    EdgeChargeState state;
};

/**
 * @brief TrackerConfiguration class to configure the tracker device in application
 *
 */
class EdgeConfiguration {
public:
    /**
     * @brief Construct a new Edge Configuration object
     *
     */
    EdgeConfiguration() :
        _enableIo(TRACKER_CONFIG_ENABLE_IO),
        _enableIoSleep(TRACKER_CONFIG_ENABLE_IO_SLEEP),
        _disableCharging(TRACKER_CONFIG_DISABLE_CHARGING),
        _gnssRetryCount(TRACKER_CONFIG_GNSS_RETRY_COUNT),
        _locationServiceConfig(EdgeGnssConfiguration()) {
    }

    /**
     * @brief Construct a new Tracker Configuration object
     *
     */
    EdgeConfiguration(EdgeConfiguration&&) = default;

    /**
     * @brief Enable or disable IO/CAN power at initialization
     *
     * @param enable Turn IO/power on CAN power at initialization
     * @return TrackerConfiguration&
     */
    EdgeConfiguration& enableIoCanPower(bool enable) {
        _enableIo = enable;
        return *this;
    }

    /**
     * @brief Indicate if IO/CAN power is powered on at initialization
     *
     * @return true Powered at initialization
     * @return false Not powered at initialization
     */
    bool enableIoCanPower() const {
        return _enableIo;
    }

    /**
     * @brief Enable or disable IO/CAN power shutdown prior to sleep
     *
     * @param enable Power down IO/CAN power before entering sleep; otherwise, leave alone for user
     * @return TrackerConfiguration&
     */
    EdgeConfiguration& enableIoCanPowerSleep(bool enable) {
        _enableIoSleep = enable;
        return *this;
    }

    /**
     * @brief Indicate if IO/CAN power will be powered down prior to sleep
     *
     * @return true Powered off at sleep
     * @return false IO/CAN power under user control
     */
    bool enableIoCanPowerSleep() const {
        return _enableIoSleep;
    }

    /**
     * @brief Disable or enable LiPo battery charging. Can be overridden in the user app with forceDisableCharging()
     *
     * @param disable Disable charging of the LiPo
     * @return TrackerConfiguration&
     */
    EdgeConfiguration& disableCharging(bool disable) {
        _disableCharging = disable;
        return *this;
    }

    /**
     * @brief Indicate if charging is disabled or not
     *
     * @return true Charging is disabled
     * @return false Charging is enabled
     */
    bool disableCharging() const {
        return _disableCharging;
    }

    /**
     * @brief Enable or disable faster GNSS lock based on HDOP.  May result in poor horizontal accuracy.
     *
     * @param enable Use faster method for GNSS lock state
     * @return TrackerConfiguration&
     */
    EdgeConfiguration& enableFastLock(bool enable) {
        _locationServiceConfig.enableFastLock(enable);
        return *this;
    }

    /**
     * @brief Indicate if faster GNSS lock based on HDOP is enabled.
     *
     * @return true Faster GNSS lock is enabled
     * @return false Faster GNSS lock is disabled
     */
    bool enableFastLock() const {
        return _locationServiceConfig.enableFastLock();
    }

    /**
     * @brief Set GNSS initialization retry count.
     *
     * @param count Number of retry attemps for GNSS initialization
     * @return TrackerConfiguration&
     */
    EdgeConfiguration& gnssRetryCount(unsigned int count) {
        _gnssRetryCount = count;
        return *this;
    }

    /**
     * @brief Get GNSS initialization retry count.
     *
     * @return unsigned int Number of retry attemps for GNSS initialization
     */
    unsigned int gnssRetryCount() const {
        return _gnssRetryCount;
    }

    /**
     * @brief Set EdgeGnssConfiguration object
     *
     * @param locServConfig config object for the Location Service
     * @return TrackerConfiguration&
     */
    EdgeConfiguration& locationServiceConfig(EdgeGnssConfiguration& locServConfig) {
        _locationServiceConfig = locServConfig;
        return *this;
    }

    /**
     * @brief Get EdgeGnssConfiguration object
     *
     * @return Location Service configuration object reference
     */
    EdgeGnssConfiguration& locationServiceConfig() {
        return _locationServiceConfig;
    }

    EdgeConfiguration& operator=(const EdgeConfiguration& rhs) {
        if (this == &rhs) {
            return *this;
        }
        this->_enableIo = rhs._enableIo;
        this->_enableIoSleep = rhs._enableIoSleep;
        this->_disableCharging = rhs._disableCharging;
        this->_gnssRetryCount = rhs._gnssRetryCount;
        this->_locationServiceConfig = rhs._locationServiceConfig;

        return *this;
    }
private:
    bool _enableIo;
    bool _enableIoSleep;
    bool _disableCharging;
    unsigned int _gnssRetryCount;
    EdgeGnssConfiguration _locationServiceConfig;
};

// this class encapsulates the underlying modules and builds on top of them to
// provide a cohesive asset tracking application
class Edge {
    public:
        static Edge &instance() {
            if(!_instance) {
                _instance = new Edge();
            }
            return *_instance;
        }

        /**
         * @brief Startup for early device intitialization
         *
         */
        static void startup();

        /**
         * @brief Initialize device for application setup()
         *
         * @retval SYSTEM_ERROR_NONE
         */
        int init();

        /**
         * @brief Initializate device with given configuration for application setup()
         *
         * @param config Configuration for general tracker operation
         * @retval SYSTEM_ERROR_NONE
         */
        int init(const EdgeConfiguration& config) {
            _deviceConfig = config;
            return init();
        }

        /**
         * @brief Initializate device with given configuration for application setup()
         *
         * @param config Configuration for general tracker operation
         * @retval SYSTEM_ERROR_NONE
         */
        int init(IEdgePlatformConfiguration *pConfig) {
            CHECK_TRUE((pConfig != nullptr), SYSTEM_ERROR_INVALID_ARGUMENT);
            _platformConfig = pConfig;
            _commonCfgData = _platformConfig->get_common_config_data();
            return init();
        }

        /**
         * @brief Perform device functionality for application loop()
         *
         */
        void loop();

        /**
         * @brief Stop services on device
         *
         * @retval SYSTEM_ERROR_NONE
         */
        int stop();

        /**
         * @brief Prepare tracker IO and peripherals for shutdown
         *
         * @retval SYSTEM_ERROR_NONE
         */
        int end();

        /**
         * @brief Prepare tracker for reset and issue
         *
         * @return SYSTEM_ERROR_NONE
         */
        int reset();

        /**
         * @brief Get the tracker hardware model number
         *
         * @return uint32_t Model number
         */
        uint32_t getModel() const {
            return _model;
        }

        /**
         * @brief Get the tracker hardware variant number
         *
         * @return uint32_t Variant number
         */
        uint32_t getVariant() const {
            return _variant;
        }

        /**
         * @brief Set the GNSS fast lock
         *
         * @param enable Enable faster GNSS lock
         */
        void setFastLock(bool enable) {
            _deviceConfig.enableFastLock(enable);
            locationService.setFastLock(enable);
        }

        /**
         * @brief Get the GNSS fast lock
         *
         * @return true Faster GNSS lock is enabled
         * @return false Faster GNSS lock is disabled
         */
        bool getFastLock() const {
            return locationService.getFastLock();;
        }

        /**
         * @brief Manually force off battery charging
         *
         * @param value true charging will be disabled (not re-enabled by Tracker Edge)
         * @param value false charging will revert to original logic (handled by Tracker Edge)
         */
        void forceDisableCharging(bool value);

        /**
         * @brief Force battery charge current
         *
         * @param current Current in milliampheres
         * @retval SYSTEM_ERROR_NONE
         */
        int setChargeCurrent(uint16_t current);

        /**
         * @brief Enable or disable IO/CAN power
         *
         * @param enable Enable IO/CAN power when true
         */
        void enableIoCanPower(bool enable);

        /**
         * @brief Indicates whether device can accept commands through USB interface
         *
         * @return true Commands are accepted via USB
         * @return false Commands are not accepted via USB
         */
        bool isUsbCommandEnabled() const {
            return _cloudConfig.UsbCommandEnable;
        }

        /**
         * @brief Enable or disable application watchdog
         *
         * @param enable
         */
        void enableWatchdog(bool enable);

        /**
         * @brief Invoke shipping mode
         *
         */
        void startShippingMode();

        /**
         * @brief Start preparing for sleep
         *
         */
        int prepareSleep();

        /**
         * @brief Exit sleep
         *
         */
        int prepareWake();

        /**
         * @brief Callback to collect Memfault metrics for heartbeat publishes
         *
         */
        void collectMemfaultHeartbeatMetrics();

        // underlying services exposed to allow sharing with rest of the system
        CloudService &cloudService;
        ConfigService &configService;
        EdgeSleep &sleep;
        EdgeGnssAbstraction &locationService;
        EdgeMotion &motionService;
        EdgeLocation &location;
        EdgeMotionConfiguration &motion;
        EdgeShipping &shipping;

    private:
        Edge();

        int chargeCallback(TemperatureChargeEvent event);

        static Edge* _instance;
    #ifdef EDGE_USE_MEMFAULT
        Memfault *_memfault {nullptr};
    #endif // EDGE_USE_MEMFAULT
        EdgeCloudConfig _cloudConfig;
        EdgeConfiguration _deviceConfig;
        IEdgePlatformConfiguration *_platformConfig {nullptr};
        EdgePlatformCommonConfiguration _commonCfgData;

        uint32_t _model;
        uint32_t _variant;

        uint32_t _lastLoopSec;
        bool _canPowerEnabled;
        bool _pastWarnLimit;
        unsigned int _evalTick;
        bool _lastBatteryCharging;
        bool _delayedBatteryCheck;
        unsigned int _delayedBatteryCheckTick;
        EdgeChargeStatus _pendingChargeStatus;
        Mutex _pendingLock;
        EdgeChargeState _chargeStatus;
        unsigned int _lowBatteryEvent;
        unsigned int _evalChargingTick;
        bool _batterySafeToCharge;
        bool _forceDisableCharging;
        bool _deviceMonitoring {false};

        // Startup and initialization related
        static int getPowerManagementConfig(hal_power_config& conf);
        static int setPowerManagementConfig(const hal_power_config& conf);
        static int enablePowerManagement();
        int initEsp32();
        int initCan();
        int initIo();

        // Sleep related
        void onSleepPrepare(EdgeSleepContext context);
        void onSleep(EdgeSleepContext context);
        void onWake(EdgeSleepContext context);
        void onSleepStateChange(EdgeSleepContext context);

        // Shutdown related
        void startLowBatteryShippingMode();

        // Various methods
        int registerConfig();
        static void loc_gen_cb(JSONWriter& writer, LocationPoint &loc, const void *context);
        EdgeChargeState batteryDecode(battery_state_t state);
        void setPendingChargeStatus(unsigned int uptime, EdgeChargeState state);
        EdgeChargeStatus getPendingChargeStatus();
        static void lowBatteryHandler(system_event_t event, int data);
        static void batteryStateHandler(system_event_t event, int data);
        void initBatteryMonitor();
        bool getChargeEnabled();
        void evaluateBatteryCharge();
        int pmicEnableCharging();
        int pmicDisableCharging();

        /**
         * @brief Handle OTA events from System.on() interface
         *
         * @param event Event class
         * @param data Particular event
         */
        void otaHandler(system_event_t event, int data);
};

// These are here for compatibility
using Tracker = Edge;
using TrackerCloudConfig = EdgeCloudConfig;
using TrackerChargeState = EdgeChargeState;
using TrackerChargeStatus = EdgeChargeStatus;
using TrackerConfiguration = EdgeConfiguration;
