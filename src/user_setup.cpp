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

#include "Particle.h"
#include "tracker_config.h"
#include "monitor_edge_ioexpansion.h"
#include "edge_location.h"
#include "edge_cellular.h"
#include "Adp8866GnssLed.h"
#include "DebounceSwitchRK.h"
#include "edge.h"
#include "StatisticCollector.h"
#include "ThresholdComparator.h"
#include "eeprom_helper.h"


//
// Constants
//
// !!!TODO!!!
// * Consider moving constants into header file
static constexpr uint8_t EEPROM_DATA_ADDRESS        {0x50};

static constexpr int32_t THRESHOLD_LOW_PERCENT      {50L};
static constexpr int32_t THRESHOLD_HIGH_PERCENT     {80L};
static constexpr uint32_t LED_DISPLAY_PERIOD_MS     {10000UL};
static constexpr uint8_t COLOR_MAX_VAL              {255};

static constexpr double VOLTAGE_IN_LOW              {0.0};
static constexpr double VOLTAGE_IN_LOW_BITS         {0};
static constexpr double VOLTAGE_IN_HIGH             {10.0};
static constexpr double VOLTAGE_IN_HIGH_BITS        {4095};
static constexpr double VOLTAGE_IN_FULL_SCALE       {3.3 * (10000.0 + 4700.0) / 4700.0};  // Full scale is Vf = Vdd * (R10 + R11) / R10
static constexpr double VOLTAGE_IN_CUTOFF_LOW       {0.001};  // Minimum frequency for the low pass filter
static constexpr double VOLTAGE_IN_CUTOFF_HIGH      {50.0};  // Maximum frequency for the low pass filter
static constexpr double VOLTAGE_IN_THRESH_LOW       {2.0};  // Low threshold for voltage input
static constexpr double VOLTAGE_IN_HYST_LOW         {1.0};  // Hysteresis for the low threshold
static constexpr double VOLTAGE_IN_THRESH_HIGH      {8.0};  // High threshold for voltage input
static constexpr double VOLTAGE_IN_HYST_HIGH        {1.0};  // Hysteresis for the high threshold

static constexpr double CURRENT_IN_LOW              {0.004};
static constexpr double CURRENT_IN_LOW_BITS         {0};
static constexpr double CURRENT_IN_HIGH             {0.020};
static constexpr double CURRENT_IN_HIGH_BITS        {4095};
static constexpr double CURRENT_IN_FULL_SCALE       {3.3 / 100.0};  // Full scale is Vf = Vdd / R2
static constexpr double CURRENT_IN_CUTOFF_LOW       {0.001};  // Minimum frequency for the low pass filter
static constexpr double CURRENT_IN_CUTOFF_HIGH      {50.0};  // Maximum frequency for the low pass filter
static constexpr double CURRENT_IN_FAULT_HYST_LOW   {0.000125};  // Hysteresis for the low threshold
static constexpr double CURRENT_IN_FAULT_TH_LOW     {CURRENT_IN_LOW - CURRENT_IN_FAULT_HYST_LOW};  // Low threshold for fault detection on the raw current input
static constexpr double CURRENT_IN_FAULT_HYST_HIGH  {0.000875};  // Hysteresis for the high threshold
static constexpr double CURRENT_IN_FAULT_TH_HIGH    {CURRENT_IN_HIGH + CURRENT_IN_FAULT_HYST_HIGH};  // High threshold for fault detection on the raw current input
static constexpr double CURRENT_IN_THRESH_LOW       {0.008};  // Low threshold for the scaled current input
static constexpr double CURRENT_IN_HYST_LOW         {0.002};  // Hysteresis for the low threshold
static constexpr double CURRENT_IN_THRESH_HIGH      {0.016};  // High threshold for the scaled current input
static constexpr double CURRENT_IN_HYST_HIGH        {0.002};  // Hysteresis for the high threshold

static constexpr double ANALOG_SAMPLE_MS            {10}; // 100Hz
static constexpr double ANALOG_SAMPLE_S             {ANALOG_SAMPLE_MS / 1000.0};

enum class HvInputEdgeType {
    None,
    Rising,
    Falling,
    Both
};

//
// Global variables
//
static Logger           monitorOneLog("MonitorOne");
static CellularSignal   Sig;
static Timer            *RestoreTmr;

// Particle cloud variables
static double VoltageInValue {};
static double CurrentInValue {};
static bool DigitalInValue {};
static ThresholdState VoltageInLowThState {};
static ThresholdState VoltageInHighThState {};
static ThresholdState CurrentInLowThState {};
static ThresholdState CurrentInHighThState {};
static bool CurrentInFaultLowThState {};
static bool CurrentInFaultHighThState {};


// Configuration settings
static double voltageCalGain {1.0};
static double voltageCalOffset {0.0};
static double voltageSensorLow {VOLTAGE_IN_LOW};
static double voltageSensorHigh {VOLTAGE_IN_HIGH};
static double voltageFilterFc {1.0}; // Hertz
static StatisticCollector<float> voltageIn(0.061, true); // For Fc=1Hz
static ThresholdComparator<float> voltageLow(VOLTAGE_IN_THRESH_LOW);
static bool voltageThresholdLowEnable {false};
static double voltageThresholdLow {VOLTAGE_IN_THRESH_LOW};
static double voltageHysteresisLow {VOLTAGE_IN_HYST_LOW};
static ThresholdComparator<float> voltageHigh(VOLTAGE_IN_THRESH_HIGH);
static bool voltageThresholdHighEnable {false};
static double voltageThresholdHigh {VOLTAGE_IN_THRESH_HIGH};
static double voltageHysteresisHigh {VOLTAGE_IN_HYST_HIGH};

static double currentCalGain {1.0};
static double currentCalOffset {0.0};
static double currentSensorLow {CURRENT_IN_LOW};
static double currentSensorHigh {CURRENT_IN_HIGH};
static double currentFilterFc {1.0}; // Hertz
static StatisticCollector<float> currentIn(0.061, true); // For Fc=1Hz
static ThresholdComparator<float> currentFaultLow(CURRENT_IN_FAULT_TH_LOW);
static bool currentFaultLowEnable {false};
static double currentFaultThresholdLow {CURRENT_IN_FAULT_TH_LOW};
static double currentFaultHysteresisLow {CURRENT_IN_FAULT_HYST_LOW};
static ThresholdComparator<float> currentFaultHigh(CURRENT_IN_FAULT_TH_HIGH);
static bool currentFaultHighEnable {false};
static double currentFaultThresholdHigh {CURRENT_IN_FAULT_TH_HIGH};
static double currentFaultHysteresisHigh {CURRENT_IN_FAULT_HYST_HIGH};
static ThresholdComparator<float> currentLow(CURRENT_IN_THRESH_LOW);
static bool currentLowEnable {false};
static double currentThresholdLow {CURRENT_IN_THRESH_LOW};
static double currentHysteresisLow {CURRENT_IN_HYST_LOW};
static ThresholdComparator<float> currentHigh(CURRENT_IN_THRESH_HIGH);
static bool currentHighEnable {false};
static double currentThresholdHigh {CURRENT_IN_THRESH_HIGH};
static double currentHysteresisHigh {CURRENT_IN_HYST_HIGH};

static bool inputPublishNow {false};
static HvInputEdgeType inputEdgeType {HvInputEdgeType::None};
static bool inputStateLast {false};


/**
 * @brief Restore default system LED behaviour
 *
 * @return None
 */
static void defaultLedBehaviour()
{
    // Restore the LED control back to the system
    RGB.control(false);
}


/**
 * @brief Handle short button press on Monitor One
 *
 * @details Asserts the correct LED pattern associated with cellular signal strength
 *          when the button is pressed for a short period
 *
 * @param[in] switchState state of button press (short, long, release, etc)
 * @param[in] context user data
 *
 * @return None
 */
static void buttonHandler(DebounceSwitchState* switchState, void *context)
{
    // See https://github.com/rickkas7/DebounceSwitchRK for more information
    // Some example DebouncePressState states:
    //           Single tap (< 3s):  PRESS_START -> SHORT -> RELEASED -> TAP
    //           Double tap (< 3s):  PRESS_START -> SHORT -> RELEASED -> PRESS_START -> SHORT -> RELEASED -> TAP
    //    Long press (> 3s, < 10s):  PRESS_START -> PROGRESS -> LONG -> RELEASED
    //    Very long press  (> 10s):  PRESS_START -> PROGRESS -> VERY_LONG -> RELEASED
    if( DebouncePressState::SHORT == switchState->getPressState() )
    {
        // Use EdgeCellular service to get the cellular signal strength
        if(!EdgeCellular::instance().getSignal(Sig))
        {
            auto pct = static_cast<int32_t>(Sig.getStrength());
            monitorOneLog.trace("Cell Strength = %ld", pct);

            // Take control of the system LED
            RGB.control(true);

            // Display the appropriate colour based on the signal strength
            if( (pct >= 0) && (pct < THRESHOLD_LOW_PERCENT) )
            {
                // Display RED for 10s
                RGB.color(COLOR_MAX_VAL, 0, 0);                 // [R, G, B]
                RestoreTmr->start();
            }
            else if( (pct >= THRESHOLD_LOW_PERCENT) && (pct < THRESHOLD_HIGH_PERCENT) )
            {
                // Display YELLOW for 10s
                RGB.color(COLOR_MAX_VAL, COLOR_MAX_VAL, 0);     // [R, G, B]
                RestoreTmr->start();
            }
            else if( pct >= THRESHOLD_HIGH_PERCENT )
            {
                // Display GREEN for 10s
                RGB.color(0, COLOR_MAX_VAL, 0);                 // [R, G, B]
                RestoreTmr->start();
            }
            else
            {
                // Shouldn't reach here. But, just in case, return control of the system LED
                RGB.control(false);
            }
        }
        else
        {
            monitorOneLog.trace("Cell strength not available");
        }
    }
}

/**
 * @brief Common setup function for Monitor One
 *
 * @details Configure common Monitor One IO and peripherals
 *
 */
static int commonMonitorOneSetup()
{
    // Set up a timer to restore LED behaviour after a button press
    RestoreTmr = new Timer(LED_DISPLAY_PERIOD_MS, defaultLedBehaviour, true);

    // Associate user button with debounce handler
    DebounceSwitch::getInstance()->setup();
    DebounceSwitch::getInstance()->addSwitch(MONITORONE_USER_BUTTON, DebounceSwitchStyle::PRESS_LOW_PULLUP, buttonHandler);

    // We want the user button to wake us up
    EdgeSleep::instance().wakeFor(MONITORONE_USER_BUTTON, FALLING);

    return SYSTEM_ERROR_NONE;
}

/**
 * @brief Timer callback to collect and average ADC values.
 *
 */
static void readAnalogInputs() {
    // Perform averaging of the raw ADC values
    auto rawVoltage = analogRead(MONITOREDGE_IOEX_VOLTAGE_IN_PIN);
    voltageIn.pushValue((float)rawVoltage);
    auto rawCurrent = analogRead(MONITOREDGE_IOEX_CURRENT_IN_PIN);
    currentIn.pushValue((float)rawCurrent);
}

/**
 * @brief Helper function to decode thresholds
 *
 * @param state Enumerated value for threshold state
 * @return String Corresponding string for given encoded state.
 */
static String readThresholdState(ThresholdState state) {
    switch (state) {
        case ThresholdState::AboveThreshold:
            return "above";
        case ThresholdState::BelowThreshold:
            return "below";
    }
    return "unknown";
}

static Timer sampleTimer(ANALOG_SAMPLE_MS, readAnalogInputs);

/**
 * @brief Setup the Monitor One IO expansion card
 *
 * @return int
 */
static int expanderIO()
{
    // Setup all of the IO pins here.  Some are digital outputs and others are analog inputs
    pinMode(MONITOREDGE_IOEX_VOLTAGE_IN_PIN, INPUT);
    pinMode(MONITOREDGE_IOEX_CURRENT_IN_PIN, INPUT);
    pinMode(MONITOREDGE_IOEX_DIGITAL_IN_PIN, INPUT);
    pinMode(MONITOREDGE_IOEX_RELAY_OUT_PIN, OUTPUT);
    digitalWrite(MONITOREDGE_IOEX_RELAY_OUT_PIN, LOW);

    pinMode(MONITOREDGE_IOEX_RS485_DE_PIN, OUTPUT);
    digitalWrite(MONITOREDGE_IOEX_RS485_DE_PIN, LOW);

    Particle.function("Relay", [](String val){
        auto trueMatch = (0 == strncasecmp("true", val.c_str(), sizeof("true")));
        auto falseMatch = (0 == strncasecmp("false", val.c_str(), sizeof("false")));
        uint8_t relay = LOW;
        if (trueMatch != falseMatch) {
            relay = (trueMatch && !falseMatch) ? HIGH : LOW;
        }
        else {
            relay = (0 == val.toInt()) ? LOW : HIGH;
        }
        digitalWrite(MONITOREDGE_IOEX_RELAY_OUT_PIN, relay);

        return 0;
    }, nullptr);

    Particle.variable("Voltage In", VoltageInValue);
    Particle.variable("Current In", CurrentInValue);
    Particle.variable("Digital In", DigitalInValue);
    Particle.variable("Voltage Low Th", []{ return readThresholdState(VoltageInLowThState); });
    Particle.variable("Voltage High Th", []{ return readThresholdState(VoltageInHighThState); });
    Particle.variable("Current Low Th", []{ return readThresholdState(CurrentInLowThState); });
    Particle.variable("Current High Th", []{ return readThresholdState(CurrentInHighThState); });
    Particle.variable("Current Low Fault", CurrentInFaultLowThState);
    Particle.variable("Current High Fault", CurrentInFaultHighThState);

    static ConfigObject ioCalibrationConfiguration("iocal", {
        ConfigObject("voltage", {
            ConfigFloat("calgain", &voltageCalGain),
            ConfigFloat("caloffset", &voltageCalOffset)
        }),
        ConfigObject("current", {
            ConfigFloat("calgain", &currentCalGain),
            ConfigFloat("caloffset", &currentCalOffset)
        })
    });
    ConfigService::instance().registerModule(ioCalibrationConfiguration);

    static ConfigObject ioConfiguration("io", {
        ConfigObject("voltage", {
            ConfigFloat("sensorlow", &voltageSensorLow),
            ConfigFloat("sensorhigh", &voltageSensorHigh),
            ConfigFloat("sensorfc",
                config_get_float_cb,
                [](double value, const void *context) {
                    voltageFilterFc = value;
                    voltageIn.setAverageAlpha((float)StatisticCollector<double>::frequencyToAlpha(ANALOG_SAMPLE_S, value));
                    return 0;
                },
                &voltageFilterFc,
                nullptr,
                VOLTAGE_IN_CUTOFF_LOW,
                VOLTAGE_IN_CUTOFF_HIGH
            ),
            ConfigFloat("threshlow",
                config_get_float_cb, [](double value, const void *context){voltageThresholdLow = value; voltageLow.setThreshold((float)value); return 0;},
                &voltageThresholdLow, nullptr, NAN, NAN),
            ConfigFloat("hystlow",
                config_get_float_cb, [](double value, const void *context){voltageHysteresisLow = value; voltageLow.setHysteresis((float)value); return 0;},
                &voltageHysteresisLow, nullptr, 0.0, NAN),
            ConfigBool("th_low_en", &voltageThresholdLowEnable),
            ConfigFloat("threshhigh",
                config_get_float_cb, [](double value, const void *context){voltageThresholdHigh = value; voltageHigh.setThreshold((float)value); return 0;},
                &voltageThresholdHigh, nullptr, NAN, NAN),
            ConfigFloat("hysthigh",
                config_get_float_cb, [](double value, const void *context){voltageHysteresisHigh = value; voltageHigh.setHysteresis((float)value); return 0;},
                &voltageHysteresisHigh, nullptr, 0.0, NAN),
            ConfigBool("th_high_en", &voltageThresholdHighEnable),
        }),
        ConfigObject("current", {
            ConfigFloat("sensorlow", &currentSensorLow),
            ConfigFloat("sensorhigh", &currentSensorHigh),
            ConfigFloat("sensorfc",
                config_get_float_cb,
                [](double value, const void *context) {
                    currentFilterFc = value;
                    currentIn.setAverageAlpha((float)StatisticCollector<double>::frequencyToAlpha(ANALOG_SAMPLE_S, value));
                    return 0;
                },
                &currentFilterFc,
                nullptr,
                CURRENT_IN_CUTOFF_LOW,
                CURRENT_IN_CUTOFF_HIGH
            ),
            ConfigFloat("threshlow",
                config_get_float_cb, [](double value, const void *context){currentThresholdLow = value; currentLow.setThreshold((float)value); return 0;},
                &currentThresholdLow, nullptr, NAN, NAN),
            ConfigFloat("hystlow",
                config_get_float_cb, [](double value, const void *context){currentHysteresisLow = value; currentLow.setHysteresis((float)value); return 0;},
                &currentHysteresisLow, nullptr, 0.0, NAN),
            ConfigBool("th_low_en", &currentLowEnable),
            ConfigFloat("threshhigh",
                config_get_float_cb, [](double value, const void *context){currentThresholdHigh = value; currentHigh.setThreshold((float)value); return 0;},
                &currentThresholdHigh, nullptr, NAN, NAN),
            ConfigFloat("hysthigh",
                config_get_float_cb, [](double value, const void *context){currentHysteresisHigh = value; currentHigh.setHysteresis((float)value); return 0;},
                &currentHysteresisHigh, nullptr, 0.0, NAN),
            ConfigBool("th_high_en", &currentHighEnable),
            ConfigFloat("th_fault_low",
                config_get_float_cb, [](double value, const void *context){currentFaultThresholdLow = value; currentFaultLow.setThreshold((float)value); return 0;},
                &currentFaultThresholdLow, nullptr, 0.0, 0.030),
            ConfigFloat("hyst_fault_low",
                config_get_float_cb, [](double value, const void *context){currentFaultHysteresisLow = value; currentFaultLow.setHysteresis((float)value); return 0;},
                &currentFaultHysteresisLow, nullptr, 0.0, CURRENT_IN_HIGH),
            ConfigBool("th_fault_low_en", &currentFaultLowEnable),
            ConfigFloat("th_fault_high",
                config_get_float_cb, [](double value, const void *context){currentFaultThresholdHigh = value; currentFaultHigh.setThreshold((float)value); return 0;},
                &currentFaultThresholdHigh, nullptr, 0.0, 0.030),
            ConfigFloat("hyst_fault_high",
                config_get_float_cb, [](double value, const void *context){currentFaultHysteresisHigh = value; currentFaultHigh.setHysteresis((float)value); return 0;},
                &currentFaultHysteresisHigh, nullptr, 0.0, CURRENT_IN_HIGH),
            ConfigBool("th_fault_high_en", &currentFaultHighEnable),
        }),
        ConfigObject("input", {
            ConfigBool("immediate", &inputPublishNow),
            ConfigStringEnum("edge", {
                {"none", (int32_t) HvInputEdgeType::None},
                {"rising", (int32_t) HvInputEdgeType::Rising},
                {"falling", (int32_t) HvInputEdgeType::Falling},
                {"both", (int32_t) HvInputEdgeType::Both}
            }, &inputEdgeType)
        })
    });
    ConfigService::instance().registerModule(ioConfiguration);

    voltageLow.setCallback([](float value, ThresholdState state) {
        if (voltageThresholdLowEnable && (ThresholdState::BelowThreshold == state)) {
            EdgeLocation::instance().triggerLocPub(Trigger::IMMEDIATE, "io_vlow");
        }
    });
    voltageHigh.setCallback([](float value, ThresholdState state) {
        if (voltageThresholdHighEnable && (ThresholdState::AboveThreshold == state)) {
            EdgeLocation::instance().triggerLocPub(Trigger::IMMEDIATE, "io_vhigh");
        }
    });
    currentFaultLow.setCallback([](float value, ThresholdState state) {
        if (currentFaultLowEnable) {
            auto event = (ThresholdState::BelowThreshold == state) ? "io_afltlow_raise" : "io_afltlow_clr";
            EdgeLocation::instance().triggerLocPub(Trigger::IMMEDIATE, event);
        }
    });
    currentFaultHigh.setCallback([](float value, ThresholdState state) {
        if (currentFaultHighEnable) {
            auto event = (ThresholdState::AboveThreshold == state) ? "io_aflthigh_raise" : "io_aflthigh_clr";
            EdgeLocation::instance().triggerLocPub(Trigger::IMMEDIATE, event);
        }
    });
    currentLow.setCallback([](float value, ThresholdState state) {
        if (currentLowEnable && (ThresholdState::BelowThreshold == state)) {
            EdgeLocation::instance().triggerLocPub(Trigger::IMMEDIATE, "io_alow");
        }
    });
    currentHigh.setCallback([](float value, ThresholdState state) {
        if (currentHighEnable && (ThresholdState::AboveThreshold == state)) {
            EdgeLocation::instance().triggerLocPub(Trigger::IMMEDIATE, "io_ahigh");
        }
    });

    EdgeLocation::instance().regLocGenCallback(
        [](JSONWriter& writer, LocationPoint& location, const void* nothing) {
            writer.name("io_v").value(VoltageInValue, 3);
            writer.name("io_a").value(CurrentInValue, 3);
            writer.name("io_in").value(DigitalInValue);
            writer.name("io_vlow").value((int)VoltageInLowThState);
            writer.name("io_vhigh").value((int)VoltageInHighThState);
            writer.name("io_alow").value((int)CurrentInLowThState);
            writer.name("io_ahigh").value((int)CurrentInHighThState);
            writer.name("io_afltlow").value(CurrentInFaultLowThState);
            writer.name("io_aflthigh").value(CurrentInFaultHighThState);
        }
    );

    sampleTimer.start();

    // The general purpose 24V input is inverted as it passes through an optoisolator.
    DigitalInValue = digitalRead(MONITOREDGE_IOEX_DIGITAL_IN_PIN) ? false : true;
    inputStateLast = DigitalInValue;

    DebounceSwitch::getInstance()->addSwitch(MONITOREDGE_IOEX_DIGITAL_IN_PIN, DebounceSwitchStyle::TOGGLE,
        [](DebounceSwitchState *switchState, void *) {
            switch (switchState->getPressState()) {
                case DebouncePressState::TOGGLE_LOW:
                    DigitalInValue = true; // 24V input is inverted as it passes through an optoisolator
                    break;

                case DebouncePressState::TOGGLE_HIGH:
                    DigitalInValue = false; // 24V input is inverted as it passes through an optoisolator
                    break;
            }

            auto inputEdgeEvent = false;
            switch (inputEdgeType) {
                case HvInputEdgeType::Rising:
                    inputEdgeEvent = (!inputStateLast && DigitalInValue);
                    break;

                case HvInputEdgeType::Falling:
                    inputEdgeEvent = (inputStateLast && !DigitalInValue);
                    break;

                case HvInputEdgeType::Both:
                    inputEdgeEvent = (inputStateLast != DigitalInValue);
                    break;
            }
            if (inputEdgeEvent) {
                EdgeLocation::instance().triggerLocPub((inputPublishNow) ? Trigger::IMMEDIATE : Trigger::NORMAL, "io_in");
            }
            inputStateLast = DigitalInValue;
        }
    );

    return SYSTEM_ERROR_NONE;
}

/**
 * @brief User setup function for Monitor One
 *
 * @details Set up a function to be invoked when the external user button is pressed
 *
 * @return Success or failure codes
 */
int user_init()
{
    CHECK(commonMonitorOneSetup());

    // !!!TODO!!!
    // * check EEPROM here
    // * Store SKU strings somewhere else
    // ExpansionEeprom eeprom {};
    // auto ret = readEepromBytes(Wire, EEPROM_DATA_ADDRESS, 0, (uint8_t*)&eeprom, sizeof(eeprom));
    // if (ret || !isEeepromValid(eeprom)) {
    //     monitorOneLog.error("This is not the card you are looking for");
    //     return SYSTEM_ERROR_NOT_SUPPORTED;
    // }
    // if (!strcmp(eeprom.sku, "EXP1_IO_BASIC_485CAN")) {
    //     monitorOneLog.info("Detected a basic IO expansion card with RS-485 and CAN bus");
        CHECK(expanderIO());
    // }
    // else if (!strcmp(eeprom.sku, "EXP1_PROTO")) {
    //     monitorOneLog.info("Detected a basic prototype expansion card");
    //     // Do proto card stuff here
    // }
    // else {
    //     return SYSTEM_ERROR_NOT_SUPPORTED;
    // }

    return SYSTEM_ERROR_NONE;
}

int user_loop()
{
    auto rawVoltage = map((double)voltageIn.getAverage(), VOLTAGE_IN_LOW_BITS, VOLTAGE_IN_HIGH_BITS, 0.0, VOLTAGE_IN_FULL_SCALE);
    auto calibratedVoltage = (rawVoltage + voltageCalOffset) * voltageCalGain;
    VoltageInValue = map(calibratedVoltage, VOLTAGE_IN_LOW, VOLTAGE_IN_HIGH, voltageSensorLow, voltageSensorHigh);
    VoltageInLowThState = voltageLow.evaluate((float)VoltageInValue);
    VoltageInHighThState = voltageHigh.evaluate((float)VoltageInValue);

    auto rawCurrent = map((double)currentIn.getAverage(), CURRENT_IN_LOW_BITS, CURRENT_IN_HIGH_BITS, 0.0, CURRENT_IN_FULL_SCALE);
    auto calibratedCurrent = (rawCurrent + currentCalOffset) * currentCalGain;
    CurrentInFaultLowThState = (currentFaultLow.evaluate((float)calibratedCurrent) == ThresholdState::BelowThreshold);
    CurrentInFaultHighThState = (currentFaultHigh.evaluate((float)calibratedCurrent) == ThresholdState::AboveThreshold);
    CurrentInValue = map(calibratedCurrent, CURRENT_IN_LOW, CURRENT_IN_HIGH, currentSensorLow, currentSensorHigh);
    CurrentInLowThState = currentLow.evaluate((float)CurrentInValue);
    CurrentInHighThState = currentHigh.evaluate((float)CurrentInValue);

    return SYSTEM_ERROR_NONE;
}
