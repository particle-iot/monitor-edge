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

#include "edge_sleep.h"
#include "cloud_service.h"
#include "edge_gnss_abstraction.h"
#include "edge.h"

// Private constants
constexpr system_tick_t TrackerSleepMinSleepDuration = 1000; // milliseconds

EdgeSleep *EdgeSleep::_instance = nullptr;

Logger sleepLog("app.sleep");

void EdgeSleep::handleOta(system_event_t event, int param) {
  EdgeSleep::instance().pauseSleep();
}

int EdgeSleep::enterReset() {
  auto deferredReset = new Timer(EdgeSleepResetTimerDelay, [this]() {
    EdgeGnssAbstraction::instance().stop();
    delay(1000);
    Edge::instance().reset();
  }, true);
  // Extend execution to wait for the eventual reset
  extendExecution(EdgeSleepResetTimerDelay / S2M(1) * 2);
  deferredReset->start();

  return SYSTEM_ERROR_NONE;
}

int EdgeSleep::handleReset(JSONValue *root) {
  _pendingReset = true;

  return 0;
}

int EdgeSleep::init(SleepWatchdogCallback watchdog) {
  static ConfigObject sleepDesc
  (
    "sleep",
    {
      ConfigStringEnum(
        "mode",
        {
          {"disable", (int32_t) EdgeSleepMode::Disable},
          {"enable", (int32_t) EdgeSleepMode::Enable},
        },
        &_config_state.mode
      ),
      ConfigInt("exe_min", &_config_state.execute_min_seconds, EdgeSleepDefaultExeMinTime, EdgeSleepDefaultMaxTime),
      ConfigInt("conn_max", &_config_state.connecting_max_seconds, EdgeSleepDefaultConnMaxTime, EdgeSleepDefaultMaxTime)
    }
  );

  _watchdog = watchdog;

  ConfigService::instance().registerModule(sleepDesc);

  // Associate OTA handler to pause sleep
  System.on(firmware_update+firmware_update_pending, handleOta);

  // Register callback to be alerted when there is a publish
  EdgeLocation::instance().regLocGenCallback([this](JSONWriter& writer, LocationPoint &loc, const void *context){annoucePublish();});

  // Register 'reset' command from the cloud
  CloudService::instance().registerCommand("reset", std::bind(&EdgeSleep::handleReset, this, std::placeholders::_1));

  return SYSTEM_ERROR_NONE;
}

EdgeSleepError EdgeSleep::updateNextWake(uint64_t milliseconds) {
  // A input value of 0 means that the requestor wants to cancel the current sleep cycle, pass through
  // the sleep state and re-enter the execution phase
  if (milliseconds == 0) {
    _nextWakeMs = 0;
    return EdgeSleepError::NONE;
  }

  // This function performs a basic priority scheduler calculation based on the next scheduled wake
  // time versus the requested value from the caller.  Any action for sleep evaluation is performed in
  // the future so the only comparison to present time would be the requested value.  The next wake
  // time is compared to the future present time later on.
  uint64_t now = System.millis();

  // Nothing from the past makes sense
  if (milliseconds <= now) {
    return EdgeSleepError::TIME_IN_PAST;
  }
  // Anything evaluated past this point assumes the requested wake time is in the future

  // We want to capture the very first wake request after waking or a subsequent time that is
  // sooner than one already established.
  if (_nextWakeMs == 0) {
    _nextWakeMs = milliseconds;
  }
  else {
    if (milliseconds > _nextWakeMs) {
      return EdgeSleepError::TIME_SKIPPED;
    }
    else {
      _nextWakeMs = milliseconds;
    }
  }

  return EdgeSleepError::NONE;
}

EdgeSleepError EdgeSleep::wakeAtSeconds(unsigned int uptimeSeconds) {
  return updateNextWake((uint64_t)uptimeSeconds * 1000);
}

EdgeSleepError EdgeSleep::wakeAtMilliseconds(system_tick_t milliseconds) {
  return updateNextWake((uint64_t)milliseconds);
}

EdgeSleepError EdgeSleep::wakeAtMilliseconds(uint64_t milliseconds) {
  return updateNextWake(milliseconds);
}

EdgeSleepError EdgeSleep::wakeAt(std::chrono::milliseconds ms) {
  return updateNextWake((uint64_t)ms.count());
}

int EdgeSleep::wakeFor(pin_t pin, InterruptMode mode) {
  // Search through existing wake pins and update mode if already existing
  for (auto item : _onPin) {
    if (item.first == pin) {
      item.second = mode;
      return SYSTEM_ERROR_NONE;
    }
  }

  _onPin.append(std::make_pair(pin, mode));
  return SYSTEM_ERROR_NONE;
}

int EdgeSleep::wakeFor(SystemSleepFlag flag) {
  _onFlag.append(flag);
  return SYSTEM_ERROR_NONE;
}

int EdgeSleep::wakeFor(network_interface_t netif) {
  if (netif != NETWORK_INTERFACE_CELLULAR) {
    return SYSTEM_ERROR_NOT_SUPPORTED;
  }

  _onNetwork = true;
  return SYSTEM_ERROR_NONE;
}

int EdgeSleep::wakeForBle() {
  _onBle = true;
  return SYSTEM_ERROR_NONE;
}

int EdgeSleep::ignore(pin_t pin) {
  int index = 0;

  // Search through list and remove (all) instances of this pin
  for (auto item : _onPin) {
    if (item.first == pin) {
      _onPin.removeAt(index);
    }
    index++;
  }

  return (index == _onPin.size()) ? SYSTEM_ERROR_NOT_FOUND : SYSTEM_ERROR_NONE;
}

int EdgeSleep::ignore(SystemSleepFlag flag) {
  if (_onFlag.removeAll(flag) == 0) {
    return SYSTEM_ERROR_NOT_FOUND;
  }

  return SYSTEM_ERROR_NONE;
}

int EdgeSleep::ignore(network_interface_t netif) {
  if (netif != NETWORK_INTERFACE_CELLULAR) {
    return SYSTEM_ERROR_NOT_SUPPORTED;
  }

  _onNetwork = false;
  return SYSTEM_ERROR_NONE;
}

int EdgeSleep::ignoreBle() {
  _onBle = false;
  return SYSTEM_ERROR_NONE;
}

int EdgeSleep::registerSleepPrepare(SleepCallback callback) {
  _onSleepPrepare.append(callback);
  return SYSTEM_ERROR_NONE;
}

int EdgeSleep::registerSleepCancel(SleepCallback callback) {
  _onSleepCancel.append(callback);
  return SYSTEM_ERROR_NONE;
}

int EdgeSleep::registerSleep(SleepCallback callback) {
  _onSleep.append(callback);
  return SYSTEM_ERROR_NONE;
}

int EdgeSleep::registerWake(SleepCallback callback) {
  _onWake.append(callback);
  return SYSTEM_ERROR_NONE;
}

int EdgeSleep::registerStateChange(SleepCallback callback) {
  _onStateTransition.append(callback);
  return SYSTEM_ERROR_NONE;
}

void EdgeSleep::startModem() {
  sleepLog.info("Starting modem");
  _lastModemOnMs = System.millis();
  Particle.connect();
  _inFullWakeup = true;
}

void EdgeSleep::stopModem() {
  sleepLog.info("Stopping modem");
  // Explicitly disconnect from the cloud with graceful offline status message
  Particle.disconnect(CloudDisconnectOptions().graceful(true).timeout(EdgeSleepGracefulTimeout));
  waitUntil(Particle.disconnected);
  Cellular.disconnect();
  waitUntilNot(Cellular.ready);
  _inFullWakeup = false;
}

EdgeSleepResult EdgeSleep::sleep() {
  EdgeSleepResult retval;

  SystemSleepConfiguration config;

  // Prepare to call all of the registered sleep prep callbacks with the same message
  EdgeSleepContext sleepContext = {
    .reason = EdgeSleepReason::PREPARE_SLEEP,
    .result = SystemSleepResult(),
    .loop = _loopCount,
    .lastSleepMs = _lastSleepMs,
    .lastWakeMs = _lastWakeMs,
    .nextWakeMs = _nextWakeMs,
    .modemOnMs = _lastModemOnMs,
  };

  // Full wakeup is requested only after this point
  _fullWakeupOverride = false;

  for (auto callback : _onSleepPrepare) {
    callback(sleepContext);
  }

  // We need to calculate the sleep duration based on the absolute uptime in milliseconds and how much time we need
  // to wake beforehand to power on the cellular modem and GNSS module.
  uint64_t now = System.millis();
  system_tick_t duration = (system_tick_t)(_nextWakeMs - now);

  // Don't sleep if too short of duration
  bool cancelSleep = false;
  if (_nextWakeMs == 0) {
    sleepLog.trace("cancelled sleep because of missing wake time");
    cancelSleep = true;
  }
  if (_nextWakeMs < now) {
    sleepLog.trace("cancelled sleep at %lu milliseconds because it is in the past", (uint32_t)_nextWakeMs);
    cancelSleep = true;
  }

  if (cancelSleep) {
    // It is not worth sleeping
    EdgeSleepContext sleepCancelContext = {
      .reason = EdgeSleepReason::CANCEL_SLEEP,
      .result = SystemSleepResult(),
      .loop = _loopCount,
      .lastSleepMs = _lastSleepMs,
      .lastWakeMs = _lastWakeMs,
      .nextWakeMs = _nextWakeMs,
      .modemOnMs = _lastModemOnMs,
    };

    for (auto callback : _onSleepCancel) {
      callback(sleepCancelContext);
    }

    // The next wake time is now invalid and should be treated uninitialized
    _nextWakeMs = 0;
    retval.error = EdgeSleepError::CANCELLED;
    return retval;
  }

  config.mode(SystemSleepMode::ULTRA_LOW_POWER)
    .gpio(PMIC_INT, FALLING)    // Always detect power events
    .gpio(LOW_BAT_UC, FALLING); // Keep fuel gauge awake

  // Accumulate all of the pin sources for wake
  for (auto pin : _onPin) {
    config.gpio(pin.first, pin.second);
  }

  if (_onNetwork) {
    config.network(NETWORK_INTERFACE_CELLULAR);
  } else {
    stopModem();
  }

  if (_onBle) {
    config.ble();
  }

  EdgeSleepContext sleepNowContext = {
    .reason = EdgeSleepReason::SLEEP,
    .result = SystemSleepResult(),
    .loop = _loopCount,
    .lastSleepMs = _lastSleepMs,
    .lastWakeMs = _lastWakeMs,
    .nextWakeMs = _nextWakeMs,
    .modemOnMs = _lastModemOnMs,
  };

  for (auto callback : _onSleep) {
    callback(sleepNowContext);
  }

  (void)Edge::instance().prepareSleep();

  // Disable watchdog
  if (_watchdog) {
    _watchdog(false);
  }

  // Perform the actual System sleep now
  // Capture time that sleep was entered
  _lastSleepMs = System.millis();

  // Re-evaluate the duration because handlers and preparation may have taken away time
  duration = (system_tick_t)(_nextWakeMs - _lastSleepMs);
  if (_lastSleepMs >= _nextWakeMs) {
    duration = TrackerSleepMinSleepDuration; // Sleep for at least 1 second
  }
  config.duration(duration);

  _lastRequestedWakeMs = _lastSleepMs + duration;
  sleepLog.info("sleeping until %lu milliseconds", (uint32_t)_lastRequestedWakeMs);

  retval.result = System.sleep(config);

  // Capture the wake time to help calculate the next sleep cycle
  _lastWakeMs = System.millis();

  _executeDurationSec = (uint32_t)_config_state.execute_min_seconds;

  // Enable watchdog
  if (_watchdog) {
    _watchdog(true);
  }

  (void)Edge::instance().prepareWake();

  // Our loop count restarts to indicate that we are executing out of sleep
  _loopCount = 0;
  _nextWakeMs = 0;
  _inFullWakeup = false;

  // Call all registered callbacks for wake and provide a common context
  EdgeSleepContext wakeContext = {
    .reason = EdgeSleepReason::WAKE,
    .result = retval.result,
    .loop = _loopCount,
    .lastSleepMs = _lastSleepMs,
    .lastWakeMs = _lastWakeMs,
    .nextWakeMs = _nextWakeMs,
    .modemOnMs = _lastModemOnMs,
  };

  for (auto callback : _onWake) {
    callback(wakeContext);
  }

  retval.error = EdgeSleepError::NONE;
  return retval;
}

void EdgeSleep::stateToConnecting() {
  _fullWakeupOverride = false;
  _executionState = EdgeExecutionState::CONNECTING;
  _lastConnectingSec = System.uptime();
  _publishFlag = false;

  startModem();

  EdgeSleepContext stateContext = {
    .reason = EdgeSleepReason::STATE_TO_CONNECTING,
    .result = SystemSleepResult(),
    .loop = _loopCount,
    .lastSleepMs = _lastSleepMs,
    .lastWakeMs = _lastWakeMs,
    .nextWakeMs = _nextWakeMs,
    .modemOnMs = _lastModemOnMs,
  };

  for (auto callback : _onStateTransition) {
    callback(stateContext);
  }
}

void EdgeSleep::stateToExecute() {
  _executionState = EdgeExecutionState::EXECUTION;
  _lastExecuteSec = System.uptime();

  EdgeSleepContext stateContext = {
    .reason = EdgeSleepReason::STATE_TO_EXECUTION,
    .result = SystemSleepResult(),
    .loop = _loopCount,
    .lastSleepMs = _lastSleepMs,
    .lastWakeMs = _lastWakeMs,
    .nextWakeMs = _nextWakeMs,
    .modemOnMs = _lastModemOnMs,
  };

  for (auto callback : _onStateTransition) {
    callback(stateContext);
  }
}

void EdgeSleep::stateToSleep() {
  _executionState = EdgeExecutionState::SLEEP;

  EdgeSleepContext stateContext = {
    .reason = EdgeSleepReason::STATE_TO_SLEEP,
    .result = SystemSleepResult(),
    .loop = _loopCount,
    .lastSleepMs = _lastSleepMs,
    .lastWakeMs = _lastWakeMs,
    .nextWakeMs = _nextWakeMs,
    .modemOnMs = _lastModemOnMs,
  };

  for (auto callback : _onStateTransition) {
    callback(stateContext);
  }
}

void EdgeSleep::stateToShutdown() {
  _executionState = EdgeExecutionState::SHUTDOWN;

  EdgeSleepContext stateContext = {
    .reason = EdgeSleepReason::STATE_TO_SHUTDOWN,
    .result = SystemSleepResult(),
    .loop = _loopCount,
    .lastSleepMs = _lastSleepMs,
    .lastWakeMs = _lastWakeMs,
    .nextWakeMs = 0,
    .modemOnMs = _lastModemOnMs,
  };

  for (auto callback : _onStateTransition) {
    callback(stateContext);
  }

  _lastShutdownMs = millis();
}

void EdgeSleep::stateToReset() {
  _executionState = EdgeExecutionState::RESET;

  EdgeSleepContext stateContext = {
    .reason = EdgeSleepReason::STATE_TO_RESET,
    .result = SystemSleepResult(),
    .loop = _loopCount,
    .lastSleepMs = _lastSleepMs,
    .lastWakeMs = _lastWakeMs,
    .nextWakeMs = 0,
    .modemOnMs = _lastModemOnMs,
  };

  for (auto callback : _onStateTransition) {
    callback(stateContext);
  }

  _lastResetMs = millis();
}

int EdgeSleep::loop() {

  // Perform state operations and transitions
  switch (_executionState) {
    /* ----------------------------------------------------------------------------------------------------------------
     * BOOT state
     * This state is only entered upon power on and can only transition to the CONNECTING state
     *-----------------------------------------------------------------------------------------------------------------
     */
    case EdgeExecutionState::BOOT: {
      _lastWakeMs = System.millis();
      _loopCount = 0;
      _executeDurationSec = (uint32_t)_config_state.execute_min_seconds;
      stateToConnecting();
      break;
    }


    /* ----------------------------------------------------------------------------------------------------------------
     * CONNECTING state
     * This state is entered from all other states and can only transition to the EXECUTION state.
     *
     * The purpose of this state is to wait for a Particle connection and publish from the location service.  If
     * that doesn't happen then transition immediately to the EXECUTE state based on a configurable timeout.  Waiting
     * for a valid connection state during poor cellular reception would otherwise cause the system to wait
     * indefinitely and run down battery charge.
     *-----------------------------------------------------------------------------------------------------------------
     */
    case EdgeExecutionState::CONNECTING: {
      if (_pendingPublishVitals && Particle.connected()) {
        _pendingPublishVitals = false;
        Particle.publishVitals();
      }
      if (_publishFlag && Particle.connected()) {
        _publishFlag = false;
        sleepLog.trace("published and transitioning to EXECUTE");
        stateToExecute();
      }
      else if (System.uptime() - _lastConnectingSec >= (uint32_t)_config_state.connecting_max_seconds) {
        EdgeLocation::instance().triggerLocPub(Trigger::IMMEDIATE, "imm");
        sleepLog.trace("publishing timed out and transitioning to EXECUTE");
        stateToExecute();
      }

      break;
    }


    /* ----------------------------------------------------------------------------------------------------------------
     * EXECUTION state
     * This state is entered from all other states and can only transition to the EXECUTION state.
     *
     * The purpose of this state is to wait for a Particle connection and publish from the location service.  If
     * that doesn't happen then transition immediately to the EXECUTE state based on a configurable timeout.  Waiting
     * for a valid connection state during poor cellular reception would otherwise cause the system to wait
     * indefinitely and run down battery charge.
     *-----------------------------------------------------------------------------------------------------------------
     */
    case EdgeExecutionState::EXECUTION: {
      // Execution depends on whether we are sleep enabled
      if (!isSleepDisabled()) {
        if (!_inFullWakeup && (_pendingPublishVitals || _pendingShutdown)) {
          _fullWakeupOverride = true;
        }

        // Check immediately if a full wake was requested and enter connecting state
        if (!_inFullWakeup && _fullWakeupOverride) {
          sleepLog.trace("full wakeup requested, connecting");
          stateToConnecting();
          break;
        }

        if (_pendingShutdown) {
          sleepLog.trace("EXECUTE time expired and transitioning to SHUTDOWN");
          stateToShutdown();
        }
        else if (_pendingReset) {
          sleepLog.trace("EXECUTE time expired and transitioning to RESET");
          stateToReset();
        }

        if (!_holdSleep &&
            (System.uptime() - _lastExecuteSec >= _executeDurationSec)) {

            sleepLog.trace("EXECUTE time expired and transitioning to SLEEP");
            stateToSleep();
        }
      }
      else {
        // Nothing specific to do
        _lastExecuteSec = System.uptime();

        if (_pendingPublishVitals && Particle.connected()) {
          _pendingPublishVitals = false;
          Particle.publishVitals();
        }

        if (_pendingShutdown) {
          stateToShutdown();
        }
        else if (_pendingReset) {
          stateToReset();
        }
      }

      break;
    }

    /* ----------------------------------------------------------------------------------------------------------------
     * SLEEP state
     * This state is only entered from the EXECUTE state and can only transition to EXECUTION and CONNECTING states.
     *
     * The purpose of this state is to enter sleep and decide what to do after waking.
     *-----------------------------------------------------------------------------------------------------------------
     */
    case EdgeExecutionState::SLEEP: {
      // Perform actual sleep here
      auto result = sleep();

      // There was a problem going to sleep so transition back to EXECUTE and re-evaluate
      if (result.error == EdgeSleepError::CANCELLED) {
        sleepLog.trace("cancelled and executing");
        stateToExecute();
      }
      else if (_fullWakeupOverride) {
        sleepLog.trace("woke and connecting");
        stateToConnecting();
      }
      else {
        sleepLog.trace("woke and executing without connection");
        stateToExecute();
      }
      break;
    }

    /* ----------------------------------------------------------------------------------------------------------------
     * SHUTDOWN state
     * This state is only entered from the EXECUTE state and can only transition to SHUTDOWN states.
     *
     * The purpose of this state is to start shipping mode and wait for it to happen.
     *-----------------------------------------------------------------------------------------------------------------
     */
    case EdgeExecutionState::SHUTDOWN: {
      if (_pendingPublishVitals && Particle.connected()) {
        _pendingPublishVitals = false;
        Particle.publishVitals();
      }
      if ((_publishFlag && Particle.connected()) ||
          (millis() - _lastShutdownMs >= EdgeSleepShutdownTimeout)) {
        // Stop everything
        stopModem();
        EdgeShipping::instance().enter(true);
        while (true) {}
      }
      break;
    }

    /* ----------------------------------------------------------------------------------------------------------------
     * RESET state
     * This state is only entered from the EXECUTE state and can only transition to RESET states.
     *
     * The purpose of this state is to start a system reset with graceful disconnect.
     *-----------------------------------------------------------------------------------------------------------------
     */
    case EdgeExecutionState::RESET: {
      if (_pendingPublishVitals && Particle.connected()) {
        _pendingPublishVitals = false;
        Particle.publishVitals();
      }
      if ((_publishFlag && Particle.connected()) ||
          (millis() - _lastResetMs >= EdgeSleepResetTimeout)) {
        enterReset();
        while (true) {}
      }
      break;
    }
  }

  _loopCount++;

  return SYSTEM_ERROR_NONE;
}
