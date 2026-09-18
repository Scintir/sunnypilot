#include <cstdlib>

#include "selfdrive/pandad/pandad.h"
#include "openpilot/cereal/messaging/messaging.h"
#include "common/swaglog.h"

// mirrors HYUNDAI_PARAM_SP_BMS_UDS in opendbc/safety/modes/hyundai_common.h
static const uint16_t HYUNDAI_PARAM_SP_BMS_UDS = 16U;

void PandaSafety::configureSafetyMode(bool is_onroad) {
  if (is_onroad && !safety_configured_) {
    updateMultiplexingMode();

    // sunnypilot: CAN discovery mode keeps the panda passive (ELM327) for the whole drive so every
    // frame on all three buses is captured to the rlog without the car safety model, relay intercept,
    // or any TX. Mode 2 additionally routes bus 1 to the OBD-II port. openpilot cannot engage in this mode.
    if (can_discovery_mode_ != 0) {
      if (!discovery_logged_) {
        LOGW("CanDiscoveryMode=%d: staying in ELM327 (bus 1 -> %s), car safety model will not be set",
             can_discovery_mode_, can_discovery_mode_ == 2 ? "OBD-II port" : "harness CAN2");
        discovery_logged_ = true;
      }
      return;
    }

    auto car_params = fetchCarParams();
    if (!car_params.empty()) {
      LOGW("got %lu bytes CarParams", car_params[0].size());
      LOGW("got %lu bytes CarParamsSP", car_params[1].size());
      setSafetyMode(car_params);
      safety_configured_ = true;
    }
  } else if (!is_onroad) {
    initialized_ = false;
    safety_configured_ = false;
    log_once_ = false;
    discovery_logged_ = false;
  }
}

void PandaSafety::updateMultiplexingMode() {
  // Initialize to ELM327 without OBD multiplexing for initial fingerprinting
  if (!initialized_) {
    prev_obd_multiplexing_ = false;
    // sunnypilot: latch the discovery mode once per onroad transition
    const std::string mode_str = params_.get("CanDiscoveryMode");
    can_discovery_mode_ = mode_str.empty() ? 0 : std::atoi(mode_str.c_str());
    // discovery mode 2 pins bus 1 to the OBD-II port for the whole drive
    const bool force_obd = can_discovery_mode_ == 2;
    panda_->set_safety_model(cereal::CarParams::SafetyModel::ELM327, force_obd ? 0U : 1U);
    initialized_ = true;
  }

  // Switch between multiplexing modes based on the OBD multiplexing request
  bool obd_multiplexing_requested = params_.getBool("ObdMultiplexingEnabled");
  if (obd_multiplexing_requested != prev_obd_multiplexing_) {
    // in discovery mode 2 the OBD mux stays on regardless of card's request; still ack so card doesn't block
    if (can_discovery_mode_ != 2) {
      const uint16_t safety_param = obd_multiplexing_requested ? 0U : 1U;
      panda_->set_safety_model(cereal::CarParams::SafetyModel::ELM327, safety_param);
    }
    prev_obd_multiplexing_ = obd_multiplexing_requested;
    params_.putBool("ObdMultiplexingChanged", true);
  }
}

// TODO-SP: Use structs instead of vector
std::vector<std::string> PandaSafety::fetchCarParams() {
  if (!params_.getBool("FirmwareQueryDone")) {
    return {};
  }

  if (!log_once_) {
    LOGW("Finished FW query, Waiting for params to set safety model");
    log_once_ = true;
  }

  if (!params_.getBool("ControlsReady")) {
    return {};
  }
  return {params_.get("CarParams"), params_.get("CarParamsSP")};
}

// TODO-SP: Use structs instead of vector
void PandaSafety::setSafetyMode(const std::vector<std::string> &params_string) {
  AlignedBuffer aligned_buf;
  AlignedBuffer aligned_buf_sp;

  capnp::FlatArrayMessageReader cmsg(aligned_buf.align(params_string[0].data(), params_string[0].size()));
  cereal::CarParams::Reader car_params = cmsg.getRoot<cereal::CarParams>();

  capnp::FlatArrayMessageReader cmsg_sp(aligned_buf_sp.align(params_string[1].data(), params_string[1].size()));
  cereal::CarParamsSP::Reader car_params_sp = cmsg_sp.getRoot<cereal::CarParamsSP>();

  auto safety_configs = car_params.getSafetyConfigs();
  uint16_t alternative_experience = car_params.getAlternativeExperience();
  uint16_t safety_param_sp = car_params_sp.getSafetyParam();

  cereal::CarParams::SafetyModel safety_model = safety_configs[0].getSafetyModel();
  uint16_t safety_param = safety_configs[0].getSafetyParam();

  LOGW("setting safety model: %d, param: %d, alternative experience: %d, param_sp: %d", (int)safety_model, safety_param, alternative_experience, safety_param_sp);
  panda_->set_alternative_experience(alternative_experience, safety_param_sp);
  panda_->set_safety_model(safety_model, safety_param);

  // sunnypilot: BMS UDS polling needs bus 1 on the OBD-II port. Setting the car safety model always returns
  // bus 1 to the harness CAN2 pair, so re-select the OBD mux afterwards. Only Hyundai CAN platforms set this bit
  // (see HYUNDAI_PARAM_SP_BMS_UDS in opendbc); on those harnesses the CAN2 pair carries nothing.
  const bool bms_uds = (safety_model == cereal::CarParams::SafetyModel::HYUNDAI ||
                        safety_model == cereal::CarParams::SafetyModel::HYUNDAI_LEGACY) &&
                       (safety_param_sp & HYUNDAI_PARAM_SP_BMS_UDS);
  const std::string bms_bus = params_.get("EvBmsUdsBus");
  if (bms_uds && bms_bus == "1") {
    LOGW("EvBmsUdsPolling: routing bus 1 to the OBD-II port");
    panda_->set_obd(true);
    // The OBD mux command only re-routes the pins; the ELM327 path in the firmware follows it with a CAN core
    // re-init (can_init_all). Without that, bus 1 came up dead after the switch on a comma four (no TX echoes,
    // RX count frozen for the whole drive). Setting the bus speed is the host-side way to re-init one bus.
    panda_->set_can_speed_kbps(1, 500);
  }
}

bool PandaSafety::getOffroadMode() {
  auto offroad_mode = params_.getBool("OffroadMode");
  return offroad_mode;
}
