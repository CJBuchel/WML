#pragma once

#include "sensors/Encoder.h"
#include "simulation/ctre_sim.h"

#include <mockdata/EncoderData.h>

#include <stdint.h>
#include <memory>
#include <algorithm>
#include <iostream>

namespace simulation {
namespace components {
  class encoder_sim {
   public:
    virtual void set_counts(int count) = 0;
    virtual void set_counts_per_sec(int cps) = 0;
  };

  class talonsrx_encoder : public encoder_sim {
   public:
    wml::TalonSrx *talon;

    talonsrx_encoder(wml::TalonSrx *t) : talon(t) {}

    void set_counts(int count) override {
      ctre::all_talons()[talon->GetPort()].sensor_pos = count;
    }

    void set_counts_per_sec(int cps) override {
      ctre::all_talons()[talon->GetPort()].sensor_vel = cps / 10;
    }
  };

  class digital_encoder : public encoder_sim {
   public:
    wml::sensors::DigitalEncoder *digital;

    digital_encoder(wml::sensors::DigitalEncoder *d) : digital(d) {}

    void set_counts(int count) override {
      HALSIM_SetEncoderCount(digital->GetSimulationHandle(), count);
    }

    void set_counts_per_sec(int cps) override {
      HALSIM_SetEncoderPeriod(digital->GetSimulationHandle(), 1.0 / (double)cps);
    }
  };

  inline std::shared_ptr<encoder_sim> create_encoder(wml::sensors::Encoder *enc) {
    if (enc == nullptr)
      return nullptr;
    
    if (wml::TalonSrx *srx = dynamic_cast<wml::TalonSrx *>(enc)) {
      // Talon SRX Encoder
      return (std::shared_ptr<encoder_sim>) std::make_shared<talonsrx_encoder>(srx);
    } else if (wml::sensors::DigitalEncoder *digital = dynamic_cast<wml::sensors::DigitalEncoder *>(enc)) {
      // Digital RoboRIO Encoder
      return (std::shared_ptr<encoder_sim>) std::make_shared<digital_encoder>(digital);
    } else {
      // TODO: Fail condition
      std::cout << "Unknown Evelator Encoder Type" << std::endl;
      return nullptr;
    }
  }
}
}