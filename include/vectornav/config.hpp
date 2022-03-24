#ifndef CONFIG_HPP_
#define CONFIG_HPP_

#include "thirdparty/config_utilities.hpp"
#include "vectornav/supported_sets.hpp"

namespace vectornav
{
  struct Config : public config_utilities::Config<Config>
  {
    std::string port;
    uint32_t baud_rate;
    uint16_t async_port;
    uint16_t async_rate_divisor;
    // Configuration for the sensor being triggered by an external source
    bool is_triggered;
    uint16_t sync_in_skip_factor;
    // Configuration for the sensor triggering external objects
    bool is_triggering;
    uint16_t sync_out_skip_factor;
    uint32_t sync_out_pulse_width;
    bool get_uncomp_measurements;

   protected:
    void checkParams() const override
    {
      checkParamNE(port, std::string(""), "port cannot be an empty string");
      checkParamCond(BAUD_RATES.find(baud_rate) != BAUD_RATES.end(),
                     "baud_rate not supported by the sensor");
    }

    void fromRosParam() override
    {
      rosParam("port", &port);
      int i_param;
      rosParam("baud_rate", &i_param);
      baud_rate = static_cast<uint32_t>(i_param);
      rosParam("async_port", &i_param);
      async_port = static_cast<uint16_t>(i_param);
      rosParam("async_rate_divisor", &i_param);
      async_rate_divisor = static_cast<uint16_t>(i_param);
      rosParam("is_triggered", &is_triggered);
      rosParam("sync_in_skip_factor", &i_param);
      sync_in_skip_factor = static_cast<uint16_t>(i_param);
      rosParam("is_triggering", &is_triggering);
      rosParam("sync_out_skip_factor", &i_param);
      sync_out_skip_factor = static_cast<uint16_t>(i_param);
      rosParam("sync_out_pulse_width", &i_param);
      sync_out_pulse_width = static_cast<uint32_t>(i_param);
      rosParam("get_uncomp_measurements", &get_uncomp_measurements);
    }
  };

}  // namespace vectornav
#endif  // CONFIG_HPP_