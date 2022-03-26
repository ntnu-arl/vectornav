#ifndef CONFIG_HPP_
#define CONFIG_HPP_

#include "thirdparty/config_utilities.hpp"
#include "vectornav/supported_sets.hpp"

namespace vectornav
{
  struct Config : public config_utilities::Config<Config>
  {
    std::string port = "/dev/ttyUSB0";
    uint32_t baud_rate = 921600;
    uint16_t async_port = 2;
    uint16_t async_rate_divisor = 4;
    // Configuration for the sensor being triggered by an external source
    bool is_triggered = false;
    uint16_t sync_in_skip_factor = 0;
    // Configuration for the sensor triggering external objects
    bool is_triggering = true;
    uint16_t sync_out_skip_factor = 39;
    uint32_t sync_out_pulse_width = 1.0e+9;
    bool publish_uncomp_imu = false;
    bool publish_uncomp_mag = false;
    std::string frame_id = "imu_link";

    Config()
    {
      setConfigName("VectorNav-Config");
    }

   protected:
    void checkParams() const override
    {
      checkParamNE(port, std::string(""), "port cannot be an empty string");
      checkParamCond(BAUD_RATES.find(baud_rate) != BAUD_RATES.end(),
                     "baud_rate not supported by the sensor");
    }

    void printFields() const override
    {
      printField("port", port);
      printField("baud_rate", baud_rate);
      printField("async_port", async_port);
      printField("async_rate_divisor", async_rate_divisor);
      printField("is_triggered", is_triggered);
      printField("sync_in_skip_factor", sync_in_skip_factor);
      printField("is_triggering", is_triggering);
      printField("sync_out_skip_factor", sync_out_skip_factor);
      printField("sync_out_pulse_width", sync_out_pulse_width);
      printField("frame_id", frame_id);
      printField("publish_uncomp_imu", publish_uncomp_imu);
      printField("publish_uncomp_mag", publish_uncomp_mag);
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
      rosParam("frame_id", &frame_id);
      rosParam("publish_uncomp_imu", &publish_uncomp_imu);
      rosParam("publish_uncomp_mag", &publish_uncomp_mag);
    }
  };

}  // namespace vectornav
#endif  // CONFIG_HPP_