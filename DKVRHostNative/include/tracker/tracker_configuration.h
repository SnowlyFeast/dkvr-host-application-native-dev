#pragma once

#include <cstdint>
#include <type_traits>
#include <vector>

namespace dkvr {

    struct TrackerBehavior
    {
        static constexpr uint8_t kBitmaskLed     = 0x01;
        static constexpr uint8_t kBitmaskActive  = 0x02;
        static constexpr uint8_t kBitmaskRaw     = 0x04;
        static constexpr uint8_t kBitmaskNominal = 0x08;
        static constexpr uint8_t kBitmaskInvalid = 0x80;

        constexpr uint8_t Encode() const
        {
            uint8_t result = 0;
            if (led)     result |= kBitmaskLed;
            if (active)	 result |= kBitmaskActive;
            if (raw)     result |= kBitmaskRaw;
            if (nominal) result |= kBitmaskNominal;
            return result;
        }

        static constexpr TrackerBehavior Decode(uint8_t behavior)
        {
            if (behavior & kBitmaskInvalid)	
                return TrackerBehavior{ 0 };

            return TrackerBehavior
            {
                .led     = static_cast<bool>(behavior & kBitmaskLed),
                .active  = static_cast<bool>(behavior & kBitmaskActive),
                .raw     = static_cast<bool>(behavior & kBitmaskRaw),
                .nominal = static_cast<bool>(behavior & kBitmaskNominal)
            };
        }

        void Reset()
        {
            led = true;
            active = false;
            raw = false;
            nominal = false;
        }

        bool led;
        bool active;
        bool raw;
        bool nominal;
    };

    struct TrackerCalibration
    {
        void Reset()
        {
            std::fill_n(gyr_transform, 12, 0.0f);
            std::fill_n(acc_transform, 12, 0.0f);
            std::fill_n(mag_transform, 12, 0.0f);
            std::fill_n(noise_variance, 9, 0.0f);
        }

        float gyr_transform[12];
        float acc_transform[12];
        float mag_transform[12];
        float noise_variance[9];

        const float* gyr_noise_var() const { return &noise_variance[0]; }
        const float* acc_noise_var() const { return &noise_variance[3]; }
        const float* mag_noise_var() const { return &noise_variance[6]; }

        float* gyr_noise_var() { return &noise_variance[0]; }
        float* acc_noise_var() { return &noise_variance[3]; }
        float* mag_noise_var() { return &noise_variance[6]; }
    };

    class TrackerConfiguration
    {
    public:
        enum class ConfigurationKey
        {
            Behavior,
            GyrTransform,
            AccTransform,
            MagTransform,
            NoiseVariance,
            Size // keep this member at last, not an actual key
        };

        std::vector<ConfigurationKey> GetEveryInvalid() const
        {
            std::vector<ConfigurationKey> result;
            for (int i = 0; i < static_cast<int>(ConfigurationKey::Size); i++)
                if (!validated_[i]) result.push_back(ConfigurationKey(i));
            return result;
        }
        bool IsValid(ConfigurationKey key) const { return validated_[static_cast<int>(key)]; }
        bool IsAllValid() const {
            for (bool b : validated_) 
                if (!b) return false; 
            return true;
        }

        void Validate(ConfigurationKey key) { validated_[static_cast<int>(key)] = true; }
        void Invalidate(ConfigurationKey key) { validated_[static_cast<int>(key)] = false; }
        void InvalidateAll() { for (bool& b : validated_) b = false; }

        void Reset() { behavior_.Reset(); calibration_.Reset(); }

        void set_led    (bool led)      { behavior_.led     = led;     Invalidate(ConfigurationKey::Behavior); }
        void set_active (bool active)	{ behavior_.active	= active;  Invalidate(ConfigurationKey::Behavior); }
        void set_raw    (bool raw)      { behavior_.raw     = raw;     Invalidate(ConfigurationKey::Behavior); }
        void set_nominal(bool nominal)  { behavior_.nominal = nominal; Invalidate(ConfigurationKey::Behavior); }

        const TrackerBehavior& behavior() const { return behavior_; }
        const TrackerCalibration& calibration() const { return calibration_; }

        TrackerBehavior& behavior() { return behavior_; }
        TrackerCalibration& calibration() { return calibration_; }

        void set_behavior(TrackerBehavior behavior) { behavior_ = behavior; Invalidate(ConfigurationKey::Behavior); }
        void set_calibration(const TrackerCalibration& calibration) {
            calibration_ = calibration;
            Invalidate(ConfigurationKey::GyrTransform);
            Invalidate(ConfigurationKey::AccTransform);
            Invalidate(ConfigurationKey::MagTransform);
            Invalidate(ConfigurationKey::NoiseVariance);
        }

    private:
        TrackerBehavior behavior_;
        TrackerCalibration calibration_;
        bool validated_[static_cast<size_t>(ConfigurationKey::Size)];
    };

    static_assert(std::is_trivial_v<TrackerCalibration>);
    static_assert(std::is_standard_layout_v<TrackerCalibration>);

}	// namespace dkvr