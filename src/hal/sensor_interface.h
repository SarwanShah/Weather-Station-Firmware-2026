/**
 * @file sensor_interface.h
 * @brief Abstract interface for all sensor drivers
 *
 * Provides a uniform contract that the SensorManager uses to
 * initialise, read, and health-check any sensor.  New sensors
 * (e.g., wind, radiation, rain gauge) implement this interface
 * and register with the SensorManager — no changes to the
 * orchestration code are needed.
 */

#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

#include <cstdint>
#include "../core/data_types.h"

class ISensor {
public:
    virtual ~ISensor() = default;

    /** Human-readable name for logging. */
    virtual const char* name() const = 0;

    /** Initialise hardware; returns true on success. */
    virtual bool begin() = 0;

    /**
     * @brief Perform a single read and populate the relevant
     *        fields of the supplied RawSample.
     * @param[out] sample  Reference to the sample struct to fill.
     * @return true if at least one channel read successfully.
     *
     * Implementations must set the corresponding validity flags
     * (temp_valid, hum_valid, press_valid) in the sample.
     */
    virtual bool read(RawSample& sample) = 0;

    /** Current health status. */
    virtual SensorStatus status() const = 0;

    /** Reset the sensor if it enters a faulted state. */
    virtual bool reset() = 0;
};

#endif // SENSOR_INTERFACE_H
