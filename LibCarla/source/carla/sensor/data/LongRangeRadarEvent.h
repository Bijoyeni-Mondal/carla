// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/sensor/SensorData.h"
#include "carla/client/Actor.h"
#include "carla/client/LongRangeRadarSensor.h"
#include "carla/rpc/ActorId.h"
#include <vector>
#include "carla/geom/Transform.h"

namespace carla {
namespace sensor {
namespace data {

class LongRangeRadarEvent : public SensorData {
public:

    explicit LongRangeRadarEvent(
        size_t frame,
        double timestamp,
        const rpc::Transform &sensor_transform,
        ActorId parent,
        std::vector<RadarDataPoint> radar_data)
      : SensorData(frame, timestamp, sensor_transform),
        _parent(parent),
        _radar_data(std::move(radar_data)) {}

    /// Get "self" actor. Actor associated with the radar sensor.
    SharedPtr<client::Actor> GetActor() const;

    /// Get radar data points.
    const std::vector<RadarDataPoint>& GetRadarData() const {
      return _radar_data;
    }

private:

    ActorId _parent;

    std::vector<RadarDataPoint> _radar_data;
};

} // namespace data
} // namespace sensor
} // namespace carla
