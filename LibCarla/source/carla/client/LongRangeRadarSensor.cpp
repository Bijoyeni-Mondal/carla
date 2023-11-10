// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/client/LongRangeRadarSensor.h"
#include "carla/Logging.h"
#include "carla/client/Vehicle.h"
#include "carla/client/detail/Simulator.h"
#include "carla/geom/Transform.h"
#include "carla/sensor/data/LongRangeRadarEvent.h"
#include "carla/client/Actor.h"

#include <exception>
#include <vector>

namespace carla {
namespace client {

// Static local methods for utility
static geom::Location Rotate(float yaw, const geom::Location &location) {
    yaw *= geom::Math::Pi<float>() / 180.0f;
    const float c = std::cos(yaw);
    const float s = std::sin(yaw);
    return {
        c * location.x - s * location.y,
        s * location.x + c * location.y,
        location.z
    };
}

// Radar data point structure
struct RadarDataPoint {
    geom::Location location;
    double relative_velocity;
    double distance;
    double azimuth;
};

// RadarCallback class for handling radar measurements
class RadarCallback {
public:

    RadarCallback(
        const Vehicle &vehicle,
        Sensor::CallbackFunctionType &&user_callback)
        : _parent(vehicle.GetId()),
          _callback(std::move(user_callback)) {}

    void Tick(const WorldSnapshot &snapshot) const;

private:

    ActorId _parent;
    Sensor::CallbackFunctionType _callback;
};

void RadarCallback::Tick(const WorldSnapshot &snapshot) const {
    // Make sure the parent is alive.
    auto parent = snapshot.Find(_parent);
    if (!parent) {
        return;
    }

    // Here, you can implement your radar measurement logic.
    std::vector<RadarDataPoint> radar_data;

    //iterate over detected objects and calculate radar data points.
    for (const auto &object : parent->GetDetectedObjects()) {
        LongRangeRadarEvent data_point;
        data_point.location = object.GetLocation();
        data_point.relative_velocity = object.GetVelocity().Length();
        data_point.distance = (object.GetLocation() - parent->GetLocation()).Length();
        //Calculate azimuth angle in degree
        const geom::vector3D realative_location=object.GetLocation() - parent->GetLocation();
        data_point.azimuth = std::atan2(realative_location.y, realative_location.x)*(180.0/ geom::Math::Pi<double>());
        radar_data.push_back(data_point);
    }

    // Create a LongRangeRadarSensorEvent instance with the collected radar data.
    const auto radar_event = std::make_shared<sensor::data::LongRangeRadarEvent>(
        snapshot.GetFrame(),
        snapshot.GetTimestamp().elapsed_seconds,
        parent->GetTransform(),
        parent->GetId(),
        radar_data);

    // Trigger the callback with the radar event.
    _callback(radar_event);
}

// LongRangeRadarSensor class for handling radar measurements
LongRangeRadarSensor::~LongRangeRadarSensor() {
    Stop();
}

void LongRangeRadarSensor::Listen(CallbackFunctionType callback) {
    auto vehicle = boost::dynamic_pointer_cast<Vehicle>(GetParent());
    if (vehicle == nullptr) {
        log_error(GetDisplayId(), ": not attached to a vehicle");
        return;
    }

    auto episode = GetEpisode().Lock();

    auto cb = std::make_shared<RadarCallback>(
        *vehicle,
        std::move(callback));

    const size_t callback_id = episode->RegisterOnTickEvent([cb = std::move(cb)](const auto &snapshot) {
        try {
            cb->Tick(snapshot);
        } catch (const std::exception &e) {
            log_error("LongRangeRadarSensor:", e.what());
        }
    });

    const size_t previous = _callback_id.exchange(callback_id);
    if (previous != 0u) {
        episode->RemoveOnTickEvent(previous);
    }
}

void LongRangeRadarSensor::Stop() {
    const size_t previous = _callback_id.exchange(0u);
    auto episode = GetEpisode().TryLock();
    if ((previous != 0u) && (episode != nullptr)) {
        episode->RemoveOnTickEvent(previous);
    }
}

} // namespace client
} // namespace carla

	
