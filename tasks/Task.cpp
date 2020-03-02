/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base-logging/Logging.hpp>

using namespace std;
using namespace usv_camera_simulation;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::~Task()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook()) {
        return false;
    }

    mUTM = gps_base::UTMConverter(_utm_parameters.get());
    auto definitions = _model_definitions.get();
    for (auto const& d : definitions) {
        sdf::SDF sdf;
        sdf.SetFromString(d.sdf);
        auto model_ptr = sdf.Root()->GetElement("model");
        if (!model_ptr) {
            throw std::runtime_error("invalid SDF " + d.sdf);
        }

        Definition new_definition(d);
        new_definition.parsed_sdf = model_ptr;
        mDefinitions.push_back(new_definition);
    }
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook()) {
        return false;
    }
    return true;
}
void Task::updateUI()
{
    ais_base::VesselInformation info;
    while (_vessel_information.read(info) == RTT::NewData) {
        addVesselInformation(info);
    }

    ais_base::Position position;
    while (_vessel_position.read(position) == RTT::NewData) {
        updateVesselPosition(position);
    }

    TaskBase::updateUI();
}

void Task::addVesselInformation(ais_base::VesselInformation const& vessel_info) {
    int mmsi = vessel_info.mmsi;
    auto& info = mInfo[mmsi];
    info.ais_info = vessel_info;

    if (info.ais_position.time.isNull()) {
        std::cout << "received vessel information for " << mmsi
                  << " but no position information received yet" << std::endl;
        return;
    }

    updateVesselPosition(info.ais_position);
}

void Task::updateVesselPosition(ais_base::Position const& position) {
    int mmsi = position.mmsi;
    auto& info = mInfo[mmsi];

    // For our purposes ... if the angle is unknown, use the course. If there's no
    // course, just ignore
    auto yaw = position.yaw;
    if (base::isUnknown(yaw)) {
        yaw = position.course_over_ground;
        if (base::isUnknown(yaw)) {
            std::cout << "ignoring position for " << mmsi
                      << " as it has no heading or course information" << std::endl;
            return;
        }
    }
    info.ais_position = position;
    info.ais_position.yaw = yaw;

    if (info.ais_info.time.isNull()) {
        std::cout << "received position for " << mmsi
                  << " but no vessel information received yet" << std::endl;
        return;
    }

    if (info.frame_id.empty()) {
        std::cout << "creating vessel for " << mmsi << std::endl;
        info.frame_id = createVessel(info);
        if (info.frame_id.empty()) {
            return;
        }
    }

    gps_base::Solution solution;
    solution.latitude = position.latitude.getDeg();
    solution.longitude = position.longitude.getDeg();
    solution.altitude = 0;
    auto nwu = mUTM.convertToNWU(solution);

    auto p = nwu.position;
    auto e = base::getEuler(nwu.orientation);
    std::cout << "MMSI=" << info.frame_id << " at\n"
              << "P: " << p.x() << " " << p.y() << " " << p.z() << "\n"
              << "E: " << e.x() << " " << e.y() << " " << e.z() << "\n"
              << "Y: " << info.ais_position.yaw.getRad() << std::endl;

    vizkit3dWorlds[0]->applyTransformation(
        "world", info.frame_id,
        nwu.position,
        Eigen::Quaterniond(
            Eigen::AngleAxisd(info.ais_position.yaw.getRad(), Eigen::Vector3d::UnitZ())
        )
    );
}

string Task::createVessel(Info const& info) {
    string frame_id = to_string(info.ais_info.mmsi);

    vector<Definition> candidates;
    float length = info.ais_info.length;

    if (mDefinitions.empty()) {
        return "";
    }

    // Also compute the closest in case there is no model matching the vessel
    // length
    float min_length_d = std::numeric_limits<float>::max();
    Definition const* closest_definition = nullptr;
    for (auto const& d : mDefinitions) {
        if (d.min_size <= length && d.max_size >= length) {
            candidates.push_back(d);
        }

        float length_d = std::min(std::abs(d.min_size - length),
                                  std::abs(d.max_size - length));
        if (length_d < min_length_d) {
            min_length_d = length_d;
            closest_definition = &d;
        }
    }

    if (candidates.empty()) {
        candidates.push_back(*closest_definition);
    }

    int chosen_i = static_cast<uint64_t>(rand()) * candidates.size() /
                   (static_cast<uint64_t>(RAND_MAX) + 1);

    auto const& chosen = candidates[chosen_i];
    std::cout << "MMSI=" << info.ais_info.mmsi
              << " length=" << length << " "
              << " is " << chosen.sdf << std::endl;
    chosen.parsed_sdf->GetAttribute("name")->SetFromString(frame_id);
    vizkit3dWorlds[0]->addModel(chosen.parsed_sdf, frame_id, chosen.sdf_version);
    return frame_id;
}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
