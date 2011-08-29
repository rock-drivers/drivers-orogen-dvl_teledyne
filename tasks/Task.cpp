/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <dvl_teledyne/Driver.hpp>

using namespace dvl_teledyne;

Task::Task(std::string const& name)
    : TaskBase(name)
    , mDriver(0)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
    , mDriver(0)
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
    delete mDriver;
    mDriver = new dvl_teledyne::Driver;
    if (!_io_port.get().empty())
    {
        mDriver->open(_io_port.get());
        // We can configure only if in direct access mode
        if (!_config_file.get().empty())
            mDriver->sendConfigurationFile(_config_file.get());
        else
            mDriver->setConfigurationMode();
    }
    setDriver(mDriver);

    if (! TaskBase::configureHook())
        return false;
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    // Start pinging using the current configuration
    mDriver->startAcquisition();
    return true;
}

void Task::processIO()
{
    mDriver->read();

    _info.set(mDriver->mDeviceInfo);
    _status.write(mDriver->mStatus);
    _bottom_tracking_samples.write(mDriver->mBottomTracking);
}

// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }

void Task::stopHook()
{
    // Make the device stop pinging
    mDriver->setConfigurationMode();
    TaskBase::stopHook();
}

// void Task::cleanupHook()
// {
//     TaskBase::cleanupHook();
// }

