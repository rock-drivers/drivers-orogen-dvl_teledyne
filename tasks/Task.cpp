/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <dvl_teledyne/Driver.hpp>
#include <aggregator/TimestampEstimator.hpp>

using namespace dvl_teledyne;

Task::Task(std::string const& name)
    : TaskBase(name)
    , mDriver(0)
    , mTimestamper(0)
    , mLastSeq(-1)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
    , mDriver(0)
    , mTimestamper(0)
    , mLastSeq(-1)
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

    delete mTimestamper;
    mTimestamper = new aggregator::TimestampEstimator(
            base::Time::fromSeconds(100));

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
    mTimestamper->reset();
    mLastSeq = -1;
    return true;
}

void Task::processIO()
{
    mDriver->read();
    _info.set(mDriver->deviceInfo);

    // Compute the data timestamp
    base::Time base_time = base::Time::now();
    int64_t seq = mDriver->status.seq;
    if (mLastSeq < 0)
        mGlobalSeq = seq;
    else if (mLastSeq > seq) // wrapped around
        mGlobalSeq += static_cast<uint64_t>((1 << 24) - seq) + mLastSeq;
    else
        mGlobalSeq += seq - mLastSeq;
    mLastSeq = seq;
    base::Time time = mTimestamper->update(base_time, mGlobalSeq);

    // Update the timestamp on each of the fields, and write it on our outputs
    mDriver->status.time = time;
    _status.write(mDriver->status);

    if (!mDriver->cellReadings.time.isNull())
        mDriver->cellReadings.time = time;

    if (!mDriver->bottomTracking.time.isNull())
    {
        mDriver->bottomTracking.time = time;
        _bottom_tracking_samples.write(mDriver->bottomTracking);
    }

    _timestamp_estimator_status.write(mTimestamper->getStatus());
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

