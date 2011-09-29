/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <dvl_teledyne/Driver.hpp>
#include <aggregator/TimestampEstimator.hpp>
#include <base/float.h>

using namespace dvl_teledyne;

Task::Task(std::string const& name)
    : TaskBase(name)
    , mDriver(0)
    , mTimestamper(0)
    , mLastSeq(-1)
{
    OutputConfiguration default_config;
    default_config.coordinate_system = INSTRUMENT;
    default_config.use_attitude = true;
    default_config.use_3beam_solution = true;
    default_config.use_bin_mapping = false;
    _output_configuration.set(default_config);
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
    , mDriver(0)
    , mTimestamper(0)
    , mLastSeq(-1)
{
    OutputConfiguration default_config;
    default_config.coordinate_system = INSTRUMENT;
    default_config.use_attitude = true;
    default_config.use_3beam_solution = true;
    default_config.use_bin_mapping = false;
    _output_configuration.set(default_config);
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
        mDriver->setConfigurationMode();

        // Send the configuration first, so that it gets overriden in the config
        // file (if people want to do it in the config file)
        mDriver->setOutputConfiguration(_output_configuration.get());

        // We can configure only if in direct access mode, i.e. send the file
        // only if _io_port is set
        if (!_config_file.get().empty())
            mDriver->sendConfigurationFile(_config_file.get());
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

    // Extract RigidBodyState data and write it to speed_samples
    //
    // This is possible in all but BEAM coordinate mode
    if (mDriver->outputConf.coordinate_system != BEAM && !base::isUnknown<float>(mDriver->bottomTracking.velocity[0]))
    {
	//check for nans 
	if( mDriver->bottomTracking.velocity[0] == mDriver->bottomTracking.velocity[0] && 
	    mDriver->bottomTracking.velocity[1] == mDriver->bottomTracking.velocity[1] && 
	    mDriver->bottomTracking.velocity[2] == mDriver->bottomTracking.velocity[2] && 
	    mDriver->bottomTracking.velocity[3] == mDriver->bottomTracking.velocity[3] ) 
	{
	    base::samples::RigidBodyState rbs;
	    rbs.invalidate();
	    rbs.time = time;

	    rbs.orientation  = mDriver->status.orientation;
	    rbs.velocity.x() = mDriver->bottomTracking.velocity[0];
	    rbs.velocity.y() = mDriver->bottomTracking.velocity[1];
	    rbs.velocity.z() = mDriver->bottomTracking.velocity[2];
	    
	    double var = mDriver->bottomTracking.velocity[3] * mDriver->bottomTracking.velocity[3];
	    Eigen::Matrix3d cov; 
	    cov.setZero(); 
	    cov(0, 0) = var;
	    cov(1, 1) = var;
	    cov(2, 2) = var;
	    rbs.cov_velocity = cov; 

	    _velocity_samples.write(rbs);
	}
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

