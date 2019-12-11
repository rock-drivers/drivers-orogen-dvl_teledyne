/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <dvl_teledyne/Driver.hpp>
#include <iodrivers_base/ConfigureGuard.hpp>
#include <aggregator/TimestampEstimator.hpp>
#include <base/Float.hpp>

using namespace dvl_teledyne;

Task::Task(std::string const& name)
    : TaskBase(name)
    , mTimestamper(0)
    , mLastSeq(-1)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
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
    std::unique_ptr<dvl_teledyne::Driver> driver(new dvl_teledyne::Driver);

    iodrivers_base::ConfigureGuard guard(this);
    if (!_io_port.get().empty())
        driver->openURI(_io_port.get());
    setDriver(driver.get());

    if (! TaskBase::configureHook())
        return false;

    // Driver Configuration
    if (driver->isValid())
    {
        driver->setReadTimeout(_io_read_timeout.get());
        driver->setWriteTimeout(_io_write_timeout.get());
        driver->setConfigurationMode();

        // Send the configuration first, so that it gets overriden in the config
        // file (if people want to do it in the config file)
        driver->applyConfig(_config.get());

        // We can configure only if in direct access mode, i.e. send the file
        // only if _io_port is set
        if (!_config_file.get().empty())
            driver->sendConfigurationFile(_config_file.get());
    }

    delete mTimestamper;
    mTimestamper = new aggregator::TimestampEstimator(
            base::Time::fromSeconds(100),base::Time::fromSeconds(1));

    mDriver = move(driver);
    guard.commit();
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
        // Invert z-axis in order to convert from the left handed frame of the DVL (with z pointing down)
        // to the right handed frame (with z pointing up).
        mDriver->bottomTracking.velocity[2] = -1. * mDriver->bottomTracking.velocity[2];

        mDriver->bottomTracking.time = time;
        _bottom_tracking_samples.write(mDriver->bottomTracking);

        base::samples::RigidBodyState rbs_ground_distance;
        rbs_ground_distance.time = time;
        rbs_ground_distance.invalidate();
        if(
                (!base::isUnknown<float>(mDriver->bottomTracking.range[0])) &&
                (!base::isUnknown<float>(mDriver->bottomTracking.range[1])) &&
                (!base::isUnknown<float>(mDriver->bottomTracking.range[2])) &&
                (!base::isUnknown<float>(mDriver->bottomTracking.range[3]))
          ){
                //Taking the Average distance to the bottom if all readings are valid
                double avg = (mDriver->bottomTracking.range[0] +
                              mDriver->bottomTracking.range[1] +
                              mDriver->bottomTracking.range[2] +
                              mDriver->bottomTracking.range[3])/4.0;
                
                avg *= cos(30.0/180.0*M_PI); //30degree angle of the pistons, convert to distance
                rbs_ground_distance.position[2] = avg;
                rbs_ground_distance.cov_position(2,2) = _variance_ground_distance.get();
        }
        //Write ground distance even we have no lock, then with NaN information
        _ground_distance.write(rbs_ground_distance);
    }
    else
    {
	std::cerr << "dvl_orogen: no DVL data received" << std::endl;
    }

    // Extract RigidBodyState data and write it to speed_samples
    //
    // This is possible in all but BEAM coordinate mode
    if (mDriver->outputConf.coordinate_system != BEAM && !base::isUnknown<float>(mDriver->bottomTracking.velocity[0]))
    {
        base::samples::RigidBodyState rbs_velocity;
        rbs_velocity.invalidate();
        rbs_velocity.time = time;
        // set variance unknown
        var = base::unknown<float>();

	//check for nans 
	if( mDriver->bottomTracking.velocity[0] == mDriver->bottomTracking.velocity[0] && 
	    mDriver->bottomTracking.velocity[1] == mDriver->bottomTracking.velocity[1] && 
	    mDriver->bottomTracking.velocity[2] == mDriver->bottomTracking.velocity[2])
	{
            if(!base::isNaN<double>(_sigma_override.get()) && _sigma_override.get() != 0.0)
            {
                var = pow(_sigma_override.get(), 2.0);
            }
            else if(mDriver->bottomTracking.velocity[3] == mDriver->bottomTracking.velocity[3] )
            { 
	        var = mDriver->bottomTracking.velocity[3] * mDriver->bottomTracking.velocity[3];
	    }

            if(!base::isUnknown<float>(var))
            {

                rbs_velocity.orientation  = mDriver->status.orientation;
                rbs_velocity.velocity.x() = mDriver->bottomTracking.velocity[0];
                rbs_velocity.velocity.y() = mDriver->bottomTracking.velocity[1];
                rbs_velocity.velocity.z() = mDriver->bottomTracking.velocity[2];

                Eigen::Matrix3d cov; 
                cov.setZero(); 
                cov(0, 0) = var;
                cov(1, 1) = var;
                cov(2, 2) = var;
                rbs_velocity.cov_velocity = cov; 

                _velocity_samples.write(rbs_velocity);
            }
	}else{
            //Write this command even we have no bottom-lock indicating it with NaN readings
            _velocity_samples.write(rbs_velocity);
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

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    mDriver.reset();
}

