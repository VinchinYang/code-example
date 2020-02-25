#ifndef SLAM_PARTICLE_FILTER_HPP_
#define SLAM_PARTICLE_FILTER_HPP_

#include <slam/sensor_model.hpp>
#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <lcmtypes/particles_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <vector>

class lidar_t;
class OccupancyGrid;

/**
Particle Filter
*   1) Draw N particles from current set of weighted particles.
*   2) Sample an action from the ActionModel and apply it to each of these 
*       particles.
*   3) Compute a weight for each particle using the SensorModel.
*   4) Normalize the weights.
*   5) Use the max-weight or mean-weight pose as the estimated pose for this 
*       update.
*/
class ParticleFilter
{
public:
    
   /// @brief Constructor for ParticleFilter
   ///
   /// @param numParticles Number of particles to use
    ParticleFilter(int numParticles);
    
   /// @brief InitializeFilterAtPose initializes the particle filter with the 
   ///        samples distributed according to the provided pose estimate.
   ///
   /// @param pose Initial pose of the robot
    void initializeFilterAtPose(const pose_xyt_t& pose);
    
    /// @beief UpdateFilter increments the state estimated by the particle filter.
    ///
    /// @param odometry Calculated odometry at the time of the final ray in the 
    ///        laser scan
    /// @param laser    Most recent laser scan of the environment
    /// @param map      Map built from the maximum likelihood pose estimate
    /// @return Estimated robot pose.
    pose_xyt_t updateFilter(const pose_xyt_t&      odometry,
                            const lidar_t& laser,
                            const OccupancyGrid&   map);

    /// @brief poseEstimate retrieves the current pose estimate computed by the 
    ///        filter.
    pose_xyt_t poseEstimate(void) const;
    
   /// @brief particles retrieves the posterior set of particles being used by 
   ///        the algorithm.
    particles_t particles(void) const;
    
private:
    /// @brief The posterior distribution of particles at the end of the 
    ///        previous update.
    std::vector<particle_t> posterior_;
    /// @brief Pose estimate associated with the posterior distribution     
    pose_xyt_t posteriorPose_;              
    /// @brief Action model to apply to particles on each update
    ActionModel actionModel_;   
    /// @brief Sensor model to compute particle weights
    SensorModel sensorModel_;   
    /// @brief Number of particles to use for estimating the pose
    int kNumParticles_;         
    /// @brief resample posterior distribution
    std::vector<particle_t> resamplePosteriorDistribution(void);
    /// @brief compute proposal distribution
    std::vector<particle_t> computeProposalDistribution(
        const std::vector<particle_t>& prior);
    /// @brief compute normalized posterior
    std::vector<particle_t> computeNormalizedPosterior(
        const std::vector<particle_t>& proposal, const lidar_t& laser,
        const OccupancyGrid& map);
    /// @brief estimate posterior pose
    pose_xyt_t estimatePosteriorPose(const std::vector<particle_t>& posterior);
};

#endif // SLAM_PARTICLE_FILTER_HPP_
