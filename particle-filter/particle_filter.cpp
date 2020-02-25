#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>
#include <common/timestamp.h>

/// @brief random device 
std::random_device rd2;

/// @brief generator 
std::mt19937 gen2(rd2());

/// @brief comparator
bool comp(particle_t a , particle_t b)
{
    return a.weight > b.weight;
}

ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}

void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    std::normal_distribution<> distribution1(pose.x, fabs(0.015));
    std::normal_distribution<> distribution2(pose.y, fabs(0.015));
    std::normal_distribution<> distribution3(pose.theta, fabs(0.0001));
    pose_xyt_t temp_pose;
    for(int i = 0; i < kNumParticles_; i++){
        temp_pose.x = distribution1(gen2);
        temp_pose.y = distribution2(gen2);
        temp_pose.theta = distribution3(gen2);
        posterior_[i] = {temp_pose, temp_pose, 1.0f/kNumParticles_};
        posteriorPose_ = pose;
    }
}

pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't 
    // move, do nothing
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        int64_t startTime = utime_now();
        auto prior = resamplePosteriorDistribution();       
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);        
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }
    
    posteriorPose_.utime = odometry.utime;
    
    return posteriorPose_;
}

pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}

particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}

std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
   std::vector<particle_t> prior;
    srand(time(NULL));
    for(int i = 0; i < kNumParticles_; i++)
    {
        double rand_num = (double)rand()/(double)RAND_MAX;
        int id = -1;
        double weight_sum = 0;
        while(weight_sum <= rand_num)
        {
            id++;
            weight_sum += posterior_[id].weight;
        }
        prior.push_back(posterior_[id]);
    }
    return prior;
}

std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    std::vector<particle_t> proposal;
    for (int i = 0; i < kNumParticles_; i++) {
        proposal.push_back(actionModel_.applyAction(prior[i]));
    }
    return proposal;
}

std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    std::vector<particle_t> posterior = proposal;
    double weight_sum = 0;
    printf("new set of weight\n");
    for (int i = 0; i < kNumParticles_; i++)
    {
        double w = sensorModel_.likelihood(proposal[i], laser, map);
        posterior[i].weight = exp(w);
        weight_sum += posterior[i].weight;
    }

    for (int i = 0; i < kNumParticles_; i++)
    {
        posterior[i].weight = (double)posterior[i].weight/(double)weight_sum;
    }
    return posterior;
}

pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    sort(posterior_.begin(), posterior_.end(), comp);
    pose_xyt_t pos = posterior_[0].pose;
    pos.x = 0;
    pos.y = 0;
    pos.theta = 0;
    double ws = 0;

    for (int i = 0; i < 50; i++) {
        pos.x += posterior_[i].pose.x * posterior_[i].weight;
        pos.y += posterior_[i].pose.y * posterior_[i].weight;
        pos.theta += posterior_[i].pose.theta;
        ws += posterior_[i].weight;
    }
    pos.x = pos.x/ws;
    pos.y = pos.y/ws;
    pos.theta = convert_angle(pos.theta/50);

    return pos;
}
