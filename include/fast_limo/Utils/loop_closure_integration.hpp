/**
 * Loop Closure Integration for Fast-LIMO
 * Header file for loop closure integration functionality
 */

#ifndef LOOP_CLOSURE_INTEGRATION_HPP
#define LOOP_CLOSURE_INTEGRATION_HPP

#include <ros/ros.h>
#include "fast_limo/Modules/Localizer.hpp"

namespace loop_closure {

// Initialize publishers
void init(ros::NodeHandle& nh, const std::string& world_frame);

// Publish loop closure results
void publish(fast_limo::Localizer& localizer, const fast_limo::Config& config);

} // namespace loop_closure

#endif // LOOP_CLOSURE_INTEGRATION_HPP 