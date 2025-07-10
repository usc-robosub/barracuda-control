#!/bin/bash
# Test script for the SetThrustZero service

echo "Testing SetThrustZero service..."

# Enable thrust zero (set all thrusters to zero)
echo "Setting thrust to zero..."
rosservice call /barracuda/set_thrust_zero "enable_thrust_zero: true"

echo "Waiting 3 seconds..."
sleep 3

# Disable thrust zero (resume normal control)
echo "Resuming normal control..."
rosservice call /barracuda/set_thrust_zero "enable_thrust_zero: false"

echo "Test complete!"