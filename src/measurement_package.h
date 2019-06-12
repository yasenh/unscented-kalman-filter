#pragma once

#include "Eigen/Dense"

class MeasurementPackage {
public:
  long timestamp_;

  enum SensorType {
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;
};