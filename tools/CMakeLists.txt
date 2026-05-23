cmake_minimum_required(VERSION 3.16)

add_library(
  tools
  OBJECT
  crc.cpp
  exiter.cpp
  extended_kalman_filter.cpp
  img_tools.cpp
  logger.cpp
  math_tools.cpp
  pid.cpp
  plotter.cpp
  ransac_sine_fitter.cpp
  recorder.cpp
  resistance_trajectory.cpp
  signal_recorder.cpp
  trajectory.cpp
)
