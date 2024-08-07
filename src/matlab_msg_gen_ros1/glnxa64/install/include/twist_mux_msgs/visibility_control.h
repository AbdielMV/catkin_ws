#ifndef TWIST_MUX_MSGS__VISIBILITY_CONTROL_H_
#define TWIST_MUX_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TWIST_MUX_MSGS_EXPORT __attribute__ ((dllexport))
    #define TWIST_MUX_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define TWIST_MUX_MSGS_EXPORT __declspec(dllexport)
    #define TWIST_MUX_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef TWIST_MUX_MSGS_BUILDING_LIBRARY
    #define TWIST_MUX_MSGS_PUBLIC TWIST_MUX_MSGS_EXPORT
  #else
    #define TWIST_MUX_MSGS_PUBLIC TWIST_MUX_MSGS_IMPORT
  #endif
  #define TWIST_MUX_MSGS_PUBLIC_TYPE TWIST_MUX_MSGS_PUBLIC
  #define TWIST_MUX_MSGS_LOCAL
#else
  #define TWIST_MUX_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define TWIST_MUX_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define TWIST_MUX_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define TWIST_MUX_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TWIST_MUX_MSGS_PUBLIC
    #define TWIST_MUX_MSGS_LOCAL
  #endif
  #define TWIST_MUX_MSGS_PUBLIC_TYPE
#endif
#endif  // TWIST_MUX_MSGS__VISIBILITY_CONTROL_H_
// Generated 07-Aug-2024 00:45:39
// Copyright 2019-2020 The MathWorks, Inc.
