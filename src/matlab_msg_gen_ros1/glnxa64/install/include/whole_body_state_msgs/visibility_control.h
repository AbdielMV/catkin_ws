#ifndef WHOLE_BODY_STATE_MSGS__VISIBILITY_CONTROL_H_
#define WHOLE_BODY_STATE_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define WHOLE_BODY_STATE_MSGS_EXPORT __attribute__ ((dllexport))
    #define WHOLE_BODY_STATE_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define WHOLE_BODY_STATE_MSGS_EXPORT __declspec(dllexport)
    #define WHOLE_BODY_STATE_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef WHOLE_BODY_STATE_MSGS_BUILDING_LIBRARY
    #define WHOLE_BODY_STATE_MSGS_PUBLIC WHOLE_BODY_STATE_MSGS_EXPORT
  #else
    #define WHOLE_BODY_STATE_MSGS_PUBLIC WHOLE_BODY_STATE_MSGS_IMPORT
  #endif
  #define WHOLE_BODY_STATE_MSGS_PUBLIC_TYPE WHOLE_BODY_STATE_MSGS_PUBLIC
  #define WHOLE_BODY_STATE_MSGS_LOCAL
#else
  #define WHOLE_BODY_STATE_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define WHOLE_BODY_STATE_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define WHOLE_BODY_STATE_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define WHOLE_BODY_STATE_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define WHOLE_BODY_STATE_MSGS_PUBLIC
    #define WHOLE_BODY_STATE_MSGS_LOCAL
  #endif
  #define WHOLE_BODY_STATE_MSGS_PUBLIC_TYPE
#endif
#endif  // WHOLE_BODY_STATE_MSGS__VISIBILITY_CONTROL_H_
// Generated 15-May-2024 19:17:24
// Copyright 2019-2020 The MathWorks, Inc.
