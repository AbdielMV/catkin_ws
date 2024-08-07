#ifndef DYNAMIC_INTROSPECTION__VISIBILITY_CONTROL_H_
#define DYNAMIC_INTROSPECTION__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DYNAMIC_INTROSPECTION_EXPORT __attribute__ ((dllexport))
    #define DYNAMIC_INTROSPECTION_IMPORT __attribute__ ((dllimport))
  #else
    #define DYNAMIC_INTROSPECTION_EXPORT __declspec(dllexport)
    #define DYNAMIC_INTROSPECTION_IMPORT __declspec(dllimport)
  #endif
  #ifdef DYNAMIC_INTROSPECTION_BUILDING_LIBRARY
    #define DYNAMIC_INTROSPECTION_PUBLIC DYNAMIC_INTROSPECTION_EXPORT
  #else
    #define DYNAMIC_INTROSPECTION_PUBLIC DYNAMIC_INTROSPECTION_IMPORT
  #endif
  #define DYNAMIC_INTROSPECTION_PUBLIC_TYPE DYNAMIC_INTROSPECTION_PUBLIC
  #define DYNAMIC_INTROSPECTION_LOCAL
#else
  #define DYNAMIC_INTROSPECTION_EXPORT __attribute__ ((visibility("default")))
  #define DYNAMIC_INTROSPECTION_IMPORT
  #if __GNUC__ >= 4
    #define DYNAMIC_INTROSPECTION_PUBLIC __attribute__ ((visibility("default")))
    #define DYNAMIC_INTROSPECTION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DYNAMIC_INTROSPECTION_PUBLIC
    #define DYNAMIC_INTROSPECTION_LOCAL
  #endif
  #define DYNAMIC_INTROSPECTION_PUBLIC_TYPE
#endif
#endif  // DYNAMIC_INTROSPECTION__VISIBILITY_CONTROL_H_
// Generated 07-Aug-2024 00:45:33
// Copyright 2019-2020 The MathWorks, Inc.
