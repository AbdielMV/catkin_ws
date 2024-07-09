#ifndef TF_LOOKUP__VISIBILITY_CONTROL_H_
#define TF_LOOKUP__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TF_LOOKUP_EXPORT __attribute__ ((dllexport))
    #define TF_LOOKUP_IMPORT __attribute__ ((dllimport))
  #else
    #define TF_LOOKUP_EXPORT __declspec(dllexport)
    #define TF_LOOKUP_IMPORT __declspec(dllimport)
  #endif
  #ifdef TF_LOOKUP_BUILDING_LIBRARY
    #define TF_LOOKUP_PUBLIC TF_LOOKUP_EXPORT
  #else
    #define TF_LOOKUP_PUBLIC TF_LOOKUP_IMPORT
  #endif
  #define TF_LOOKUP_PUBLIC_TYPE TF_LOOKUP_PUBLIC
  #define TF_LOOKUP_LOCAL
#else
  #define TF_LOOKUP_EXPORT __attribute__ ((visibility("default")))
  #define TF_LOOKUP_IMPORT
  #if __GNUC__ >= 4
    #define TF_LOOKUP_PUBLIC __attribute__ ((visibility("default")))
    #define TF_LOOKUP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TF_LOOKUP_PUBLIC
    #define TF_LOOKUP_LOCAL
  #endif
  #define TF_LOOKUP_PUBLIC_TYPE
#endif
#endif  // TF_LOOKUP__VISIBILITY_CONTROL_H_
// Generated 09-Jul-2024 13:59:40
// Copyright 2019-2020 The MathWorks, Inc.
