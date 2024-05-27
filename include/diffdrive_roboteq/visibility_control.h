#ifndef DIFFDRIVE_ROBOTEQ__VISIBILITY_CONTROL_H_
#define DIFFDRIVE_ROBOTEQ__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DIFFDRIVE_ROBOTEQ_EXPORT __attribute__((dllexport))
#define DIFFDRIVE_ROBOTEQ __attribute__((dllimport))
#else
#define DIFFDRIVE_ROBOTEQ_EXPORT __declspec(dllexport)
#define DIFFDRIVE_ROBOTEQ __declspec(dllimport)
#endif
#ifdef DIFFDRIVE_ROBOTEQ_BUILDING_DLL
#define DIFFDRIVE_ROBOTEQ_PUBLIC DIFFDRIVE_ROBOTEQ_EXPORT
#else
#define DIFFDRIVE_ROBOTEQ_PUBLIC DIFFDRIVE_ROBOTEQ
#endif
#define DIFFDRIVE_ROBOTEQ_PUBLIC_TYPE DIFFDRIVE_ROBOTEQ_PUBLIC
#define DIFFDRIVE_ROBOTEQ_LOCAL
#else
#define DIFFDRIVE_ROBOTEQ_EXPORT __attribute__((visibility("default")))
#define DIFFDRIVE_ROBOTEQ
#if __GNUC__ >= 4
#define DIFFDRIVE_ROBOTEQ_PUBLIC __attribute__((visibility("default")))
#define DIFFDRIVE_ROBOTEQ_LOCAL __attribute__((visibility("hidden")))
#else
#define DIFFDRIVE_ROBOTEQ_PUBLIC
#define DIFFDRIVE_ROBOTEQ_LOCAL
#endif
#define DIFFDRIVE_ROBOTEQ_PUBLIC_TYPE
#endif

#endif  // DIFFDRIVE_ROBOTEQ__VISIBILITY_CONTROL_H_