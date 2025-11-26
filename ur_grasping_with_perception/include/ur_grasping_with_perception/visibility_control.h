#ifndef ACTION_FIND_APRILTAG__VISIBILITY_CONTROL_H_
#define ACTION_FIND_APRILTAG__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ACTION_FIND_APRILTAG_EXPORT __attribute__ ((dllexport))
    #define ACTION_FIND_APRILTAG_IMPORT __attribute__ ((dllimport))
  #else
    #define ACTION_FIND_APRILTAG_EXPORT __declspec(dllexport)
    #define ACTION_FIND_APRILTAG_IMPORT __declspec(dllimport)
  #endif
  #ifdef ACTION_FIND_APRILTAG_BUILDING_DLL
    #define ACTION_FIND_APRILTAG_PUBLIC ACTION_FIND_APRILTAG_EXPORT
  #else
    #define ACTION_FIND_APRILTAG_PUBLIC ACTION_FIND_APRILTAG_IMPORT
  #endif
  #define ACTION_FIND_APRILTAG_PUBLIC_TYPE ACTION_FIND_APRILTAG_PUBLIC
  #define ACTION_FIND_APRILTAG_LOCAL
#else
  #define ACTION_FIND_APRILTAG_EXPORT __attribute__ ((visibility("default")))
  #define ACTION_FIND_APRILTAG_IMPORT
  #if __GNUC__ >= 4
    #define ACTION_FIND_APRILTAG_PUBLIC __attribute__ ((visibility("default")))
    #define ACTION_FIND_APRILTAG_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ACTION_FIND_APRILTAG_PUBLIC
    #define ACTION_FIND_APRILTAG_LOCAL
  #endif
  #define ACTION_FIND_APRILTAG_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ACTION_FIND_APRILTAG__VISIBILITY_CONTROL_H_