#ifndef DUA_CV_BRIDGE__VISIBILITY_H_
#define DUA_CV_BRIDGE__VISIBILITY_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DUA_CV_BRIDGE_EXPORT __attribute__ ((dllexport))
    #define DUA_CV_BRIDGE_IMPORT __attribute__ ((dllimport))
  #else
    #define DUA_CV_BRIDGE_EXPORT __declspec(dllexport)
    #define DUA_CV_BRIDGE_IMPORT __declspec(dllimport)
  #endif
  #ifdef DUA_CV_BRIDGE_BUILDING_LIBRARY
    #define DUA_CV_BRIDGE_PUBLIC DUA_CV_BRIDGE_EXPORT
  #else
    #define DUA_CV_BRIDGE_PUBLIC DUA_CV_BRIDGE_IMPORT
  #endif
  #define DUA_CV_BRIDGE_PUBLIC_TYPE DUA_CV_BRIDGE_PUBLIC
  #define DUA_CV_BRIDGE_LOCAL
#else
  #define DUA_CV_BRIDGE_EXPORT __attribute__ ((visibility("default")))
  #define DUA_CV_BRIDGE_IMPORT
  #if __GNUC__ >= 4
    #define DUA_CV_BRIDGE_PUBLIC __attribute__ ((visibility("default")))
    #define DUA_CV_BRIDGE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DUA_CV_BRIDGE_PUBLIC
    #define DUA_CV_BRIDGE_LOCAL
  #endif
  #define DUA_CV_BRIDGE_PUBLIC_TYPE
#endif

#endif  // DUA_CV_BRIDGE__VISIBILITY_H_
