#pragma once

#ifdef _WIN32
#ifdef MMHAND_MANUS_SDK_EXPORTS
#define MMHAND_MANUS_API __declspec(dllexport)
#else
#define MMHAND_MANUS_API __declspec(dllimport)
#endif
#else
#define MMHAND_MANUS_API
#endif

extern "C" {

MMHAND_MANUS_API int mmhand_manus_start(
    int connect_locally,
    unsigned int host_search_seconds,
    unsigned int max_raw_skeleton_nodes);

MMHAND_MANUS_API void mmhand_manus_stop();

MMHAND_MANUS_API int mmhand_manus_is_connected();

// Returns:
//   >0: JSON byte length copied to buffer, excluding the trailing NUL.
//    0: no new MANUS frame is available.
//   <0: buffer is too small; required size is -return_value bytes including NUL.
MMHAND_MANUS_API int mmhand_manus_get_latest_frame_json(char* buffer, int buffer_size);

MMHAND_MANUS_API const char* mmhand_manus_last_error();

}
