This folder is reserved for project-local public headers if the bridge grows.

For the MANUS SDK itself, prefer:

  third_party/ManusSDK/include

or pass the SDK location to CMake:

  cmake -S . -B build -A x64 -DMANUS_SDK_ROOT=C:/path/to/ManusSDK
