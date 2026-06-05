Put the MANUS C++ SDK files here when you want a self-contained project:

  third_party/ManusSDK/include/ManusSDK.h
  third_party/ManusSDK/include/ManusSDKTypes.h
  third_party/ManusSDK/include/ManusSDKTypeInitializers.h
  third_party/ManusSDK/lib/ManusSDK.lib
  third_party/ManusSDK/lib/ManusSDK.dll

Alternatively, keep the SDK elsewhere and pass:

  -DMANUS_SDK_ROOT=C:/path/to/ManusSDK
