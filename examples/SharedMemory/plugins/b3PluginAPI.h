#ifndef B3_PLUGIN_API_H
#define B3_PLUGIN_API_H

#ifdef _WIN32
#define B3_SHARED_API __declspec(dllexport)
#elif defined (__GNUC__)
#define B3_SHARED_API __attribute__((visibility("default")))
#else
#define B3_SHARED_API
#endif


#if defined(_WIN32)
#define B3_API_ENTRY
#define B3_API_CALL     __cdecl 
#define B3_CALLBACK     __cdecl 
#else
#define B3_API_ENTRY
#define B3_API_CALL
#define B3_CALLBACK
#endif



#ifdef __cplusplus
extern "C" {
#endif
	/* Plugin API */
	typedef B3_API_ENTRY int (B3_API_CALL * PFN_INIT)(struct b3PluginContext* context);
	typedef B3_API_ENTRY void (B3_API_CALL * PFN_EXIT)(struct b3PluginContext* context);
	typedef B3_API_ENTRY int (B3_API_CALL * PFN_EXECUTE)(struct b3PluginContext* context, const struct b3PluginArguments* arguments);
	typedef B3_API_ENTRY int (B3_API_CALL * PFN_TICK)(struct b3PluginContext* context);

	typedef B3_API_ENTRY struct UrdfRenderingInterface* (B3_API_CALL * PFN_GET_RENDER_INTERFACE)(struct b3PluginContext* context);
	

#ifdef __cplusplus
}
#endif

#endif //B3_PLUGIN_API_H
