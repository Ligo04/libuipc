#pragma once

#ifdef __cplusplus
#define UIPC_EXTERN_C extern "C"
#define UIPC_NOEXCEPT noexcept
#else
#define UIPC_EXTERN_C
#define UIPC_NOEXCEPT
#endif

#ifdef _MSC_VER
#define UIPC_FORCE_INLINE inline
#define UIPC_NEVER_INLINE __declspec(noinline)
#define UIPC_EXPORT_API UIPC_EXTERN_C __declspec(dllexport)
#define UIPC_IMPORT_API UIPC_EXTERN_C __declspec(dllimport)
#else
#define UIPC_FORCE_INLINE __attribute__((always_inline, hot)) inline
#define UIPC_NEVER_INLINE __attribute__((noinline))
#define UIPC_EXPORT_API UIPC_EXTERN_C __attribute__((visibility("default")))
#define UIPC_IMPORT_API UIPC_EXTERN_C
#endif

#ifdef _MSC_VER

// CORE
#ifdef UIPC_CORE_EXPORT_DLL
#define UIPC_CORE_API __declspec(dllexport)
#else
#define UIPC_CORE_API __declspec(dllimport)
#endif
// GEOMETRY
#ifdef UIPC_GEOMETRY_EXPORT_DLL
#define UIPC_GEOMETRY_API __declspec(dllexport)
#else
#define UIPC_GEOMETRY_API __declspec(dllimport)
#endif
// CONSTITUTION
#ifdef UIPC_CONSTITUTION_EXPORT_DLL
#define UIPC_CONSTITUTION_API __declspec(dllexport)
#else
#define UIPC_CONSTITUTION_API __declspec(dllimport)
#endif
// IO
#ifdef UIPC_IO_EXPORT_DLL
#define UIPC_IO_API __declspec(dllexport)
#else
#define UIPC_IO_API __declspec(dllimport)
#endif
// BACKEND
#ifdef UIPC_BACKEND_EXPORT_DLL
#define UIPC_BACKEND_API __declspec(dllexport)
#else
#define UIPC_BACKEND_API __declspec(dllimport)
#endif

#else

// CORE
#define UIPC_CORE_API
// GEOMETRY
#define UIPC_GEOMETRY_API
// CONSTITUTION
#define UIPC_CONSTITUTION_API
// IO
#define UIPC_IO_API
// BACKEND
#define UIPC_BACKEND_API

#endif

// Template specialization export

#ifdef _MSC_VER
// Define the template export macro for windows platforms

// CORE
#ifdef UIPC_CORE_EXPORT_DLL
#define UIPC_CORE_EXPORT_TEMPLATE_CLASS(className) template class UIPC_CORE_API className
#else
#define UIPC_CORE_EXPORT_TEMPLATE_CLASS(className) extern template class UIPC_CORE_API className
#endif

// GEOMETRY
#ifdef UIPC_GEOMETRY_EXPORT_DLL
#define UIPC_GEOMETRY_EXPORT_TEMPLATE_CLASS(className) template class UIPC_GEOMETRY_API className
#else
#define UIPC_GEOMETRY_EXPORT_TEMPLATE_CLASS(className)                                    \
    extern template class UIPC_GEOMETRY_API className
#endif

// CONSTITUTION
#ifdef UIPC_CONSTITUTION_EXPORT_DLL
#define UIPC_CONSTITUTION_EXPORT_TEMPLATE_CLASS(className)                                \
    template class UIPC_CONSTITUTION_API className
#else
#define UIPC_CONSTITUTION_EXPORT_TEMPLATE_CLASS(className)                                \
    extern template class UIPC_CONSTITUTION_API className
#endif

// IO
#ifdef UIPC_IO_EXPORT_DLL
#define UIPC_IO_EXPORT_TEMPLATE_CLASS(className) template class UIPC_IO_API className
#else
#define UIPC_IO_EXPORT_TEMPLATE_CLASS(className) extern template class UIPC_IO_API className
#endif

// BACKEND
#ifdef UIPC_BACKEND_EXPORT_DLL
#define UIPC_BACKEND_EXPORT_TEMPLATE_CLASS(className) template class UIPC_BACKEND_API className
#else
#define UIPC_BACKEND_EXPORT_TEMPLATE_CLASS(className)                                     \
    extern template class UIPC_BACKEND_API className
#endif

#else
// Define the template export macro for non-windows platforms

// CORE
#define UIPC_CORE_EXPORT_TEMPLATE_CLASS(className) 

// GEOMETRY
#define UIPC_GEOMETRY_EXPORT_TEMPLATE_CLASS(className)

// CONSTITUTION
#define UIPC_CONSTITUTION_EXPORT_TEMPLATE_CLASS(className)

// IO
#define UIPC_IO_EXPORT_TEMPLATE_CLASS(className)

// BACKEND
#define UIPC_BACKEND_EXPORT_TEMPLATE_CLASS(className)
#endif