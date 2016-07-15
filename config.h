/* #undef RTCORE_RAY_MASK */
/* #undef RTCORE_STAT_COUNTERS */
/* #undef RTCORE_BACKFACE_CULLING */
#define RTCORE_INTERSECTION_FILTER
#define RTCORE_INTERSECTION_FILTER_RESTORE
/* #undef RTCORE_BUFFER_STRIDE */
/* #undef RTCORE_ENABLE_RAYSTREAM_LOGGER */
/* #undef RTCORE_RETURN_SUBDIV_NORMAL */
/* #undef RTCORE_IGNORE_INVALID_RAYS */
#define RTCORE_GEOMETRY_TRIANGLES
#define RTCORE_GEOMETRY_QUADS
#define RTCORE_GEOMETRY_LINES
#define RTCORE_GEOMETRY_HAIR
#define RTCORE_GEOMETRY_SUBDIV
#define RTCORE_GEOMETRY_USER
#define RTCORE_RAY_PACKETS

#if defined(RTCORE_GEOMETRY_TRIANGLES)
  #define IF_ENABLED_TRIS(x) x
#else
  #define IF_ENABLED_TRIS(x)
#endif

#if defined(RTCORE_GEOMETRY_QUADS)
  #define IF_ENABLED_QUADS(x) x
#else
  #define IF_ENABLED_QUADS(x)
#endif

#if defined(RTCORE_GEOMETRY_LINES)
  #define IF_ENABLED_LINES(x) x
#else
  #define IF_ENABLED_LINES(x)
#endif

#if defined(RTCORE_GEOMETRY_HAIR)
  #define IF_ENABLED_HAIR(x) x
#else
  #define IF_ENABLED_HAIR(x)
#endif

#if defined(RTCORE_GEOMETRY_SUBDIV)
  #define IF_ENABLED_SUBDIV(x) x
#else
  #define IF_ENABLED_SUBDIV(x)
#endif

#if defined(RTCORE_GEOMETRY_USER)
  #define IF_ENABLED_USER(x) x
#else
  #define IF_ENABLED_USER(x)
#endif




