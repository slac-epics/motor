/* strtok_r prototype */
#ifdef __cplusplus
extern "C" {
#endif

#if !defined(__rtems__)
char* strtok_r(char *, const char *, char **);
#endif

#ifdef __cplusplus
}
#endif
