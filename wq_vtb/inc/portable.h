#ifdef __GNUC__
#define HAVE_BUILTINS
#endif


#ifdef HAVE_BUILTINS
#define count_num __builtin_popcount
#define prefetch __builtin_prefetch
#else

int count_num(int x);

void prefetch(void* x);

#endif
