#if \
  defined(__AVX512F__) &&  defined(__AVX512CD__) && \
  defined(__AVX512DQ__) && defined(__AVX512BW__) && defined(__AVX512VL__)
char const *info_isa = "ISA" ":" "AVX512SKX";
#elif \
  defined(__AVX512F__) &&  defined(__AVX512CD__) && \
  defined(__AVX512ER__) && defined(__AVX512PF__)
char const *info_isa = "ISA" ":" "AVX512KNL";
#elif defined(__AVX2__)
char const *info_isa = "ISA" ":" "AVX2";
#elif defined(__AVX__)
char const *info_isa = "ISA" ":" "AVX";
#elif defined(__SSE4_2__)
char const *info_isa = "ISA" ":" "SSE42";
#else // defined(__SSE2__)
char const *info_isa = "ISA" ":" "SSE2";
#endif

int main(int argc, char **argv)
{
  int require = 0;
  require += info_isa[argc];
  return require;
}
