// ======================================================================== //
// Copyright 2017 Kitware, Inc.                                             //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

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
