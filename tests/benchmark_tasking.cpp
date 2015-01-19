// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

#include "tasking/taskscheduler.h"
#include "math/math.h"
#include <tbb/tbb.h>

namespace embree
{
  struct reduce_benchmark
  {
    reduce_benchmark() 
      : N(0), array(NULL) {}
    
    size_t N;
    double* array;
    double threadReductions[1024];
    
    TASK_FUNCTION_(reduce_benchmark,reduce);
    
    void reduce(size_t threadIndex, size_t threadCount)
    {
      const size_t begin = (threadIndex+0)*N/threadCount;
      const size_t end   = (threadIndex+1)*N/threadCount;
      double c = 0;
      for (size_t i=begin; i<end; i++) c += sin(array[i]);
      threadReductions[threadIndex] = c;
    }
    
    void init(size_t N)
    {
      this->N = N;
      array = new double[N];
      for (size_t i=0; i<N; i++) 
        array[i] = drand48();
    }
    
    double run_sequential (size_t N) 
    {
      double c0 = 0;
      for (size_t i=0; i<N; i++) c0 += sin(array[i]);
      return c0;
    }  
    
    double run_locksteptaskscheduler(size_t N)
    {
      this->N = N;
      LockStepTaskScheduler* scheduler = LockStepTaskScheduler::instance();
      scheduler->dispatchTask( task_reduce, this, 0, scheduler->getNumThreads() );
      
      double c = 0;
      for (size_t i=0; i<scheduler->getNumThreads(); i++) 
        c += threadReductions[i];
      
      return c;
    }
    
    double run_tbb(size_t N)
    {
      double c2 = 0;
      //LockStepTaskScheduler::execute_tbb([&] () {
      c2 = tbb::parallel_reduce(tbb::blocked_range<size_t>(0,N,1024), 0.0, 
                                [&](const tbb::blocked_range<size_t>& r, double value) -> double {
                                  double c = value; 
                                  for (size_t i=r.begin(); i<r.end(); i++) c += sin(array[i]); 
                                  return c;
                                },
                                std::plus<double>());
      //});
      return c2;
    }
    
    const char* name;
  };

  template<typename Closure>
  void benchmark(size_t N0, size_t N1, const char* name, const Closure& closure)
  {
    std::cout << "# " << name << std::endl;
    std::cout << "# N dt_min dt_avg dt_max M/s(min) M/s(avg) M/s(max)" << std::endl;
    for (size_t N = N0; N < N1; N *= 1.5) 
    {
      double t_min = pos_inf;
      double t_avg = 0.0f;
      double t_max = neg_inf;
      for (size_t i=0; i<10; i++)
      {
        double t0 = getSeconds();
        volatile auto c1 = closure(N);
        double dt = getSeconds()-t0;
        t_min = min(t_min,dt);
        if (i != 0) t_avg = t_avg + dt;
        t_max = max(t_max,dt);
      }
      t_avg /= 9.0;
      std::cout << N << " " 
                << 1000.0f*t_min << " " << 1000.0f*t_avg << " " << 1000.0f*t_max << " " 
                << 1E-6*N/t_max << " " << 1E-6*N/t_avg << " " << 1E-6*N/t_min << std::endl;
    }
  }

  reduce_benchmark reduce;

  void main()
  {
    const size_t N = 10*1024*1024;
    reduce.init(N);
    
    benchmark(1000,N,"sequential",[] (size_t N) -> double { return reduce.run_sequential(N); });
    
    TaskScheduler::create();
    execute_closure([] () -> double { benchmark(1000,N,"lockstep",[] (size_t N) -> double { return reduce.run_locksteptaskscheduler(N); }); return 0.0; });
    TaskScheduler::destroy();

    {
      tbb::task_scheduler_init init(tbb::task_scheduler_init::default_num_threads());
      benchmark(1000,N,"tbb",[] (size_t N) -> double { return reduce.run_tbb(N); });
    }
  }
}

int main() {
  embree::main();
  return 0;
}
  
  
  
  
  
  
