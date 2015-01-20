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
#include "tasking/taskscheduler_new.h"

#include "math/math.h"
#include "kernels/algorithms/sort.h"
#include <tbb/tbb.h>

namespace embree
{
  std::fstream fs;

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

    double reduce_sequential(size_t N)
    {
      double sum = 0.0;
      for (size_t i=0; i<N; i++) 
        sum += sin(array[i]);
      return sum;
    }
    
    double run_sequential (size_t N) 
    {
      double t0 = getSeconds();
      double c0 = 0;
      for (size_t i=0; i<N; i++) c0 += sin(array[i]);
      volatile double result = c0;
      double t1 = getSeconds();

      /*double sum = reduce_sequential(N);
      if (abs(sum-c0) > 1E-5) {
        std::cerr << "internal error: " << sum << " != " << c0 << std::endl;
        exit(1);
        }*/
      return t1-t0;
    }  
    
    double run_locksteptaskscheduler(size_t N)
    {
      double t0 = getSeconds();
      this->N = N;
      LockStepTaskScheduler* scheduler = LockStepTaskScheduler::instance();
      scheduler->dispatchTask( task_reduce, this, 0, scheduler->getNumThreads() );
      
      double c = 0;
      for (size_t i=0; i<scheduler->getNumThreads(); i++) 
        c += threadReductions[i];
      volatile double result = c;
      double t1 = getSeconds();

      /*double sum = reduce_sequential(N);
      if (abs(sum-c) > 1E-5) {
        std::cerr << "internal error: " << sum << " != " << c << std::endl;
        exit(1);
        }*/
      return t1-t0;
    }
    
    double run_tbb(size_t N)
    {
      double t0 = getSeconds();
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
      volatile double result = c2;
      double t1 = getSeconds();

      /*double sum = reduce_sequential(N);
      if (abs(sum-c2) > 1E-5) {
        std::cerr << "internal error: " << sum << " != " << c2 << std::endl;
        exit(1);
        }*/
      return t1-t0;
    }
    
    const char* name;
  };

  template<typename Closure>
  void benchmark(size_t N0, size_t N1, const char* name, const Closure& closure)
  {
    fs << "# " << name << std::endl;
    fs << "# N dt_min dt_avg dt_max M/s(min) M/s(avg) M/s(max)" << std::endl;
    for (size_t N = N0; N < N1; N *= 1.5) 
    {
      double t_min = pos_inf;
      double t_avg = 0.0f;
      double t_max = neg_inf;
      for (size_t i=0; i<10; i++)
      {
        double dt = closure(N);
        t_min = min(t_min,dt);
        if (i != 0) t_avg = t_avg + dt;
        t_max = max(t_max,dt);
      }
      t_avg /= 9.0;
      fs << N << " " 
                << 1000.0f*t_min << " " << 1000.0f*t_avg << " " << 1000.0f*t_max << " " 
                << 1E-6*N/t_max << " " << 1E-6*N/t_avg << " " << 1E-6*N/t_min << std::endl;
    }
  }

  reduce_benchmark reduce;

  size_t fib(size_t i)
  {
    if (i == 0) 
      return 0;
    else if (i == 1) 
      return 1;
    else {
      return fib(i-1) + fib(i-2);
    }
  }

  /* regression testing */
  struct task_scheduler_regression_test //: public RegressionTest
  {
    task_scheduler_regression_test(const char* name) : name(name) {
      //registerRegressionTest(this);
    }
    
    bool operator() (size_t N)
    {
      bool passed = true;
      printf("%s::%s ... \n",TOSTRING(isa),name);
      fflush(stdout);

      /* create task scheduler */
      TaskSchedulerNew* scheduler = new TaskSchedulerNew;

      struct Fib
      {
        size_t& r;
        size_t i;
        
        __forceinline Fib (size_t& r, size_t i) : r(r), i(i) {}
        
        void operator() (size_t, size_t) const
        {
          if (i == 0) 
            r = 0;
          else if (i == 1) 
            r = 1;
          else {
            size_t r0; const Fib fib0(r0, i-1);
            size_t r1; const Fib fib1(r1, i-2);
            TaskSchedulerNew::spawn(fib0);
            TaskSchedulerNew::spawn(fib1);
            TaskSchedulerNew::wait();
            r = r0+r1;
          }
        }
      };

      /* parallel calculation of sum of fibonacci number */
      double t0 = getSeconds();
      size_t r0 = fib(N);
      double t1 = getSeconds();
      size_t r1; Fib pfib(r1,N);
      scheduler->spawn_root(pfib);
      double t2 = getSeconds();
      printf("  sequential_fib(%zu) = %zu, %3.2fms\n",N,r0,1000.0f*(t1-t0));
      printf("  parallel_fib  (%zu) = %zu, %3.2fms\n",N,r1,1000.0f*(t2-t1));
      passed = r0 == r1;
            
      /* output if test passed or not */
      if (passed) printf("[passed]\n");
      else        printf("[failed]\n");

      delete scheduler;
      return passed;
    }

    const char* name;
  };

  void main()
  {
#if 1
    task_scheduler_regression_test task_scheduler_regression("task_scheduler_regression_test");
    task_scheduler_regression(40);
#else

    const size_t N = 100*1024*1024;
    const size_t N_seq = 10*1024*1024;

    /* parallel reduction */
    reduce.init(N);
    
    fs.open ("benchmark_reduce_sequential.csv", std::fstream::out);
    benchmark(1000,N_seq,"reduce_sequential",[] (size_t N) -> double { return reduce.run_sequential(N); });
    fs.close();

    fs.open ("benchmark_reduce_lockstep.csv", std::fstream::out);
    TaskScheduler::create();
    execute_closure([&] () -> double { benchmark(1000,N,"reduce_lockstep",[] (size_t N) -> double { return reduce.run_locksteptaskscheduler(N); }); return 0.0; });
    TaskScheduler::destroy();
    fs.close();

    {
      fs.open ("benchmark_reduce_tbb.csv", std::fstream::out);
      tbb::task_scheduler_init init(tbb::task_scheduler_init::default_num_threads());
      benchmark(1000,N,"reduce_tbb",[] (size_t N) -> double { return reduce.run_tbb(N); });
      fs.close();
    }

    /* parallel sort */
    int* src = new int[N];
    int* array = new int[N];
    int* temp = new int[N];
    for (size_t i=0; i<N; i++) src[i] = random<int>();
    for (size_t i=0; i<N; i++) array[i] = i;
    for (size_t i=0; i<N; i++) temp[i] = i;

    fs.open ("benchmark_sort_sequential.csv", std::fstream::out);
    benchmark(1000,N_seq,"sort_sequential",[&] (size_t N) -> double 
    { 
      for (size_t i=0; i<N; i++) array[i] = src[i];
      double t0 = getSeconds();
      std::sort(array,array+N); 
      double t1 = getSeconds();
      //for (size_t i=1; i<N; i++) if (array[i] < array[i-1]) { std::cerr << "internal error" << std::endl;  exit(1); }
      return t1-t0; 
    });
    fs.close();

    {
      fs.open ("benchmark_sort_tbb.csv", std::fstream::out);
      tbb::task_scheduler_init init(tbb::task_scheduler_init::default_num_threads());
      benchmark(1000,N,"sort_tbb",[&] (size_t N) -> double 
      { 
        for (size_t i=0; i<N; i++) array[i] = src[i];
        double t0 = getSeconds();
        tbb::parallel_sort(array,array+N); 
        double t1 = getSeconds();
        //for (size_t i=1; i<N; i++) if (array[i] < array[i-1]) { std::cerr << "internal error" << std::endl;  exit(1); }
        return t1-t0; 
      });
      fs.close();
    }

    fs.open ("benchmark_sort_lockstep.csv", std::fstream::out);
    TaskScheduler::create();
    execute_closure([&] () -> double 
    { 
      benchmark(1000,N,"sort_lockstep",[&] (size_t N) -> double
      { 
        for (size_t i=0; i<N; i++) array[i] = src[i];
        double t0 = getSeconds();
        radix_sort_u32(array,temp,N); 
        double t1 = getSeconds();
        //for (size_t i=1; i<N; i++) if (array[i] < array[i-1]) { std::cerr << "internal error" << std::endl;  exit(1); }
        return t1-t0; 
      });
      return 0.0;
    });
    TaskScheduler::destroy();
    fs.close();
#endif
  }
}

int main() {
  embree::main();
  return 0;
}
  
  
  
  
  
  
