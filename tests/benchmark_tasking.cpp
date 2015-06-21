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
#include "tasking/taskscheduler_tbb.h"
#include "kernels/algorithms/parallel_reduce.h"

#include "math/math.h"
#include "math/bbox.h"

#include <tbb/tbb.h>
#include <tbb/tbb_stddef.h>

#define OUTPUT 1
#define PROFILE 0

#define ENABLE_FIB_BENCHMARK 0
#define ENABLE_REDUCE_BENCHMARK 0
#define ENABLE_BOX_REDUCE_BENCHMARK 1

// ================ TBB ===============

#define USE_TASK_ARENA_CURRENT_SLOT 1 /* For Intel(R) TBB 4.2 U1 and later (TBB_INTERFACE_VERSION >= 7001) */
#define LOG_PINNING 0

#if USE_TASK_ARENA_CURRENT_SLOT
#define TBB_PREVIEW_TASK_ARENA 1 /* To enable tbb::task_arena::current_slot() method */
#include <tbb/task_arena.h>
#endif
#include <tbb/task_scheduler_observer.h>
#include <tbb/atomic.h>

#include <tbb/task_scheduler_init.h>
#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range.h>
#include <tbb/tick_count.h>

#if defined(__MACOSX__)
#define ENABLE_PINNING 0
#else
#define ENABLE_PINNING 1
#endif

class pinning_observer: public tbb::task_scheduler_observer
{
#if ENABLE_PINNING
    cpu_set_t *mask;
    int ncpus;
#endif
  
    const int pinning_step;
    tbb::atomic<int> thread_index;
public:
    pinning_observer( int pinning_step=1 ) : pinning_step(pinning_step), thread_index()
    {
#if ENABLE_PINNING
        for ( ncpus = sizeof(cpu_set_t)/CHAR_BIT; ncpus < 16*1024 /* some reasonable limit */; ncpus <<= 1 ) {
            mask = CPU_ALLOC( ncpus );
            if ( !mask ) break;
            const size_t size = CPU_ALLOC_SIZE( ncpus );
            CPU_ZERO_S( size, mask );
            const int err = sched_getaffinity( 0, size, mask );
            if ( !err ) break;

            CPU_FREE( mask );
            mask = nullptr;
            if ( errno != EINVAL )  break;
        }
        if ( !mask )
            std::cout << "Warning: Failed to obtain process affinity mask. Thread affinitization is disabled." << std::endl;
#endif
    }

/*override*/ void on_scheduler_entry( bool )
  {
#if ENABLE_PINNING
    if ( !mask ) return;
    const size_t size = CPU_ALLOC_SIZE( ncpus );
    const int num_cpus = CPU_COUNT_S( size, mask );
    int thr_idx =
#if USE_TASK_ARENA_CURRENT_SLOT
        tbb::task_arena::current_thread_index();
#else
        thread_index++;
#endif
#if __MIC__
    thr_idx += 1; // To avoid logical thread zero for the master thread on Intel(R) Xeon Phi(tm)
#endif
    thr_idx %= num_cpus; // To limit unique number in [0; num_cpus-1] range

        // Place threads with specified step
        int cpu_idx = 0;
        for ( int i = 0, offset = 0; i<thr_idx; ++i ) {
            cpu_idx += pinning_step;
            if ( cpu_idx >= num_cpus )
                cpu_idx = ++offset;
        }

        // Find index of 'cpu_idx'-th bit equal to 1
        int mapped_idx = -1;
        while ( cpu_idx >= 0 ) {
            if ( CPU_ISSET_S( ++mapped_idx, size, mask ) )
                --cpu_idx;
        }

        cpu_set_t *target_mask = CPU_ALLOC( ncpus );
        CPU_ZERO_S( size, target_mask );
        CPU_SET_S( mapped_idx, size, target_mask );
        const int err = sched_setaffinity( 0, size, target_mask );

        if ( err ) {
            std::cout << "Failed to set thread affinity!\n";
            exit( EXIT_FAILURE );
        }
#if LOG_PINNING
        else {
            std::stringstream ss;
            ss << "Set thread affinity: Thread " << thr_idx << ": CPU " << mapped_idx << std::endl;
            std::cerr << ss.str();
        }
#endif
        CPU_FREE( target_mask );
#else
        static std::once_flag flag;
        std::call_once(flag, [](){ std::cout << "disabled setting of thread affinity" << std::endl; });
#endif
    }

    ~pinning_observer()
    {
#if ENABLE_PINNING
        if ( mask )
            CPU_FREE( mask );
#endif
    }
};

class concurrency_tracker: public tbb::task_scheduler_observer {
    tbb::atomic<int> num_threads;
public:
    concurrency_tracker() : num_threads() { observe(true); }
    /*override*/ void on_scheduler_entry( bool ) { ++num_threads; }
    /*override*/ void on_scheduler_exit( bool ) { --num_threads; }

    int get_concurrency() { return num_threads; }
};

template <typename R, typename S>
R tbb_pi( S num_steps )
{
    const R step = R(1) / num_steps;
    return step * tbb::parallel_reduce( tbb::blocked_range<S>( 0, num_steps ), R(0),
        [step] ( const tbb::blocked_range<S> r, R local_sum ) -> R {
            for ( S i = r.begin(); i < r.end(); ++i ) {
                R x = (i + R(0.5)) * step;
                local_sum += R(4) / (R(1) + x*x);
            }
            return local_sum;
        },
        std::plus<R>()
    );
}


////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

namespace embree
{
  std::fstream fs;

  static const size_t ITER = 10;
 
  struct box_benchmark
  {
    box_benchmark() 
      : N(0), array(nullptr) {}
    
    size_t N;
    BBox3fa* array;
    BBox3fa threadReductions[1024];

    static BBox3fa result;
    static const bool showResult = false;

    void init(size_t N)
    {
      this->N = N;
      array = new BBox3fa[N];
      srand48(N*32323);

      for (size_t i=0; i<N; i++) 
	{
	  float x = drand48();
	  float y = drand48();
	  float z = drand48();
	  BBox3fa b;
	  const float f = 0.1f;
	  b.lower = Vec3fa(x-f,y-f,z-f);
	  b.upper = Vec3fa(x+f,y+f,z+f);
	  array[i] = b;
	}      
    }

    TASK_FUNCTION_(box_benchmark,reduce);
    
    void reduce(size_t threadIndex, size_t threadCount)
    {
      const size_t begin = (threadIndex+0)*N/threadCount;
      const size_t end   = (threadIndex+1)*N/threadCount;
      BBox3fa b( empty );
      for (size_t i=begin; i<end; i++) 
	b.extend(array[i]);
      threadReductions[threadIndex] = b;
    }

    TASK_FUNCTION_(box_benchmark,nop);
    
    void nop(size_t threadIndex, size_t threadCount) {}

    BBox3fa reduce_sequential(size_t N)
    {
      BBox3fa b( empty );
      for (size_t i=0; i<N; i++) 
	b.extend(array[i]);
      return b;
    }

    double run_sequential (size_t N) 
    {
      double t0 = getSeconds();
      result = reduce_sequential(N);
      double t1 = getSeconds();

      if (showResult)
	PRINT( result );

      return t1-t0;
    }  


    double run_locksteptaskscheduler(size_t N)
    {
#if 1
      double t0 = getSeconds();
      this->N = N;
      LockStepTaskScheduler* scheduler = LockStepTaskScheduler::instance();
      scheduler->dispatchTask( task_reduce, this, 0, scheduler->getNumThreads() );
      
      BBox3fa b( empty );
      for (size_t i=0; i<scheduler->getNumThreads(); i++) 
        b.extend( threadReductions[i] );
      result = b;

      double t1 = getSeconds();

      if (showResult)
	PRINT( result );

      return t1-t0;
#endif

#if 0
      LockStepTaskScheduler* scheduler = LockStepTaskScheduler::instance();
      const size_t threadCount = scheduler->getNumThreads();
      PRINT(threadCount);
      while (true) 
      {
        double t0 = getSeconds();
        for (size_t i=0; i<1000; i++) 
          scheduler->dispatchTask( task_nop, this, 0, threadCount );
        double t1 = getSeconds();
        //PRINT((t1-t0)*1E9f/1000.0);
        PRINT(1000.0f*(t1-t0)/1000.0);
      }
      return 0;
#endif
    }
    
    double run_tbb(size_t N)
    {
      double t0 = getSeconds();
      
#define threshold 256
      tbb::affinity_partitioner ap;
      result = tbb::parallel_reduce(tbb::blocked_range<size_t>(0,N,threshold), BBox3fa( empty ), 
				    [&](const tbb::blocked_range<size_t>& r, BBox3fa c) -> BBox3fa {
				      BBox3fa b( empty ); 
				      for (size_t i=r.begin(); i<r.end(); i++) {
					b.extend(array[i]); 
				      }
				      return c.extend(b);
				    },
				    [](const BBox3fa &a,const BBox3fa &b) { BBox3fa c = a; return c.extend(b); },
					ap);
      double t1 = getSeconds();

      if (showResult)
	PRINT( result );

      return t1-t0;
    }

#if 0
    BBox3fa myreduce(size_t n0, size_t n1)
    {
      BBox3fa b( empty );
      if (n1-n0<128) {
	for (size_t i=n0; i<n1; i++)
	  b.extend( array[i] );
	return b;
      }
      else {
	size_t center = (n0+n1)/2;
	BBox3fa c0; TaskSchedulerTBB::spawn(center-n0,[&](){ c0=myreduce(n0,center); });
	BBox3fa c1; TaskSchedulerTBB::spawn(n1-center,[&](){ c1=myreduce(center,n1); });
	TaskSchedulerTBB::wait();
	c0.extend( c1 );
	return c0;
      }
    }

    double run_mytbb(size_t N)
    {
      double t0 = getSeconds();
      BBox3fa c2( empty );
      c2 = myreduce(0,N);
      result = c2;
      double t1 = getSeconds();

      if (showResult)
	PRINT( result );
      
      return t1-t0;
    }
#else

    double run_mytbb(size_t N)
    {
      double t0 = getSeconds();
#if defined(__MIC__)
      setAffinity(0);
#endif

#if 0
      result = parallel_reduce(size_t(0), size_t(N), size_t(1024), BBox3fa(empty), [&] (const range<size_t>& r)
                      { 
                        BBox3fa c0(empty);
                        for (size_t i=r.begin(); i<r.end(); i++)
                          c0.extend(array[i]);
                        return c0;
                      },
                      [] (const BBox3fa& a, const BBox3fa& b) { return merge(a,b); });

#endif

#if 0
      this->N = N;

      const size_t threadCount = TaskSchedulerTBB::threadCount();
      parallel_for(size_t(threadCount),[&](size_t threadIndex) {
          reduce(threadIndex,threadCount);
        });
      
      BBox3fa b( empty );
      for (size_t i=0; i<threadCount; i++) 
        b.extend( threadReductions[i] );
      result = b;
      
      if (showResult)
	PRINT( result );

#endif

#if 0
      const size_t threadCount = TaskSchedulerTBB::threadCount();
      PRINT(threadCount);
      while (true) 
      {
        double t0 = getSeconds();
        for (size_t i=0; i<1000; i++)
          parallel_for(size_t(0),size_t(threadCount),size_t(1),[&](const range<size_t>& r) {});
        double t1 = getSeconds();
        //PRINT((t1-t0)*1E9f/1000.0);
        PRINT(1000.0*(t1-t0)/1000.0);
      }
#endif

      double t1 = getSeconds();

      if (showResult)
	PRINT( result );
      
      return t1-t0;
    }

#endif


  };

  BBox3fa box_benchmark::result;

  struct reduce_benchmark
  {
    reduce_benchmark() 
      : N(0), array(nullptr) {}
    
    size_t N;
    double* array;
    double threadReductions[1024];
    
    TASK_FUNCTION_(reduce_benchmark,reduce);
    
    void reduce(size_t threadIndex, size_t threadCount)
    {
      const size_t begin = (threadIndex+0)*N/threadCount;
      const size_t end   = (threadIndex+1)*N/threadCount;
      double c = 0;
      for (size_t i=begin; i<end; i++) 
	for (size_t j=0; j<ITER; j++)
	  c += sin(array[i]);
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
	for (size_t j=0; j<ITER; j++)
	  sum += sin(array[i]);
      return sum;
    }
    
    double run_sequential (size_t N) 
    {
      double t0 = getSeconds();
      double c0 = 0;
      for (size_t i=0; i<N; i++) 
	for (size_t j=0; j<ITER; j++)
	  c0 += sin(array[i]);
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
      c2 = tbb::parallel_reduce(tbb::blocked_range<size_t>(0,N,128), 0.0, 
                                [&](const tbb::blocked_range<size_t>& r, double value) -> double {
                                  double c = value; 
                                  for (size_t i=r.begin(); i<r.end(); i++) {
				    for (size_t j=0; j<ITER; j++)
				      c += sin(array[i]); 
				  }
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

#if 0
    double myreduce(size_t n0, size_t n1)
    {
      double c = 0;
      if (n1-n0<128) {
	for (size_t i=n0; i<n1; i++)
	  for (size_t j=0; j<ITER; j++)
	    c += sin(array[i]);
	return c;
      }
      else {
	size_t center = (n0+n1)/2;
	double c0; TaskSchedulerTBB::spawn(center-n0,[&](){ c0=myreduce(n0,center); });
	double c1; TaskSchedulerTBB::spawn(n1-center,[&](){ c1=myreduce(center,n1); });
	TaskSchedulerTBB::wait();
	return c0+c1;
      }
    }

    double run_mytbb(size_t N)
    {
      double t0 = getSeconds();
      double c2 = 0;
      c2 = myreduce(0,N);
      volatile double result = c2;
      double t1 = getSeconds();
      
      /*double sum = reduce_sequential(N);
      if (abs(sum-c2) > 1E-5) {
        std::cerr << "internal error: " << sum << " != " << c2 << std::endl;
        exit(1);
	}*/
      return t1-t0;
    }
#endif
    
    const char* name;
  };

#if ENABLE_REDUCE_BENCHMARK == 1
    reduce_benchmark reduce;
#elif ENABLE_BOX_REDUCE_BENCHMARK == 1
    box_benchmark reduce;
#endif

  template<typename Closure>
  void benchmark(size_t N0, size_t N1, const char* name, const Closure& closure)
  {
#if OUTPUT == 1
    std::cout << "# " << name << std::endl;
    std::cout << "# N dt_min dt_avg dt_max M/s(min) M/s(avg) M/s(max)" << std::endl;
#endif
    for (size_t N = N0; N < N1; N *= 1.5) 
    //for (size_t N = N1; N >= N0; N *= 1./1.5) 

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
#if OUTPUT == 1
      std::cout << N << " " 
		<< 1000.0f*t_min << " " << 1000.0f*t_avg << " " << 1000.0f*t_max << " " 
		<< 1E-6*N/t_max << " " << 1E-6*N/t_avg << " " << 1E-6*N/t_min << std::endl;
#endif
    }
  }

  template<typename Closure>
  void benchmark_fixed_n(size_t N, const char* name, const Closure& closure)
  {
#if OUTPUT == 1
    std::cout << "# " << name << std::endl;
    std::cout << "# N dt_min dt_avg dt_max M/s(min) M/s(avg) M/s(max)" << std::endl;
#endif
    //for (size_t N = N0; N < N1; N *= 1.5)  // orig for version
    //for (size_t N = N1; N >= N0; N *= 1./1.5) 
    for (size_t i = 0; i < 10; i++) 
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
#if OUTPUT == 1
      std::cout << N << " " 
		<< 1000.0f*t_min << " " << 1000.0f*t_avg << " " << 1000.0f*t_max << " " 
		<< 1E-6*N/t_max << " " << 1E-6*N/t_avg << " " << 1E-6*N/t_min << std::endl;
#endif
    }
  }



  const int CUTOFF = 20;

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
      Ref<TaskSchedulerTBB> scheduler = new TaskSchedulerTBB;

#if 1
      struct Fib
      {
        size_t& r;
        size_t i;
        
        __forceinline Fib (size_t& r, size_t i) : r(r), i(i) {}
        
        void operator() () const
        {
          //mutex.lock(); PRINT2(TaskSchedulerTBB::thread()->threadIndex,i); mutex.unlock();
          if (i < CUTOFF) {
            r = fib(i); //i;
          } else {
            size_t r0; const Fib fib0(r0, i-1);
            size_t r1; const Fib fib1(r1, i-2);
            TaskSchedulerTBB::spawn(fib0);
            TaskSchedulerTBB::spawn(fib1);
            TaskSchedulerTBB::wait();
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
      
#else

      atomic_t cntr = 0;
      const size_t M = 160000;
      
      scheduler->spawn_root([&]() 
      {
	  scheduler->spawn(0,M,1,[&] (const range<size_t>& r)
			   {
			     for (size_t i=r.begin(); i<r.end(); i++)
			       atomic_add(&cntr,i);
			   });
	  scheduler->wait();
      });
      
      PRINT2(cntr,M*(M-1)/2);
#endif
      
      delete scheduler;
      return passed;
    }

    const char* name;
  };

  size_t Fib(size_t n)
  {
    if (n<CUTOFF) {
      return fib(n);
    }
    else {
      size_t x, y;
      //for (volatile size_t i=0; ; i++);
      tbb::parallel_invoke([&]{x=Fib(n-1);}, [&]{y=Fib(n-2);});
      //if (n == 40) *((char*)nullptr) = 0;
      return x+y;
    }
  }

  /*
  struct myobserver : public tbb::task_scheduler_observer
  {
    virtual void on_scheduler_entry( bool is_worker ) {
      PRINT("on_scheduler_entry");
    }
    virtual void on_scheduler_exit( bool is_worker ) {
      PRINT("on_scheduler_exit");
    }
  };
  */

  void main(int argc, const char* argv[])
  {
#if ENABLE_FIB_BENCHMARK == 1
    const size_t N = 40;
    //const size_t N = 22;
    //tbb::task_arena limited(2);
    //limited.my_limit = 1000000;

    {
      task_scheduler_regression_test task_scheduler_regression("task_scheduler_regression_test");
      task_scheduler_regression(N);
    }
    {
      tbb::task_scheduler_init init(tbb::task_scheduler_init::default_num_threads());
      //tbb::task_scheduler_init init(8);
      //myobserver observer; observer.observe();
      double t0 = getSeconds();
      size_t r0 = Fib(N);
      double t1 = getSeconds();
      //sleep(1);
      //*((char*)nullptr) = 0;
      //Fib(N);
      printf("  tbb_fib(%zu) = %zu, %3.2fms\n",N,r0,1000.0f*(t1-t0));

    }
#else

    const size_t N_start = 1000;
    const size_t N = 10*1024*1024;
    const size_t N_seq = 10*1024*1024;

    int test = 0;
    if (argc > 1) test = atoi(argv[1]);

    /* parallel reduction */
    reduce.init(N);

    if (test == 1) {
      fs.open ("benchmark_reduce_sequential.csv", std::fstream::out);
      benchmark(N_start,N_seq,"reduce_sequential",[] (size_t N) -> double { return reduce.run_sequential(N); });
      fs.close();
    }

    if (test == 2) {
      fs.open ("benchmark_reduce_lockstep.csv", std::fstream::out);
      TaskScheduler::create();
#if PROFILE == 1
      while(1)
#endif
      execute_closure([&] () -> double { benchmark(N_start,N,"reduce_lockstep",[] (size_t N) -> double { return reduce.run_locksteptaskscheduler(N); }); return 0.0; });
      TaskScheduler::destroy();
      fs.close();
    }

    if (test == 3)
    {
      fs.open ("benchmark_reduce_tbb.csv", std::fstream::out);
      //tbb::task_scheduler_init init(128);
      const bool use_pinning = true;

      tbb::task_scheduler_init init(tbb::task_scheduler_init::default_num_threads());
      PRINT( TBB_INTERFACE_VERSION );
      PRINT( tbb::TBB_runtime_interface_version() );

      pinning_observer pinner( 4 /* the number of hyper threads on each core */ );
      pinner.observe( use_pinning );

      // Warmer
      concurrency_tracker tracker;
      while (tracker.get_concurrency() < tbb::task_scheduler_init::default_num_threads()) tbb_pi<double> (N);

#if PROFILE == 1
      while(1)
#endif
	benchmark(N_start,N,"reduce_tbb",[] (size_t N) -> double { return reduce.run_tbb(N); });

      // Always disable observation before observers destruction
      tracker.observe( false );
      pinner.observe( false );
      
      fs.close();
    }

    if (test == 4)
    {
      TaskSchedulerTBB::create(0);
      //newscheduler = new TaskSchedulerTBB(0,true); // false
      fs.open ("benchmark_reduce_mytbb.csv", std::fstream::out);
#if PROFILE == 1
      while(1)
#endif
	benchmark(N_start,N,"reduce_mytbb",[] (size_t N) -> double { return reduce.run_mytbb(N); });
      fs.close();
      //delete newscheduler;
      TaskSchedulerTBB::destroy();
    }

    if (test == 5)
    {
      fs.open ("benchmark_reduce_tbb.csv", std::fstream::out);
      //tbb::task_scheduler_init init(128);
      const bool use_pinning = true;


      tbb::task_scheduler_init init(tbb::task_scheduler_init::default_num_threads());
      PRINT( TBB_INTERFACE_VERSION );
      PRINT( tbb::TBB_runtime_interface_version() );

      pinning_observer pinner( 4 /* the number of hyper threads on each core */ );
      pinner.observe( use_pinning );

      // Warmer
      concurrency_tracker tracker;
      while (tracker.get_concurrency() < tbb::task_scheduler_init::default_num_threads()) tbb_pi<double> (N);
	
      TaskScheduler::create();
      int n = 291871;	

      while(1){
          benchmark_fixed_n(n,"reduce_tbb",[] (size_t N) -> double { return reduce.run_tbb(N); });	
      }	     
      // Always disable observation before observers destruction
      tracker.observe( false );
      pinner.observe( false );
      
      fs.close();
      TaskScheduler::destroy();
    }

    if (test == 6)
      {
	fs.open ("benchmark_reduce_lockstep.csv", std::fstream::out);

	TaskScheduler::create();
      int n = 291871;	

      while(1){
	execute_closure([&] () -> double { benchmark_fixed_n(n,"reduce_lockstep",[] (size_t N) -> double { return reduce.run_locksteptaskscheduler(N); }); return 0.0; });


      }
      fs.close();
      TaskScheduler::destroy();

      }

#endif
  }
}

int main(int argc, const char* argv[]) 
{
  embree::main(argc,argv);
  return 0;
}
  
  


#if 0
    /* parallel sort */
    int* src = new int[N];
    int* array = new int[N];
    int* temp = new int[N];
    for (size_t i=0; i<N; i++) src[i] = random<int>();
    for (size_t i=0; i<N; i++) array[i] = i;
    for (size_t i=0; i<N; i++) temp[i] = i;

    fs.open ("benchmark_sort_sequential.csv", std::fstream::out);
    benchmark(N_start,N_seq,"sort_sequential",[&] (size_t N) -> double 
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
      benchmark(N_start,N,"sort_tbb",[&] (size_t N) -> double 
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
      benchmark(N_start,N,"sort_lockstep",[&] (size_t N) -> double
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
  
  
  
  
