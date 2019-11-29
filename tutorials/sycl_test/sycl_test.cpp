
#include <CL/sycl.hpp>
#include <iostream>
#include <memory>

class NEOGPUDeviceSelector : public cl::sycl::device_selector
{
public:
  int operator()(const cl::sycl::device &Device) const override {
    using namespace cl::sycl::info;
    
    const std::string DeviceName = Device.get_info<device::name>();
    const std::string DeviceVendor = Device.get_info<device::vendor>();      
    return Device.is_gpu() && DeviceName.find("HD Graphics NEO") ? 1 : -1;
  }
};

class CPUDeviceSelector : public cl::sycl::device_selector {
  public:
  int operator()(const cl::sycl::device &Device) const override {
    return Device.is_cpu() ? 1 : -1;
  }
};

void exception_handler(cl::sycl::exception_list exceptions)
{
  for (std::exception_ptr const& e : exceptions) {
    try {
      std::rethrow_exception(e);
    } catch(cl::sycl::exception const& e) {
      std::cout << "Caught asynchronous SYCL exception: " << e.what() << std::endl;
    }
  }
};


struct Test
{
  Test (std::string name)
    : name(name) {}

  virtual ~Test() {}

  virtual bool run(cl::sycl::device& myDevice, cl::sycl::context myContext, cl::sycl::queue& myQueue) = 0;

  void invoke(cl::sycl::device& myDevice, cl::sycl::context myContext, cl::sycl::queue& myQueue)
  {
    std::cout << name << ": " << std::flush;
    bool passed = run(myDevice,myContext,myQueue);
    if (passed) std::cout << " [PASSED]" << std::endl;
    else        std::cout << " [FAILED]" << std::endl;
  }
  
  std::string name;
};

struct parallel_for_sycl_buffer_test : public Test
{
  parallel_for_sycl_buffer_test ()
    : Test("parallel_for_sycl_buffer_test") {}
  
  bool run (cl::sycl::device& myDevice, cl::sycl::context myContext, cl::sycl::queue& myQueue)
  {
    std::vector<int> h_a(1000);
    std::vector<int> h_b(1000);
    std::vector<int> h_c(1000);
    
    for (int i=0; i<1000; i++) {
      h_a[i] = i;
      h_b[i] = i+5;
      h_c[i] = 0;
    }
    
    {
      cl::sycl::buffer<int> d_a(h_a);
      cl::sycl::buffer<int> d_b(h_b);
      cl::sycl::buffer<int> d_c(h_c);
      
      myQueue.submit([&](cl::sycl::handler& cgh) {
          auto a = d_a.get_access<cl::sycl::access::mode::read>(cgh);
          auto b = d_b.get_access<cl::sycl::access::mode::read>(cgh);
          auto c = d_c.get_access<cl::sycl::access::mode::write>(cgh);
          cgh.parallel_for<class test>(cl::sycl::range<1>(1000), [=](cl::sycl::id<1> item) {
              int i = item.get(0);
              c[i] = a[i] + b[i];
            });
        });

      myQueue.wait_and_throw();
    }
    
    for (int i=0; i<1000; i++)
    {
      if (h_a[i]+h_b[i] != h_c[i])
        return false;
    }
    
    return true;
  }
};

struct parallel_for_sycl_aligned_alloc_test : public Test
{
  parallel_for_sycl_aligned_alloc_test ()
    : Test("parallel_for_sycl_aligned_alloc_test") {}
  
  bool run (cl::sycl::device& myDevice, cl::sycl::context myContext, cl::sycl::queue& myQueue)
  {
    int* a = (int*) cl::sycl::aligned_alloc(64,1000*sizeof(int),myDevice,myContext,cl::sycl::usm::alloc::shared);
    int* b = (int*) cl::sycl::aligned_alloc(64,1000*sizeof(int),myDevice,myContext,cl::sycl::usm::alloc::shared);
    int* c = (int*) cl::sycl::aligned_alloc(64,1000*sizeof(int),myDevice,myContext,cl::sycl::usm::alloc::shared);

    for (int i=0; i<1000; i++) {
      a[i] = i;
      b[i] = i+5;
      c[i] = 0;
    }
    
    {
      myQueue.submit([&](cl::sycl::handler& cgh) {
          cgh.parallel_for<class test>(cl::sycl::range<1>(1000), [=](cl::sycl::id<1> item) {
              int i = item.get(0);
              c[i] = a[i] + b[i];
            });
        });
      myQueue.wait_and_throw();
    }
    
    for (int i=0; i<1000; i++)
    {
      if (a[i]+b[i] != c[i])
        return false;
    }

    cl::sycl::free(a, myContext);
    cl::sycl::free(b, myContext);
    cl::sycl::free(c, myContext);
    
    return true;
  }
};

struct subgroup_test : public Test
{
  subgroup_test ()
    : Test("subgroup_test") {}

  [[cl::intel_reqd_sub_group_size(16)]] static inline void test(const uint groupID, cl::sycl::intel::sub_group& subgroup, int* a, int* b, int* c)
  {
    const uint subgroupLocalID = subgroup.get_local_id()[0];
    const uint i = groupID*16+subgroupLocalID;
    c[i] = a[i] + b[i];
  }
                                                                              
  bool run (cl::sycl::device& myDevice, cl::sycl::context myContext, cl::sycl::queue& myQueue)
  {
    int* a = (int*) cl::sycl::aligned_alloc(64,1000*sizeof(int),myDevice,myContext,cl::sycl::usm::alloc::shared);
    int* b = (int*) cl::sycl::aligned_alloc(64,1000*sizeof(int),myDevice,myContext,cl::sycl::usm::alloc::shared);
    int* c = (int*) cl::sycl::aligned_alloc(64,1000*sizeof(int),myDevice,myContext,cl::sycl::usm::alloc::shared);

    for (int i=0; i<1000; i++) {
      a[i] = i;
      b[i] = i+5;
      c[i] = 0;
    }
    
    {
      myQueue.submit([&](cl::sycl::handler &cgh) {
          
          const cl::sycl::nd_range<1> nd_range(cl::sycl::range<1>(256 * 16), cl::sycl::range<1>(16));
		  
          cgh.parallel_for(nd_range,[=](cl::sycl::nd_item<1> item) {
              const uint groupID = item.get_group(0);
              cl::sycl::intel::sub_group subgroup = item.get_sub_group();
              test(groupID,subgroup,a,b,c);
            });		  
        });
      
      myQueue.wait_and_throw();
    }
    
    for (int i=0; i<1000; i++)
    {
      if (a[i]+b[i] != c[i])
        return false;
    }

    cl::sycl::free(a, myContext);
    cl::sycl::free(b, myContext);
    cl::sycl::free(c, myContext);
    
    return true;
  }
};

typedef int (*fptr_ty) (int a, int b);
[[intel::device_indirectly_callable]] int add_function(int a, int b) { return a + b; }
[[intel::device_indirectly_callable]] int sub_function(int a, int b) { return a - b; }

struct function_pointer_test : public Test
{
  static const int size = 1000;

  function_pointer_test ()
    : Test("function_pointer_test") {}

  bool run (cl::sycl::device& myDevice, cl::sycl::context myContext, cl::sycl::queue& myQueue)
  {
    std::vector<int> A(size, 1);
    std::vector<int> B(size, 2);
    std::vector<int> C(size, 0);

    std::generate(A.begin(), A.end(), std::rand);
    std::generate(B.begin(), B.end(), std::rand);
    std::generate(C.begin(), C.end(), std::rand);
    
    cl::sycl::buffer<int> bufA(A.data(), cl::sycl::range<1>(size));
    cl::sycl::buffer<int> bufB(B.data(), cl::sycl::range<1>(size));
    cl::sycl::buffer<int> bufC(C.data(), cl::sycl::range<1>(size));
    
    myQueue.submit([&](cl::sycl::handler& cgh) {
        
        auto accA = bufA.get_access<cl::sycl::access::mode::read>(cgh);
        auto accB = bufB.get_access<cl::sycl::access::mode::read>(cgh);
        auto accC = bufC.get_access<cl::sycl::access::mode::write>(cgh);
        
        cgh.parallel_for(cl::sycl::range<1>(size), [=](cl::sycl::id<1> index) {
            fptr_ty fptr = &add_function;
            accC[index] = fptr(accA[index],accB[index]);
          });
      });
      
    auto hostA = bufA.get_access<cl::sycl::access::mode::read>();
    auto hostB = bufB.get_access<cl::sycl::access::mode::read>();
    auto hostC = bufC.get_access<cl::sycl::access::mode::read>();

    bool passed = true;
    for (size_t i=0; i<size; i++)
      passed &= hostA[i] + hostB[i] == hostC[i];
    
    return passed;
  }
};

struct function_pointer_take_address_on_device_test : public Test
{
  static const int size = 1000;

  function_pointer_take_address_on_device_test ()
    : Test("function_pointer_take_address_on_device_test") {}

  bool run (cl::sycl::device& myDevice, cl::sycl::context myContext, cl::sycl::queue& myQueue)
  {
    bool passed = true;
  
    for (int Mode = 0; Mode < 2; ++Mode)
    {
      std::vector<int> A(size, 1);
      std::vector<int> B(size, 2);
      std::vector<int> C(size, 0);
      
      cl::sycl::buffer<int> bufA(A.data(), cl::sycl::range<1>(size));
      cl::sycl::buffer<int> bufB(B.data(), cl::sycl::range<1>(size));
      cl::sycl::buffer<int> bufC(C.data(), cl::sycl::range<1>(size));
      
      myQueue.submit([&](cl::sycl::handler &cgh) {
          
          auto accA = bufA.get_access<cl::sycl::access::mode::read>(cgh);
          auto accB = bufB.get_access<cl::sycl::access::mode::read>(cgh);
          auto accC = bufC.get_access<cl::sycl::access::mode::write>(cgh);
          
          cgh.parallel_for(cl::sycl::range<1>(size), [=](cl::sycl::id<1> index) {
              fptr_ty fptr = Mode ? &sub_function : &add_function;
              accC[index] = fptr(accA[index], accB[index]);
            });
        });
      
      auto hostA = bufA.get_access<cl::sycl::access::mode::read>();
      auto hostB = bufB.get_access<cl::sycl::access::mode::read>();
      auto hostC = bufC.get_access<cl::sycl::access::mode::read>();

      if (Mode == 0) {
        for (size_t i=0; i<size; i++) passed &= hostA[i] + hostB[i] == hostC[i];
      } else {
        for (size_t i=0; i<size; i++) passed &= hostA[i] - hostB[i] == hostC[i];
      }
    }
    return passed;
  }
};

struct function_pointer_take_address_on_device_array_test : public Test
{
  static const int size = 1000;
  
  function_pointer_take_address_on_device_array_test ()
    : Test("function_pointer_take_address_on_device_array_test") {}

  bool run (cl::sycl::device& myDevice, cl::sycl::context myContext, cl::sycl::queue& myQueue)
  {
    bool passed = true;
    
    cl::sycl::buffer<cl_ulong> DispatchTableBuffer(2);
    {
      myQueue.submit([&](cl::sycl::handler &cgh) {
          auto accDT = DispatchTableBuffer.get_access<cl::sycl::access::mode::discard_write>(cgh);
          cgh.single_task([=]() {
              accDT[0] = reinterpret_cast<cl_ulong>(&add_function);
              accDT[1] = reinterpret_cast<cl_ulong>(&sub_function);
            });
        });
    }
    
    for (int Mode = 0; Mode < 2; ++Mode)
    {
      std::vector<int> A(size, 1);
      std::vector<int> B(size, 2);
      std::vector<int> C(size, 0);
      
      cl::sycl::buffer<int> bufA(A.data(), cl::sycl::range<1>(size));
      cl::sycl::buffer<int> bufB(B.data(), cl::sycl::range<1>(size));
      cl::sycl::buffer<int> bufC(C.data(), cl::sycl::range<1>(size));
      
      myQueue.submit([&](cl::sycl::handler &cgh) {
          
          auto accA = bufA.get_access<cl::sycl::access::mode::read>(cgh);
          auto accB = bufB.get_access<cl::sycl::access::mode::read>(cgh);
          auto accC = bufC.get_access<cl::sycl::access::mode::write>(cgh);
          auto accDT = DispatchTableBuffer.get_access<cl::sycl::access::mode::read>(cgh);
          
          cgh.parallel_for<class K>(cl::sycl::range<1>(size), [=](cl::sycl::id<1> index) {
              fptr_ty fptr = reinterpret_cast<fptr_ty>(accDT[Mode]);
              accC[index] = fptr(accA[index], accB[index]);
            });
        });

      auto hostA = bufA.get_access<cl::sycl::access::mode::read>();
      auto hostB = bufB.get_access<cl::sycl::access::mode::read>();
      auto hostC = bufC.get_access<cl::sycl::access::mode::read>();

      if (Mode == 0) {
        for (size_t i=0; i<size; i++) passed &= hostA[i] + hostB[i] == hostC[i];
      } else {
        for (size_t i=0; i<size; i++) passed &= hostA[i] - hostB[i] == hostC[i];
      }
    } 
    return passed;
  }
};

int main()
{
  cl::sycl::device myDevice = cl::sycl::device(NEOGPUDeviceSelector());
  //cl::sycl::device myDevice = cl::sycl::device(CPUDeviceSelector());
  cl::sycl::queue myQueue = cl::sycl::queue(myDevice,exception_handler);
  cl::sycl::context myContext = myQueue.get_context();
  
  std::cout << "GPU Device: " << myDevice.get_info<cl::sycl::info::device::name>() << std::endl;
  int gpu_maxWorkGroupSize = myDevice.get_info<cl::sycl::info::device::max_work_group_size>();
  int gpu_maxComputeUnits  = myDevice.get_info<cl::sycl::info::device::max_compute_units>();    
  std::cout << "- Max Work Group Size : " << gpu_maxWorkGroupSize << std::endl;
  std::cout << "- Max Compute Units   : " << gpu_maxComputeUnits  << std::endl;

  /* create all tests */
  std::vector<std::unique_ptr<Test>> tests;
  tests.push_back(std::unique_ptr<Test>(new parallel_for_sycl_buffer_test()));
  tests.push_back(std::unique_ptr<Test>(new parallel_for_sycl_aligned_alloc_test()));
  tests.push_back(std::unique_ptr<Test>(new subgroup_test()));
  //tests.push_back(std::unique_ptr<Test>(new function_pointer_test()));
  tests.push_back(std::unique_ptr<Test>(new function_pointer_take_address_on_device_test()));
  tests.push_back(std::unique_ptr<Test>(new function_pointer_take_address_on_device_array_test()));

  /* invoke all tests */
  for (auto& test : tests)
    test->invoke(myDevice,myContext,myQueue);
 
  return 0;    
}
