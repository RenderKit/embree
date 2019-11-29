
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
      std::cout << "Caught asynchronous SYCL exception:\n" << e.what() << std::endl;
      exit(1);
    }
  }
};


struct Test
{
  Test (std::string name)
    : name(name) {}

  virtual ~Test() {}

  virtual bool run(cl::sycl::queue& myQueue) = 0;

  void invoke(cl::sycl::queue& myQueue)
  {
    std::cout << name << ": " << std::flush;
    bool passed = run(myQueue);
    if (passed) std::cout << " [PASSED]" << std::endl;
    else        std::cout << " [FAILED]" << std::endl;
  }
  
  std::string name;
};

struct parallel_for_sycl_buffer_test : public Test
{
  parallel_for_sycl_buffer_test ()
    : Test("parallel_for_sycl_buffer_test") {}
  
  bool run (cl::sycl::queue& myQueue)
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
    }
    
    for (int i=0; i<1000; i++)
    {
      if (h_a[i]+h_b[i] != h_c[i])
        return false;
    }
    
    return true;
  }
};
    
int main()
{
  cl::sycl::device myDevice = cl::sycl::device(NEOGPUDeviceSelector());
  //cl::sycl::device myDevice = cl::sycl::device(CPUDeviceSelector());
  cl::sycl::queue myQueue = cl::sycl::queue(myDevice,exception_handler);
  
  std::cout << "GPU Device: " << myDevice.get_info<cl::sycl::info::device::name>() << std::endl;
  int gpu_maxWorkGroupSize = myDevice.get_info<cl::sycl::info::device::max_work_group_size>();
  int gpu_maxComputeUnits  = myDevice.get_info<cl::sycl::info::device::max_compute_units>();    
  std::cout << "- Max Work Group Size : " << gpu_maxWorkGroupSize << std::endl;
  std::cout << "- Max Compute Units   : " << gpu_maxComputeUnits  << std::endl;

  std::vector<std::unique_ptr<Test>> tests;
  tests.push_back(std::unique_ptr<Test>(new parallel_for_sycl_buffer_test()));

  for (auto& test : tests)
    test->invoke(myQueue);
 
  return 0;    
}
