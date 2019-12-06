
#include <CL/sycl.hpp>
#include <iostream>

class NEOGPUDeviceSelector : public cl::sycl::device_selector
{
public:
  int operator()(const cl::sycl::device& device) const override {
    using namespace cl::sycl::info;
    
    const std::string deviceName = device.get_info<device::name>();
    const std::string deviceVendor = device.get_info<device::vendor>();      
    return device.is_gpu() && deviceName.find("HD Graphics NEO") ? 1 : -1;
  }
};

class CPUDeviceSelector : public cl::sycl::device_selector {
  public:
  int operator()(const cl::sycl::device& device) const override {
    return device.is_cpu() ? 1 : -1;
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

  virtual bool run(cl::sycl::device& device, cl::sycl::context context, cl::sycl::queue& queue) = 0;

  void invoke(cl::sycl::device& device, cl::sycl::context context, cl::sycl::queue& queue)
  {
    std::cout << name << ": " << std::flush;
    bool passed = run(device,context,queue);
    if (passed) std::cout << " [PASSED]" << std::endl;
    else        std::cout << " [FAILED]" << std::endl;
    std::cout << std::flush;
  }
  
  std::string name;
};

struct access_virtual_structure_test : public Test
{
  static const int size = 1000;

  struct OtherVirtualStructure {
    virtual void add() {}
  };

  struct VirtualStructure
  {
    VirtualStructure () = default;
    
    VirtualStructure (int value)
      : value(value) {}

    static VirtualStructure rand() {
      return VirtualStructure(std::rand());
    }

    virtual void add() {}
    
    int value;
    OtherVirtualStructure* ptr;
  };

  access_virtual_structure_test ()
    : Test("access_virtual_structure_test") {}
  
  bool run (cl::sycl::device& device, cl::sycl::context context, cl::sycl::queue& queue)
  {
    std::vector<VirtualStructure> A(size);
    std::vector<VirtualStructure> B(size);
    std::vector<VirtualStructure> C(size);

    std::generate(A.begin(), A.end(), VirtualStructure::rand);
    std::generate(B.begin(), B.end(), VirtualStructure::rand);
    std::generate(C.begin(), C.end(), VirtualStructure::rand);

    cl::sycl::buffer<VirtualStructure> bufA(A);
    cl::sycl::buffer<VirtualStructure> bufB(B);
    cl::sycl::buffer<VirtualStructure> bufC(C);
    
    queue.submit([&](cl::sycl::handler& cgh) {
        
        auto a = bufA.get_access<cl::sycl::access::mode::read>(cgh);
        auto b = bufB.get_access<cl::sycl::access::mode::read>(cgh);
        auto c = bufC.get_access<cl::sycl::access::mode::write>(cgh);
        
        cgh.parallel_for<class test>(cl::sycl::range<1>(size), [=](cl::sycl::id<1> item) {
            c[item].value = a[item].value + b[item].value;
          });
      });
    
    auto hostA = bufA.get_access<cl::sycl::access::mode::read>();
    auto hostB = bufB.get_access<cl::sycl::access::mode::read>();
    auto hostC = bufC.get_access<cl::sycl::access::mode::read>();
    
    for (int i=0; i<size; i++)
    {
      if (hostA[i].value+hostB[i].value != hostC[i].value)
        return false;
    }
    
    return true;
  }
};

int main()
{
  cl::sycl::device device = cl::sycl::device(NEOGPUDeviceSelector());
  //cl::sycl::device device = cl::sycl::device(CPUDeviceSelector());
  cl::sycl::queue queue = cl::sycl::queue(device,exception_handler);
  cl::sycl::context context = queue.get_context();
  
  std::cout << "GPU Device: " << device.get_info<cl::sycl::info::device::name>() << std::endl;
  int gpu_maxWorkGroupSize = device.get_info<cl::sycl::info::device::max_work_group_size>();
  int gpu_maxComputeUnits  = device.get_info<cl::sycl::info::device::max_compute_units>();    
  std::cout << "- Max Work Group Size : " << gpu_maxWorkGroupSize << std::endl;
  std::cout << "- Max Compute Units   : " << gpu_maxComputeUnits  << std::endl;

  /* create all tests */
  std::vector<std::unique_ptr<Test>> tests;
  tests.push_back(std::unique_ptr<Test>(new access_virtual_structure_test()));
   
  /* invoke all tests */
  for (auto& test : tests)
    test->invoke(device,context,queue);
 
  return 0;    
}
