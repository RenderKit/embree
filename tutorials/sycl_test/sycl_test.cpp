
#include <CL/sycl.hpp>
#include <iostream>

 class NEOGPUDeviceSelector : public cl::sycl::device_selector {
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

// === create exception handler ===
  
/*auto exception_handler = [] (cl::sycl::exception_list exceptions) {
    for (std::exception_ptr const& e : exceptions) {
      try {
	std::rethrow_exception(e);
      } catch(cl::sycl::exception const& e) {
	std::cout << "Caught asynchronous SYCL exception:\n" << e.what() << std::endl;
	exit(1);
      }
    }
  };
*/


int main()
{
  std::vector<int> h_a(1000);
  std::vector<int> h_b(1000);
  std::vector<int> h_c(1000);

  std::cout << "A" << std::endl;

  for (int i=0; i<1000; i++) {
    h_a[i] = i;
    h_b[i] = i+5;
    h_c[i] = 0;
  }

  {
    std::cout << "A1" << std::endl;
  
    cl::sycl::buffer<int> d_a(h_a);
    cl::sycl::buffer<int> d_b(h_b);
    cl::sycl::buffer<int> d_c(h_c);
    std::cout << "A2" << std::endl;

    cl::sycl::device myDevice = cl::sycl::device(NEOGPUDeviceSelector());
    cl::sycl::queue myQueue = cl::sycl::queue(myDevice);

    std::cout << "GPU Device: " << myDevice.get_info<cl::sycl::info::device::name>() << std::endl;
    int gpu_maxWorkGroupSize = myDevice.get_info<cl::sycl::info::device::max_work_group_size>();
    int gpu_maxComputeUnits  = myDevice.get_info<cl::sycl::info::device::max_compute_units>();    
    std::cout << "- Max Work Group Size : " << gpu_maxWorkGroupSize << std::endl;
    std::cout << "- Max Compute Units   : " << gpu_maxComputeUnits  << std::endl;

    std::cout << "B" << std::endl;

    myQueue.submit([&](cl::sycl::handler& cgh) {

        cl::sycl::stream out(1000, 1000, cgh);
         
        std::cout << "C" << std::endl;
        
        //cl::sycl::accessor<int, 0, cl::sycl::access::mode::read, cl::sycl::access::target::local> a;
        auto a = d_a.get_access<cl::sycl::access::mode::read>();
        std::cout << "D" << std::endl;
        auto b = d_b.get_access<cl::sycl::access::mode::read>();
        std::cout << "E" << std::endl;
        auto c = d_c.get_access<cl::sycl::access::mode::write>();

        std::cout << "F" << std::endl;

        /*cgh.parallel_for<class test>(cl::sycl::range<1>(1000), [=](cl::sycl::id<1> item) {
            int i = item.get_id(0);
            c[i] = a[i] + b[i];
            });*/

        cgh.single_task<class name>([=] () {
            c[0] = a[0] + b[0];
            out << "gpu: c[0] = " << c[0] << cl::sycl::endl;
          });
       });
    
     myQueue.wait_and_throw();
  }

  std::cout << "h_a[0] = " << h_a[0] << std::endl;
  std::cout << "h_b[0] = " << h_b[0] << std::endl;
  std::cout << "h_c[0] = " << h_c[0] << std::endl;

  for (int i=0; i<1000; i++) {
    assert(h_a[i]+h_b[i] == h_c[i]);
  }

  return 0;    
}
