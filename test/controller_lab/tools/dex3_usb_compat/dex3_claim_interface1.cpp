#include <dlfcn.h>
#include <libusb-1.0/libusb.h>

#include <atomic>
#include <cstdio>

namespace {

template <typename Function>
Function Resolve(const char* name) {
  dlerror();
  void* symbol = dlsym(RTLD_NEXT, name);
  const char* error = dlerror();
  if (error != nullptr || symbol == nullptr) {
    std::fprintf(stderr, "[dex3-usb-compat] cannot resolve %s: %s\n", name,
                 error != nullptr ? error : "unknown error");
    return nullptr;
  }
  return reinterpret_cast<Function>(symbol);
}

using ClaimInterface = int (*)(libusb_device_handle*, int);
using ReleaseInterface = int (*)(libusb_device_handle*, int);
using KernelDriverActive = int (*)(libusb_device_handle*, int);
using DetachKernelDriver = int (*)(libusb_device_handle*, int);
using BulkTransfer = int (*)(libusb_device_handle*, unsigned char,
                             unsigned char*, int, int*, unsigned int);

}  // namespace

extern "C" int libusb_claim_interface(libusb_device_handle* handle,
                                        int interface_number) {
  static ClaimInterface real_claim =
      Resolve<ClaimInterface>("libusb_claim_interface");
  if (real_claim == nullptr) {
    return LIBUSB_ERROR_OTHER;
  }

  const int primary_result = real_claim(handle, interface_number);
  if (primary_result != LIBUSB_SUCCESS || interface_number != 0) {
    return primary_result;
  }

  // dex3_service_pc4 2.1.0.1 claims interface 0, while the two bulk
  // endpoints (0x01 and 0x81) of the observed Dex3 devices belong to
  // interface 1. Claim interface 1 as well before the service starts I/O.
  static KernelDriverActive kernel_driver_active =
      Resolve<KernelDriverActive>("libusb_kernel_driver_active");
  static DetachKernelDriver detach_kernel_driver =
      Resolve<DetachKernelDriver>("libusb_detach_kernel_driver");
  if (kernel_driver_active != nullptr && detach_kernel_driver != nullptr &&
      kernel_driver_active(handle, 1) == 1) {
    const int detach_result = detach_kernel_driver(handle, 1);
    if (detach_result != LIBUSB_SUCCESS &&
        detach_result != LIBUSB_ERROR_NOT_FOUND) {
      std::fprintf(stderr,
                   "[dex3-usb-compat] cannot detach interface 1: %s\n",
                   libusb_error_name(detach_result));
    }
  }

  const int interface_one_result = real_claim(handle, 1);
  if (interface_one_result != LIBUSB_SUCCESS) {
    static ReleaseInterface real_release =
        Resolve<ReleaseInterface>("libusb_release_interface");
    if (real_release != nullptr) {
      real_release(handle, 0);
    }
    std::fprintf(stderr,
                 "[dex3-usb-compat] claim interface 1 failed: %s\n",
                 libusb_error_name(interface_one_result));
    return interface_one_result;
  }

  std::fprintf(stderr,
               "[dex3-usb-compat] claimed Dex3 USB interfaces 0 and 1\n");
  return LIBUSB_SUCCESS;
}

extern "C" int libusb_release_interface(libusb_device_handle* handle,
                                          int interface_number) {
  static ReleaseInterface real_release =
      Resolve<ReleaseInterface>("libusb_release_interface");
  if (real_release == nullptr) {
    return LIBUSB_ERROR_OTHER;
  }

  if (interface_number == 0) {
    const int interface_one_result = real_release(handle, 1);
    if (interface_one_result != LIBUSB_SUCCESS &&
        interface_one_result != LIBUSB_ERROR_NOT_FOUND &&
        interface_one_result != LIBUSB_ERROR_NO_DEVICE) {
      std::fprintf(stderr,
                   "[dex3-usb-compat] release interface 1 failed: %s\n",
                   libusb_error_name(interface_one_result));
    }
  }
  return real_release(handle, interface_number);
}

extern "C" int libusb_bulk_transfer(libusb_device_handle* handle,
                                      unsigned char endpoint,
                                      unsigned char* data, int length,
                                      int* transferred,
                                      unsigned int timeout) {
  static BulkTransfer real_bulk =
      Resolve<BulkTransfer>("libusb_bulk_transfer");
  if (real_bulk == nullptr) {
    return LIBUSB_ERROR_OTHER;
  }

  constexpr unsigned char kDex3InputEndpoint = 0x81;
  constexpr int kDex3StateFrameSize = 1270;
  if (endpoint != kDex3InputEndpoint || length != kDex3StateFrameSize ||
      data == nullptr || transferred == nullptr) {
    return real_bulk(handle, endpoint, data, length, transferred, timeout);
  }

  int total = 0;
  int reads = 0;
  int last_result = LIBUSB_SUCCESS;
  while (total < length && reads < 4) {
    int received = 0;
    const int result = real_bulk(handle, endpoint, data + total,
                                 length - total, &received, timeout);
    ++reads;
    last_result = result;
    if (received > 0) {
      total += received;
    }

    static std::atomic<unsigned int> partial_logs{0};
    if ((result != LIBUSB_SUCCESS || total < length) &&
        partial_logs.fetch_add(1, std::memory_order_relaxed) < 20) {
      std::fprintf(stderr,
                   "[dex3-usb-compat] state read %d: result=%s received=%d "
                   "total=%d/%d\n",
                   reads, libusb_error_name(result), received, total, length);
    }

    // libusb may report TIMEOUT and still return a packet-aligned prefix in
    // `transferred`. Keep that prefix and request only the missing suffix.
    // The vendor service still checks the complete frame header and CRC.
    if (result == LIBUSB_ERROR_TIMEOUT && received > 0 && total < length) {
      continue;
    }
    if (result != LIBUSB_SUCCESS) {
      *transferred = total;
      return result;
    }
    if (received <= 0) {
      break;
    }
  }

  *transferred = total;
  if (reads > 1 && total == length) {
    static std::atomic<unsigned int> reconstructed_logs{0};
    if (reconstructed_logs.fetch_add(1, std::memory_order_relaxed) < 10) {
      std::fprintf(stderr,
                   "[dex3-usb-compat] reconstructed 1270-byte state frame "
                   "from %d USB reads\n",
                   reads);
    }
  }
  return total == length ? LIBUSB_SUCCESS : last_result;
}
