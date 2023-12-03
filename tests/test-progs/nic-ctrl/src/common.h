#ifndef __TEST_COMMON__

#include "../../../../src/dev/xdr/kfd_ioctl.hh"
#include "../../../../src/dev/rdma/kfd_ioctl.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdint.h>
#include <string.h>
#include <malloc.h>

#define ACCEL_FILE_NAME "/dev/accel"
#define RNIC_FILE_NAME "/dev/hangu_rnic0"
#define TRANS_RRDMA_DATA "hello RDMA Read!"

//int a = sizeof(TRANS_RRDMA_DATA);

#define DATA_8B "8 bytes"
#define DATA_64B "==========================64 bytes============================="

uint32_t fill_read_mr(uint8_t *addr, uint32_t offset, char *payload, size_t payload_len);
struct accelkfd_ioctl_mr_addr *set_mr_addr_args(uint8_t side, size_t len, int is_fill);

#define __TEST_COMMON__
#endif
