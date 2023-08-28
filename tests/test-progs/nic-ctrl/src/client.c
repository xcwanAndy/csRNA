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

uint32_t fill_read_mr(uint8_t *addr, uint32_t offset) {

#define TRANS_RRDMA_DATA "hello RDMA Read!"

    // Write data to mr
    char *string = (char *)(addr + offset);
    memcpy(string, TRANS_RRDMA_DATA, sizeof(TRANS_RRDMA_DATA));

    fprintf(stderr, "init_snd_wqe: string is %s, string vaddr is 0x%lx, start vaddr is 0x%lx\n",
            string, (uint64_t)string, (uint64_t)(addr + offset));

    offset += sizeof(TRANS_RRDMA_DATA);
    return offset;
}

struct accelkfd_ioctl_mr_addr * set_mr_addr_args() {
    unsigned long mem_len = 1024;
    struct accelkfd_ioctl_mr_addr *args =
        (struct accelkfd_ioctl_mr_addr *) malloc(sizeof(struct accelkfd_ioctl_mr_addr));

    uint8_t *mr_addr = (uint8_t *)memalign(1<<12, mem_len);
    memset(mr_addr, 0, mem_len);
    printf("[DEBUG] allocated addr: %p\n", mr_addr);
    args->side = CLIENT_SIDE;
    args->vaddr = (uint64_t)mr_addr;
    args->paddr = 0;
    args->size = mem_len;

    fill_read_mr(mr_addr, 0);

    return args;
}

int main() {
    /* Open and mmap accelerater */
    int accel_fd = open(ACCEL_FILE_NAME, O_RDWR);
    printf("[DEBUG] open /dev/accel success!\n");
    uint8_t *ctrl_addr = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, accel_fd, 0);
    printf("[DEBUG] get ctrl_addr 0x%lx\n", (uint64_t)ctrl_addr);


    /* Open and mmap rnic */
    int rnic_fd = open(RNIC_FILE_NAME, O_RDWR);
    printf("[DEBUG] open /dev/hangu_rnic success!\n");
    uint8_t *hcr_addr = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, rnic_fd, 0);
    printf("[DEBUG] get hcr_addr 0x%lx\n", (uint64_t)hcr_addr);


    /* Set memory region args for accelerater */
    struct accelkfd_ioctl_mr_addr *args = set_mr_addr_args();
    ioctl(accel_fd, ACCELKFD_SEND_MR_ADDR, (void *)args);


    /* Set offpath memory region */
    uint8_t *offpath_addr = (uint8_t *)memalign(1<<12, 1 << 12);
    memset(offpath_addr, 0, 1 << 12);
    printf("[DEBUG] allocated offpath addr %p\n", offpath_addr);

    struct kfd_ioctl_set_mem_map *mem_map_args =
        (struct kfd_ioctl_set_mem_map *)malloc(sizeof(struct kfd_ioctl_set_mem_map));
    mem_map_args->size = 1 << 12;
    mem_map_args->host_mem = (uint64_t)offpath_addr;
    fill_read_mr(offpath_addr, 0);
    ioctl(rnic_fd, HGKFD_IOC_SET_MEM_MAP, (void*)mem_map_args);

    /* Start client */
    ioctl(accel_fd, ACCELKFD_START_CLT, NULL);

    int flag = 0;
    while (1) {
        sleep(10);
        /*if (flag && *(char *)args->vaddr != 0 ) {*/
            /*fprintf(stderr, "The string in mr_addr: '%s'\n", (char*)mem_map_args->host_mem);*/
            /*flag = 0;*/
        /*}*/
    }

    return 0;
}
