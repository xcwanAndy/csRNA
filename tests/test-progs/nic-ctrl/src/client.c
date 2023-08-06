#include "../../../../src/dev/xdr/kfd_ioctl.hh"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdint.h>
#include <string.h>
#include <malloc.h>

#define KERNEL_FILE_NAME "/dev/accel"

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

int main() {
    unsigned long mem_len = 1024;
    struct accelkfd_ioctl_mr_addr *args =
        (struct accelkfd_ioctl_mr_addr *) malloc(sizeof(struct accelkfd_ioctl_mr_addr));

    int fd = open(KERNEL_FILE_NAME, O_RDWR);
    printf("[DEBUG] open /dev/accel success!\n");

    uint8_t *ctrl_addr = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    printf("[DEBUG] get ctrl_addr 0x%lx\n", (uint64_t)ctrl_addr);

    uint8_t *mr_addr = (uint8_t *)memalign(1<<12, mem_len);
    memset(mr_addr, 0, mem_len);
    printf("[DEBUG] allocated addr: %p\n", mr_addr);
    args->side = CLIENT_SIDE;
    args->vaddr = (uint64_t)mr_addr;
    args->paddr = 0;
    args->size = mem_len;

    fill_read_mr(mr_addr, 0);

    /*memcpy(ctrl_addr, args, sizeof(struct kfd_ioctl_mr_addr));*/

    ioctl(fd, ACCELKFD_SEND_MR_ADDR, (void *)args);

    ioctl(fd, ACCELKFD_START_CLT, NULL);

    int flag = 0;
    while (1) {
        sleep(10);
        if (flag && *mr_addr != 0 ) {
            fprintf(stderr, "The string in mr_addr: %s", (char*)mr_addr);
            flag = 0;
        }
    }

    return 0;
}
