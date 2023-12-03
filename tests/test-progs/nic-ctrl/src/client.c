#include "common.h"

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
    struct accelkfd_ioctl_mr_addr *args = set_mr_addr_args(CLIENT_SIDE, 1024, 0);
    ioctl(accel_fd, ACCELKFD_SEND_MR_ADDR, (void *)args);


    /* Set offpath memory region */
    uint8_t *offpath_addr = (uint8_t *)memalign(1<<12, 1 << 12);
    memset(offpath_addr, 0, 1 << 12);
    printf("[DEBUG] allocated offpath addr %p\n", offpath_addr);

    struct kfd_ioctl_set_mem_map *mem_map_args =
        (struct kfd_ioctl_set_mem_map *)malloc(sizeof(struct kfd_ioctl_set_mem_map));
    mem_map_args->size = 1 << 12;
    mem_map_args->host_mem = (uint64_t)offpath_addr;
    fill_read_mr(offpath_addr, 0, DATA_8B, sizeof(DATA_8B));
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
