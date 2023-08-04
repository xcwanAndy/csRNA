#ifndef __ACCEL_KFD_IOCTL_HH__
#define __ACCEL_KFD_IOCTL_HH__

#include <linux/types.h>
#include <linux/ioctl.h>
#include <stddef.h>
#include <stdint.h>

enum accel_side{
    SERVER_SIDE = 0,
    CLIENT_SIDE = 1
};

enum accel_cmd {
    ACCEL_NULL,
    ACCEL_SEND,
    ACCEL_RECV,
    ACCEL_WRITE,
    ACCEL_READ
};

struct accelkfd_ioctl_type {
    uint32_t type;
    uint32_t len;
};

struct accelkfd_ioctl_mr_addr {
    enum accel_side side;
    uint64_t vaddr;
    uint64_t paddr;
    size_t size;
};

struct kfd_ioctl_cmd {
    enum accel_side side;
    enum accel_cmd command;
};


#define ACCELKFD_IOCTL_BASE 'A'
#define ACCELKFD_IO(nr)			( _IO(ACCELKFD_IOCTL_BASE, nr)         )
#define ACCELKFD_IOR(nr, type)		( _IOR(ACCELKFD_IOCTL_BASE, nr, type)  )
#define ACCELKFD_IOW(nr, type)		( _IOW(ACCELKFD_IOCTL_BASE, nr, type)  )
#define ACCELKFD_IOWR(nr, type)    ( _IOWR(ACCELKFD_IOCTL_BASE, nr, type) )

#define ACCELKFD_SEND_MR_ADDR		\
		ACCELKFD_IOW(0x01, struct accelkfd_ioctl_mr_addr)

#define ACCELKFD_SEND_CMD		\
		ACCELKFD_IOW(0x02, struct accelkfd_ioctl_cmd)

#define ACCELKFD_START_CLT		\
		ACCELKFD_IOW(0x03, NULL)

#define ACCELKFD_START_SVR		\
		ACCELKFD_IOW(0x04, NULL)

#endif /*__ACCEL_KFD_IOCTL_HH__*/
