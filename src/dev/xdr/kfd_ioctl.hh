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


struct kfd_ioctl_mr_addr {
    enum accel_side side;
    uint64_t addr;
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
		ACCELKFD_IOW(0x01, struct kfd_ioctl_mr_addr)

#define ACCELKFD_SEND_CMD		\
		ACCELKFD_IOW(0x02, struct kfd_ioctl_cmd)


#endif /*__ACCEL_KFD_IOCTL_HH__*/
