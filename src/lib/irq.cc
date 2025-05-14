#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <linux/vfio.h>

#include "nvme.h"
#include "vfio.h"
#include "irq.h"

int enable_msix(nvme_dev_t* dev) {
    printf("Use MSI-X interrupt\n");
    struct vfio_irq_set* irq_set;
    char irq_set_buf[sizeof(struct vfio_irq_set) + sizeof(int) * MAX_MSIX_VECTOR_NUM];
    irq_set = (struct vfio_irq_set*)irq_set_buf;
    irq_set->argsz = sizeof(irq_set_buf);
    irq_set->count = MAX_MSIX_VECTOR_NUM;
    irq_set->flags = VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_TRIGGER;
    irq_set->index = VFIO_PCI_MSIX_IRQ_INDEX;
    irq_set->start = 0;

    for (int i = 0; i < MAX_MSIX_VECTOR_NUM; i++) {
        dev->efds[i] = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
        if(dev->efds[i] < 0){
            printf("efd init failed\n");
            return -1;
        }
    }
    memcpy((int*)&irq_set->data, dev->efds, sizeof(dev->efds));
    int ret = ioctl(dev->fd, VFIO_DEVICE_SET_IRQS, irq_set);

    if(ret != 0) {
        printf("faield to enable MSI-X interrupt\n");
        return -1;    
    }
    return 0;
}

int disable_msix(nvme_dev_t* vdev)
{
    nvme_dev_t* dev = (nvme_dev_t*)vdev;
    if (dev->msix_nvec == 0) return 0;

    struct vfio_irq_set irq_set = {
        .argsz = sizeof(irq_set),
        .flags = VFIO_IRQ_SET_DATA_NONE | VFIO_IRQ_SET_ACTION_TRIGGER,
        .index = VFIO_PCI_MSIX_IRQ_INDEX,
        .start = 0,
        .count = 0,
    };
    if (ioctl(dev->fd, VFIO_DEVICE_SET_IRQS, &irq_set)) {
        printf("Unable to disable MSI-X interrupt\n");
        return -1;
    }

    dev->msix_nvec = 0;
    return 0;
}

static int unmask_intx(nvme_dev_t* dev) {
    struct vfio_irq_set irq_set = {0};
    irq_set.argsz = sizeof(struct vfio_irq_set);
    irq_set.count = 1;
    irq_set.flags = VFIO_IRQ_SET_DATA_NONE | VFIO_IRQ_SET_ACTION_UNMASK;
    irq_set.index = VFIO_PCI_INTX_IRQ_INDEX;
    irq_set.start = 0;

    int ret = ioctl(dev->fd, VFIO_DEVICE_SET_IRQS, &irq_set);
    if(ret != 0){
        printf("faield to unmask INTx interrupt\n");
        return -1;
    }
    return 0;
    
}

int enable_intx(nvme_dev_t* dev) {
    printf("Use INTx interrupt\n");
    struct vfio_irq_set* irq_set;
    char irq_set_buf[sizeof(struct vfio_irq_set) + sizeof(int)];
    irq_set = (struct vfio_irq_set*)irq_set_buf;
    irq_set->argsz = sizeof(irq_set_buf);
    irq_set->count = 1;
    irq_set->flags = VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_TRIGGER;
    irq_set->index = VFIO_PCI_INTX_IRQ_INDEX;
    irq_set->start = 0;
    dev->efd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    printf("efd = %d\n", dev->efd);
    if(dev->efd < 0){
        printf("efd init failed\n");
        return -1;
    }
    
    *(int*)&irq_set->data = dev->efd;
    int ret = ioctl(dev->fd, VFIO_DEVICE_SET_IRQS, irq_set);
    if(ret != 0){
        printf("faield to enable INTx interrupt\n");
        return -1;
    }
    
    return unmask_intx(dev);
}

int disable_intx(nvme_dev_t* dev){

    printf("disable INTx interrupt\n");
    struct vfio_irq_set* irq_set;
    char irq_set_buf[sizeof(struct vfio_irq_set) + sizeof(int)];

    irq_set = (struct vfio_irq_set*)irq_set_buf;
    irq_set->argsz = sizeof(irq_set_buf);
    irq_set->count = 1;
    irq_set->flags = VFIO_IRQ_SET_DATA_NONE | VFIO_IRQ_SET_ACTION_MASK;
    irq_set->index = VFIO_PCI_INTX_IRQ_INDEX;
    irq_set->start = 0;
  
    //*(int*)&irq_set->data = dev->efd;
    int ret = ioctl(dev->fd, VFIO_DEVICE_SET_IRQS, irq_set);
    if(ret != 0){
        printf("Error masking INTx interrupts \n");
        return -1;
    }

	/* disable INTx*/
	memset(irq_set, 0, sizeof(struct vfio_irq_set));
    irq_set->argsz = sizeof(irq_set_buf);
	irq_set->count = 0;
	irq_set->flags = VFIO_IRQ_SET_DATA_NONE | VFIO_IRQ_SET_ACTION_TRIGGER;
	irq_set->index = VFIO_PCI_INTX_IRQ_INDEX;
	irq_set->start = 0;

	ret = ioctl(dev->fd, VFIO_DEVICE_SET_IRQS, irq_set);
    if(ret != 0){
        printf("Error disabling INTx interrupts\n");
        return -1;
    }

    return 0;
}

void* interrupt_hadler(void* p) {
    // Create epoll fd
    nvme_dev_t* dev = (nvme_dev_t*)p;

    dev->epfd = epoll_create1(EPOLL_CLOEXEC);
    if(dev->epfd < 0){
        printf( "failed to create epoll fd\n");
        exit(1);
    }
   
    // Add eventfd to epoll

#if defined(MSIX)
    for (int i = 0; i < MAX_MSIX_VECTOR_NUM; i++) {
        struct epoll_event ev = {0}; //{.events = EPOLLIN | EPOLLPRI,
                                 //.data.fd = dev->efds[i]};
        ev.events = EPOLLIN | EPOLLPRI;
        ev.data.fd = dev->efds[i];
        int ret = epoll_ctl(dev->epfd, EPOLL_CTL_ADD, dev->efds[i], &ev);
        if(ret != 0){
            printf("cannot add fd to epoll\n");
            exit(1);
        }
    }

#elif defined(INTx)
    struct epoll_event ev = {0}; 
    ev.events = EPOLLIN | EPOLLPRI;
    ev.data.fd = dev->efd;
    int ret = epoll_ctl(dev->epfd, EPOLL_CTL_ADD, dev->efd, &ev);
    if(ret != 0){
        printf("cannot add fd to epoll\n");
        exit(1);
    }

#endif

    struct epoll_event evs;
    u64 u;

    for (;;) {
        printf("waiting interrupts...\n");
        // blocking wait
        int rc = epoll_wait(dev->epfd, &evs, 1, -1);
        if(rc <= 0){
            printf("epoll error\n");
            exit(1);
        }

        ssize_t s = read(evs.data.fd, &u, sizeof(u));
        if(s != sizeof(u)){
            printf("efd read failed\n");
        }

#if defined(MSIX)
        for(int i=0; i<MAX_MSIX_VECTOR_NUM; ++i){
            if (evs.data.fd == dev->efds[i]) {
                check_cq(&dev ->q_pair[i]);
                if(i == 0){
                    printf("MSIx vector 0 interrupt ocuured\n");           
                }else{
                    printf("MSIx vector %d interrupt ocuured\n", i);
                }
                break;
            }
        }

#elif defined(INTx)
        printf("INTx occured\n");
        check_cq(&dev ->q_pair[0]);

        for(int i=1; i< dev->current_io_q_pair_num + 1; ++i) check_cq(&dev ->q_pair[i]);
        
        if(unmask_intx(dev) != 0){
            printf("Error: umask intx\n");
        }

#endif

    }
}