#pragma once



/// Starting device DMA address
#define VFIO_IOVA           0x0

/// Adjust to 4K page aligned size
#define VFIO_PASIZE(n)      (((n) + 0xfff) & ~0xfff)


#define MAX_MSIX_VECTOR_NUM                 1
#define MAX_IO_Q_PAIR_NUM               32


static const char* region_name[VFIO_PCI_NUM_REGIONS] = {"BAR0", "BAR1", "BAR2", "BAR3", "BAR4", "BAR5", "ROM", "CONFIG", "VGA"};
static const char* irq_name[VFIO_PCI_NUM_IRQS] = {"INTX", "MSI", "MISX", "ERR", "REQ"};


typedef struct _vfio_dma {
    u8*                     user_buf;        ///< memory buffer
    size_t                  size;       ///< allocated size
    u64                     dma_addr;       ///< I/O DMA address
    void*                   id;         ///< private id
} vfio_dma_t;



typedef struct _vfio_mem {
    struct _nvme_dev*       dev;        ///< device ownder
    int                     mmap;       ///< mmap indication flag
    vfio_dma_t              dma;        ///< dma mapped memory
    struct _vfio_mem*       prev;       ///< previous entry
    struct _vfio_mem*       next;       ///< next entry
} vfio_mem_t;



typedef struct _nvme_queue_pair {
    struct _nvme_dev*               dev;        ///< device reference
    int                             id;         ///< queue id
    //int                           size;       ///< queue size
    
    int                             sq_size;       ///< queue size
    int                             cq_size;       ///< queue size
    

    nvme_sq_entry_t*                sq;         
    nvme_cq_entry_t*                cq;         
    u32*                            sq_doorbell; 
    u32*                            cq_doorbell; 
    int                             sq_tail = 0;   
    int                             cq_head = 0;   
    int                             cq_phase =1;   
    pthread_mutex_t                 sq_lock;
    pthread_mutex_t                 cq_lock;


    pthread_mutex_t sync_cmd_mutex;
    pthread_cond_t sync_cmd_cond;


} nvme_queue_pair_t;


typedef struct _nvme_dev {
    int                             pci;        ///< PCI device id
    int                             fd;         ///< device descriptor
    int                             groupfd;    ///< group file descriptor
    int                             contfd;     ///< container file descriptor
    int                             msix_size;  ///< max MSIX table size
    int                             msix_nvec;  ///< number of enabled MSIX vectors

    int                             efd;                        // event fd (for INTx, MSI)
    int                             efds[MAX_MSIX_VECTOR_NUM];  // event fd (for MSI-x)
    int                             epfd;                       // epoll fd

    nvme_controller_reg_t*          ctrl_reg = nullptr;

    vfio_dma_t*                     admin_sq = nullptr;
    vfio_dma_t*                     admin_cq = nullptr;
    nvme_queue_pair_t               admin_q_pair;

    nvme_queue_pair_t               io_q_pair[MAX_IO_Q_PAIR_NUM];
    int                             current_io_q_pair_num = 0;

    struct vfio_device_info         device_info;
    struct vfio_group_status        group_status;
    struct vfio_iommu_type1_info    iommu_info;
    struct vfio_region_info         regs[VFIO_PCI_NUM_REGIONS];
    struct vfio_irq_info            irqs[VFIO_PCI_NUM_IRQS];

    u64                             iova;       ///< next DMA (virtual IO) address
    vfio_mem_t*                     memlist;    ///< memory allocated list
    pthread_spinlock_t              lock;       ///< multithreaded lock
} nvme_dev_t;



bool map_bar0(nvme_dev_t * dev);
void unmap_bar0(nvme_dev_t* dev);


vfio_dma_t* dma_alloc(nvme_dev_t* vdev, size_t size);
vfio_dma_t* dma_aligned_alloc(nvme_dev_t* vdev, size_t size, size_t aligned_size);
int dma_free(vfio_dma_t* dma);


nvme_dev_t* create_instance(int segn, int busn, int devn, int funcn);
void delete_instance(nvme_dev_t* vdev);

uintptr_t virt_to_phys(void* virt);

void enable_bus_master( nvme_dev_t* dev) ;
void disable_bus_master(nvme_dev_t* dev);

void check_cq(nvme_queue_pair_t* q);

void nvme_submit_cmd(nvme_queue_pair_t* q);

int nvme_ctrl_disable(nvme_dev_t* dev);

int nvme_ctrl_enable(nvme_dev_t* dev, nvme_controller_config_t *cc);

/**
 * @fn
 * @brief 　nvmeコントローラを初期化する
 * @param (引数名) 引数の説明
 * @param (引数名) 引数の説明
 * @return 戻り値の説明

 */
int init_device(nvme_dev_t* dev, size_t sq_sz, size_t cq_sz);






nvme_dev_t* create_instance(int segn, int busn, int devn, int funcn);


void delete_instance(nvme_dev_t* vdev);