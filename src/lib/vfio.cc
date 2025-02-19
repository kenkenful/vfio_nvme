#include "vfio.h"


uintptr_t virt_to_phys(void* virt) {
    long pagesize = sysconf(_SC_PAGESIZE);
    int fd = open("/proc/self/pagemap", O_RDONLY);

    if(fd == -1){
        printf("failed to open /proc/self/pagemap\n");
        return (uintptr_t)0;
    }

   // ASSERT(fd != -1, "failed to open /proc/self/pagemap");
    off_t ret = lseek(fd, (uintptr_t)virt / pagesize * sizeof(uintptr_t), SEEK_SET);
    
    if(ret == -1){
        printf("lseek error\n");
        return (uintptr_t)0;
    }

    uintptr_t entry = 0;
    ssize_t rc = read(fd, &entry, sizeof(entry));
    if(rc <= 0){
        printf("read error\n");
        return (uintptr_t)0;
    }

    if(entry == 0){
        printf("failed to get physical address for %p (perhaps forgot sudo?)", virt);
        return (uintptr_t)0;
    }

    //ASSERT(rc > 0, "read error");
    //ASSERT(entry != 0,"failed to get physical address for %p (perhaps forgot sudo?)",virt);
    close(fd);

    return (entry & 0x7fffffffffffffULL) * pagesize + ((uintptr_t)virt) % pagesize;
}

static inline void* zalloc(int size)
{
    void* mem = calloc(1, size);
    if (!mem) {
        printf("Error: calloc\n");
        exit(1);
    }
    return mem;
}

#if 0
#define PHYSADDR_MAP

static u64 get_iova(u64 virt_addr, ssize_t size) {
    static u64 _iova = VFIO_IOVA;
#if defined(IDENTITY_MAP)
    // Use virtual address as IOVA
    // Note that some architecture only support 3-level page table (39-bit) and
    // cannot use virtual address as IOVA
    return virt_addr;
#elif defined(PHYSADDR_MAP)
    // Use physical address as IOVA
    return (u64)virt_to_phys(reinterpret_cast<void*>(virt_addr));
#else
    // Assign IOVA from 0
    u64 ret = _iova;
    _iova += size;
    return ret;
#endif
}
#endif

 bool map_bar0(nvme_dev_t * dev){
    struct vfio_region_info* bar0_info = &dev->regs[VFIO_PCI_BAR0_REGION_INDEX];
    if(dev -> ctrl_reg != nullptr){
        printf("Error: controller reg is already mapped\n");
        return false;
    }
    dev->ctrl_reg = (nvme_controller_reg_t*)mmap(NULL, bar0_info->size, PROT_READ | PROT_WRITE, MAP_SHARED, dev->fd, bar0_info->offset);
    if(dev->ctrl_reg == MAP_FAILED){
        return false;
    }
    return true;
}


void unmap_bar0(nvme_dev_t* dev){
    struct vfio_region_info* bar0_info = &dev->regs[VFIO_PCI_BAR0_REGION_INDEX];
    if(dev -> ctrl_reg != nullptr){
        munmap(dev -> ctrl_reg, bar0_info->size);
    }
    dev -> ctrl_reg = nullptr;
}


void enable_bus_master( nvme_dev_t* dev) {
    struct vfio_region_info* cs_info = &dev->regs[VFIO_PCI_CONFIG_REGION_INDEX];
    char buf[2];
    pread(dev->fd, buf, 2, cs_info->offset + 4);
    *(u16*)(buf) |= 1 << 2;
    pwrite(dev->fd, buf, 2, cs_info->offset + 4);
    printf("PCI configuration space command reg = %04X\n", *(u16*)buf);
}

void disable_bus_master(nvme_dev_t* dev){
    struct vfio_region_info* cs_info = &dev->regs[VFIO_PCI_CONFIG_REGION_INDEX];
    char buf[2];
    pread(dev->fd, buf, 2, cs_info->offset + 4);
    *(u16*)(buf) &= ~((1) << (2));
    pwrite(dev->fd, buf, 2, cs_info->offset + 4);
    printf("PCI configuration space command reg = %04X\n", *(u16*)buf);

}


void check_cq(nvme_queue_pair_t* q){
    printf("QID: %d\n", q ->id);

    printf("%s, %s, %d\n", __FILE__, __func__, __LINE__);

    pthread_mutex_lock(&q->cq_lock);
    printf("path1\n");

    printf("%d\n" , q->cq[q -> cq_head].p);
    printf("%d\n" , q -> cq_phase);


    if(q->cq[q -> cq_head].p == q -> cq_phase){
        printf("path2\n");
        while(q->cq[q -> cq_head].p == q -> cq_phase){
            int head = q -> cq_head;
            printf("sqid: %d, cid: %d, sc: %d, sct: %d \n", q->cq[head].sqid, q->cq[head].cid, q->cq[head].sc, q->cq[head].sct);
            if(++q ->cq_head == q-> cq_size){
                q -> cq_head = 0;
                q -> cq_phase = !q->cq_phase;
            }

#if defined(MSIX) || defined(INTx)
            pthread_mutex_lock(&q->sync_mutex);
            pthread_cond_signal(&q->sync_cond);
            pthread_mutex_unlock(&q->sync_mutex);
#endif

        }

        *(volatile u32*)(q->cq_doorbell) = q -> cq_head; 
    }

    pthread_mutex_unlock(&q->cq_lock);
        printf("%s, %s, %d\n", __FILE__, __func__, __LINE__);

}


void nvme_check_completion(nvme_queue_pair_t *q){
    nvme_cq_entry_t *cqe;

    pthread_mutex_lock(&q->cq_lock);

    for(;;){
        cqe = &q->cq[q->cq_head];
        if(cqe ->p != q->cq_phase) break;

        if(++q ->cq_head == q->cq_size){
            q->cq_head = 0;
            q->cq_phase = !q->cq_phase;

        }
        *(volatile u32*)(q->cq_doorbell) = q -> cq_head; 

        printf("sqid: %x, cid: %x, sc: %x, sct: %x\n", cqe ->sqid, cqe->cid, cqe->sc, cqe->sct);

#if defined(MSIX) || defined(INTx)
        pthread_mutex_lock(&q->sync_mutex);
        pthread_cond_signal(&q->sync_cond);
        pthread_mutex_unlock(&q->sync_mutex);
#endif

    }
    pthread_mutex_unlock(&q->cq_lock);

}



void nvme_submit_cmd(nvme_queue_pair_t* q)
{
    pthread_mutex_lock(&q->sq_lock);
    if (++q->sq_tail == q->sq_size) q->sq_tail = 0;

    *(volatile u32*)q->sq_doorbell = q->sq_tail;
    pthread_mutex_unlock(&q->sq_lock);
}




int nvme_ctrl_wait_ready(nvme_dev_t *dev, int ready){

    nvme_controller_cap_t cap;
    cap.val = dev ->ctrl_reg ->cap.val;
    int timeout = cap.to;

    for(int i = 0; i<timeout ; ++i){
        usleep(500000);      //  1 unit is 500 [ms]
        nvme_controller_status_t csts;
        csts.val = dev -> ctrl_reg ->csts.val;
        if(csts.rdy == ready) return 0;

    }
    printf("timeout\n");

    return -1;

}


int nvme_ctrl_disable(nvme_dev_t* dev){
    nvme_controller_config_t cc;   
    cc.val = dev -> ctrl_reg->cc.val;
    cc.en = 0;
    dev ->ctrl_reg->cc.val = cc.val;

    return nvme_ctrl_wait_ready(dev ,0);


}

int nvme_ctrl_enable(nvme_dev_t* dev, nvme_controller_config_t *cc){
    nvme_controller_config_t cc_temp;
    cc_temp.val = cc->val;
    cc_temp .en = 1;
    dev ->ctrl_reg->cc.val = cc_temp.val;
    return nvme_ctrl_wait_ready(dev, 1);


}


/**
 * @fn
 * @brief 　nvmeコントローラを初期化する
 * @param (引数名) 引数の説明
 * @param (引数名) 引数の説明
 * @return 戻り値の説明

 */
int init_nvme_ctrl(nvme_dev_t* dev, size_t sq_sz, size_t cq_sz){

    nvme_controller_cap_t cap   = {0};
    nvme_adminq_attr_t aqa      = {0};
    nvme_controller_config_t cc = {0};


    nvme_ctrl_disable(dev);  /* コントローラをnot readyにする */

    printf("csts: %x\n", dev->ctrl_reg->csts.val);
    printf("cap: %lx\n", dev->ctrl_reg->cap.val);
    printf("vs: %x\n", dev->ctrl_reg->vs.val);

    pthread_mutex_init(&dev -> q_pair[0]. cq_lock, nullptr);
    pthread_mutex_init(&dev -> q_pair[0]. sq_lock, nullptr);


    //cap . val = dev -> ctrl_reg -> cap.val;

    dev -> q_pair[0] . dev = dev;
    dev -> q_pair[0] . id = 0;
    dev -> q_pair[0] . sq_size = sq_sz;
    dev -> q_pair[0] . cq_size = cq_sz;

    /*  CQ phase tagとCQ Head、SQ tailを初期化する */
    dev -> q_pair[0] . cq_phase = 1;
    dev -> q_pair[0] . cq_head  = 0;
    dev -> q_pair[0] . sq_tail  = 0;


    if(pthread_cond_init(&dev->q_pair[0].sync_cond, nullptr)){
        perror("pthread_cond_init");
        return -1;
    }

    if(pthread_mutex_init(&dev->q_pair[0].sync_mutex, nullptr)){
        perror("pthread_mutex_init");
        return -1;
    }

#if 1
    /*  CQ用のDMAメモリを確保する  */
    dev -> admin_cq = dma_alloc(dev, dev->q_pair[0].cq_size  * sizeof(nvme_cq_entry_t));
    if(dev ->admin_cq == nullptr) {
      printf("Error: allocate admin cq\n");
      return -1;
    }

    /*  SQ用のDMAメモリを確保する  */
    dev ->admin_sq = dma_alloc(dev, dev->q_pair[0].sq_size  * sizeof(nvme_sq_entry_t));
    if(dev ->admin_sq == nullptr) {
      printf("Error: allocate admin sq\n");
      return -1;
    }


#else
    /*  CQ用のDMAメモリを確保する  */
    dev ->admin_cq = dma_aligned_alloc(dev, dev->q_pair[0].sq_size * sizeof(nvme_sq_entry_t), 4096);
    if(dev ->admin_cq == nullptr) {
      printf("Error: allocate admin cq\n");
      return -1;
    }

    /*  SQ用のDMAメモリを確保する  */
    dev ->admin_sq = dma_aligned_alloc(dev, dev->q_pair[0].sq_size * sizeof(nvme_sq_entry_t), 4096 );
    if(dev ->admin_sq == nullptr) {
      printf("Error: allocate admin sq\n");
      return -1;
    }

#endif

    cap.val = dev -> ctrl_reg ->cap.val;
    dev -> dstrd = cap.dstrd;


    dev -> q_pair[0].sq = (nvme_sq_entry_t*)dev ->admin_sq -> user_buf;
    dev -> q_pair[0].cq = (nvme_cq_entry_t*)dev ->admin_cq -> user_buf;

    dev -> q_pair[0].sq_doorbell = dev -> ctrl_reg -> sq0tdbl;
    dev -> q_pair[0].cq_doorbell = dev -> ctrl_reg -> sq0tdbl + (1<< dev->dstrd);
    
    dev -> ctrl_reg ->asq = dev ->admin_sq -> dma_addr;  /* asqレジスタにSQのDMAアドレスを設定する */
    dev -> ctrl_reg ->acq = dev ->admin_cq -> dma_addr;  /* acqレジスタにCQのDMAアドレスを設定する */

    /* SQ, CQのDMAアドレスを出力する。 */
    printf("SQ DMA address: %lx\n", dev -> ctrl_reg ->asq);
    printf("CQ DMA address: %lx\n", dev -> ctrl_reg ->acq);

    aqa.asqs = dev->q_pair[0] . sq_size -1;
    aqa.acqs = dev->q_pair[0] . cq_size -1;
    dev -> ctrl_reg -> aqa.val = aqa.val;    /* aqaレジスタにSQ、CQのサイズを設定する */


    cc.val = NVME_CC_CSS_NVM;
    cc.val |= 0 << NVME_CC_MPS_SHIFT;
    cc.val |= NVME_CC_AMS_RR | NVME_CC_SHN_NONE;
    cc.val |= NVME_CC_IOSQES | NVME_CC_IOCQES;

    nvme_ctrl_enable(dev, &cc);       /* コントローラを readyにする */

    printf("csts: %x\n", dev->ctrl_reg->csts.val);
    printf("cap: %lx\n", dev->ctrl_reg->cap.val);
    printf("vs: %x\n", dev->ctrl_reg->vs.val);
    printf("cc: %x\n", dev->ctrl_reg->cc.val);
   
    return 0;


}




static vfio_mem_t* vfio_mem_alloc(nvme_dev_t* dev, size_t size, void* pmb)
{
    vfio_mem_t* mem = ( vfio_mem_t* )zalloc(sizeof(*mem));
    struct vfio_iommu_type1_dma_map dma_map = {0};

    //printf("size: %ld\n", size);

    if (pmb) {
        mem->dma.user_buf = (u8*)pmb;
    } else {

        mem->dma.user_buf = (u8*)mmap(nullptr, size, PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0);   // MAP_PRIVATEでCopy On Write
        if (mem->dma.user_buf == MAP_FAILED) {
            printf("mmap errno %d\n", errno);
            goto error;
        }
        mem->mmap = 1;

    }

    memset(mem->dma.user_buf, 0, size);   // COW

    pthread_spin_lock(&dev->lock);

    dma_map.argsz = sizeof(dma_map);
    dma_map.flags = (VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE);
    dma_map.size = (u64)size;
    dma_map.vaddr = (u64)mem->dma.user_buf;
    dma_map.iova = (u64)virt_to_phys(reinterpret_cast<void*>(mem->dma.user_buf));

    if (ioctl(dev->contfd, VFIO_IOMMU_MAP_DMA, &dma_map) < 0) {
        printf("ioctl VFIO_IOMMU_MAP_DMA errno %d\n", errno);
        pthread_spin_unlock(&dev->lock);
        goto error;
    }
    mem->dma.size = size;
    mem->dma.dma_addr = dma_map.iova;
    mem->dma.id = mem;
    mem->dev = dev;

    // add node to the memory list
    if (!dev->memlist) {
        mem->prev = mem;
        mem->next = mem;
        dev->memlist = mem;
    } else {
        mem->prev = dev->memlist->prev;
        mem->next = dev->memlist;
        dev->memlist->prev->next = mem;
        dev->memlist->prev = mem;
    }

    //dev->iova = map.iova + size;
    //DEBUG_FN("%x: %#lx %#lx %#lx", dev->pci, map.iova, map.size, dev->iova);
    pthread_spin_unlock(&dev->lock);

    return mem;

error:
    if (mem->mmap) munmap(mem->dma.user_buf, size);
    free(mem);
    return NULL;
}

static int vfio_mem_free(vfio_mem_t* mem)
{
    nvme_dev_t* dev = mem->dev;

    printf("vfio_mem_free\n");

    struct vfio_iommu_type1_dma_unmap unmap = {0};

    unmap.argsz = sizeof(unmap);
    unmap.size =  (__u64)mem->dma.size;
    unmap.iova =  mem->dma.dma_addr;

    // unmap and free dma memory
    if (mem->dma.user_buf) {
        if (ioctl(dev->contfd, VFIO_IOMMU_UNMAP_DMA, &unmap) < 0) {
            printf("ioctl VFIO_IOMMU_UNMAP_DMA errno %d\n", errno);
            return -1;
        }
    }

    if (mem->mmap) {
        printf("munmap\n");
        if (munmap(mem->dma.user_buf, mem->dma.size) < 0) {
            printf("munmap errno %d", errno);
            return -1;
        }
    }
    else{
        printf("free\n");
        free(mem->dma.user_buf);
    }

    // remove node from memory list
    pthread_spin_lock(&dev->lock);
    if (mem->next == dev->memlist) dev->iova -= mem->dma.size;
    if (mem->next == mem) {
        dev->memlist = NULL;
        dev->iova = VFIO_IOVA;
    } else {
        mem->next->prev = mem->prev;
        mem->prev->next = mem->next;
        if (dev->memlist == mem) dev->memlist = mem->next;
        dev->iova = dev->memlist->prev->dma.dma_addr + dev->memlist->prev->dma.size;
    }
    //DEBUG_FN("%x: %#lx %ld %#lx", dev->pci, unmap.iova, unmap.size, dev->iova);
    pthread_spin_unlock(&dev->lock);

    free(mem);
    return 0;
}

/**
 * @fn
 * @brief 　DMA転送用のメモリーを確保する。
 * @param (引数名) 引数の説明
 * @param (引数名) 引数の説明
 * @return 戻り値の説明
 */
vfio_dma_t* dma_alloc(nvme_dev_t* vdev, size_t size)
{
    size_t sz = VFIO_PASIZE(size);
    vfio_mem_t* mem = vfio_mem_alloc(vdev, sz, 0);
    return mem ? &mem->dma : nullptr;
}


/**
 * @fn
 * @brief 　メモリのアライメントを指定して、DMA転送用のメモリーを確保する。
 * @param (引数名) 引数の説明
 * @param (引数名) 引数の説明
 * @return 戻り値の説明

 */
vfio_dma_t* dma_aligned_alloc(nvme_dev_t* vdev, size_t size, size_t aligned_size)
{
    size_t sz = VFIO_PASIZE(size);
    
    void *ptr;

    if(posix_memalign(&ptr, aligned_size, sz) != 0){
      perror("posix_memalign");
      return nullptr;    
    }

    vfio_mem_t* mem = vfio_mem_alloc(vdev, sz, ptr);
    return mem ? &mem->dma : nullptr;
}

int dma_free(vfio_dma_t* dma)
{
    return vfio_mem_free((vfio_mem_t*)dma->id);
}



nvme_dev_t* create_instance(int segn, int busn, int devn, int funcn)
{
    // map PCI to vfio device number
    char pciname[64];
    char path[128];
    int i;
    sprintf(pciname, "%04x:%02x:%02x.%x", segn, busn, devn, funcn);

    sprintf(path, "/sys/bus/pci/devices/%s/driver", pciname);

    if ((i = readlink(path, path, sizeof(path))) < 0) {
        printf("unknown PCI device %s", pciname);
        return NULL;
    }
    path[i] = 0;
    if (!strstr(path, "/vfio-pci")) {
        printf("device %s not bound to vfio driver", pciname);
        return NULL;
    }

    sprintf(path, "/sys/bus/pci/devices/%s/iommu_group", pciname);
    if ((i = readlink(path, path, sizeof(path))) < 0) {
        printf("No iommu_group associated with device %s", pciname);
        return NULL;
    }
    path[i] = 0;
    int vfid = atoi(strrchr(path, '/') + 1);
    
    struct vfio_group_status group_status = { .argsz = sizeof(group_status) };
    struct vfio_iommu_type1_info iommu_info = { .argsz = sizeof(iommu_info) };
    struct vfio_device_info dev_info = { .argsz = sizeof(dev_info) };

    // allocate and initialize device context
    nvme_dev_t* dev = (nvme_dev_t*)zalloc(sizeof(*dev));

    dev->device_info.argsz = sizeof(struct vfio_device_info);
    dev->group_status.argsz = sizeof(struct vfio_group_status);
    dev->iommu_info.argsz = sizeof(struct vfio_iommu_type1_info);
    for (int i = 0; i < VFIO_PCI_NUM_REGIONS; i++) {
        dev->regs[i].argsz = sizeof(struct vfio_region_info);
    }
    for (int i = 0; i < VFIO_PCI_NUM_IRQS; i++) {
        dev->irqs[i].argsz = sizeof(struct vfio_irq_info);
    }

    dev->pci = ((segn << 24) & 0xffff000000) | ((busn << 16) & 0xff0000) | ((devn << 8) & 0xff00) | (funcn & 0xff);

    printf("%x\n", dev->pci);

    dev->iova = VFIO_IOVA;
    if (pthread_spin_init(&dev->lock, PTHREAD_PROCESS_PRIVATE)) return NULL;

    // map vfio context
    if ((dev->contfd = open("/dev/vfio/vfio", O_RDWR)) < 0) {
        printf("open /dev/vfio/vfio\n");
        goto error;
    }
    if (ioctl(dev->contfd, VFIO_GET_API_VERSION) != VFIO_API_VERSION) {
        printf("ioctl VFIO_GET_API_VERSION\n");
        goto error;
    }
    if (ioctl(dev->contfd, VFIO_CHECK_EXTENSION, VFIO_TYPE1_IOMMU) == 0) {
        printf("ioctl VFIO_CHECK_EXTENSION\n");
        goto error;
    }

    sprintf(path, "/dev/vfio/%d", vfid);
    if ((dev->groupfd = open(path, O_RDWR)) < 0) {
        printf("open %s failed\n", path);
        goto error;
    }
    if (ioctl(dev->groupfd, VFIO_GROUP_GET_STATUS, &group_status) < 0) {
        printf("ioctl VFIO_GROUP_GET_STATUS\n");
        goto error;
    }
    if (!(group_status.flags & VFIO_GROUP_FLAGS_VIABLE)) {
        printf("group not viable %#x\n", group_status.flags);
        goto error;
    }
    if (ioctl(dev->groupfd, VFIO_GROUP_SET_CONTAINER, &dev->contfd) < 0) {
        printf("ioctl VFIO_GROUP_SET_CONTAINER\n");
        goto error;
    }

    if (ioctl(dev->contfd, VFIO_SET_IOMMU, VFIO_TYPE1_IOMMU) < 0) {
        printf("ioctl VFIO_SET_IOMMU\n");
        goto error;
    }
    if (ioctl(dev->contfd, VFIO_IOMMU_GET_INFO, &iommu_info) < 0) {
        printf("ioctl VFIO_IOMMU_GET_INFO\n");
        goto error;
    }

    dev->fd = ioctl(dev->groupfd, VFIO_GROUP_GET_DEVICE_FD, pciname);
    if (dev->fd < 0) {
        printf("ioctl VFIO_GROUP_GET_DEVICE_FD\n");
        goto error;
    }
    if (ioctl(dev->fd, VFIO_DEVICE_GET_INFO, &dev_info) < 0) {
        printf("ioctl VFIO_DEVICE_GET_INFO\n");
    }
    //printf("%x: flags=%u regions=%u irqs=%u", pci, dev_info.flags, dev_info.num_regions, dev_info.num_irqs);

    for (i = 0; i < dev_info.num_regions; i++) {
        dev -> regs[i].index = i;
        if (ioctl(dev->fd, VFIO_DEVICE_GET_REGION_INFO, &dev ->regs[i])) continue;
    }

    for (i = 0; i < dev_info.num_irqs; i++) {
        dev ->irqs[i].index = i;    
        if (ioctl(dev->fd, VFIO_DEVICE_GET_IRQ_INFO, &dev -> irqs[i])) continue;
    }

    return (nvme_dev_t*)dev;

error:
    delete_instance((nvme_dev_t*)dev);
    return NULL;
}



void delete_instance(nvme_dev_t* vdev)
{
    if (!vdev) return;
    nvme_dev_t* dev = (nvme_dev_t*)vdev;
    //DEBUG_FN("%x", dev->pci);

    // free all memory associated with the device
    while (dev->memlist) vfio_mem_free(dev->memlist);

    if (dev->fd) {
        close(dev->fd);
        dev->fd = 0;
    }
    if (dev->contfd) {
        close(dev->contfd);
        dev->contfd = 0;
    }
    if (dev->groupfd) {
        close(dev->groupfd);
        dev->groupfd = 0;
    }

    pthread_spin_destroy(&dev->lock);
    free(dev);
}
