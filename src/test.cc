#include "vfio.h"
#include "irq.h"
#include "command.h"

int main(int argc, char* argv[]) {

    nvme_dev_t* dev = nullptr;

    vfio_dma_t* getlog_buffer = nullptr;
    vfio_dma_t* identify_buffer = nullptr;
    
    int io_qid = 1;

    int io_cq_size = 64;
    vfio_dma_t* io_cq_buffer = nullptr;


    int io_sq_size = 64;
    vfio_dma_t* io_sq_buffer = nullptr;

    int cid = 0;

    pthread_t intr_th;

    bool ret = false;
    int segn, busn, devn, funcn;

    if (argc < 2 || sscanf(argv[1], "%04x:%02x:%02x.%d", &segn, &busn, &devn,&funcn) != 4) {
        printf("Usage: %s ssss:bb:dd.f\n", argv[1]);
        return -1;
    }

    dev = create_instance(segn, busn, devn, funcn);

    if(dev == nullptr) return -1;

    dev -> current_io_q_pair_num = 0;


    /* Bus Master Enableにする */
    enable_bus_master(dev);

    /* コントローラレジスタをメモリにmmapする。 */
    ret = map_bar0(dev);
    if(ret == false) return -1;

    /* 割り込みの設定 */
#if defined(MSIX)   
      if(enable_msix(dev) != 0){
        printf("error: enable MSIx\n");
        goto error;
      }
#elif defined(INTx)
      if(enable_intx(dev) != 0){
        printf("error: enable INTx\n");
        goto error;
      }

#endif



    /* NVMeコントローラの初期化 */
    if(init_nvme_ctrl(dev, 16, 16) != 0){
      goto error;

    }


    /* NVMeからの割り込みを受けるThreadをキックする */
    if (pthread_create(&intr_th, NULL, interrupt_hadler, dev) != 0) {
      printf("Error: pthread create\n");
      goto error;
    }
    if (pthread_detach(intr_th) != 0) {
      printf("Error: pthread detach\n");
      goto error;
    }



    /* コマンドを作成して発行する */

    getlog_buffer = dma_alloc(dev, 4096);
    if(getlog_buffer == nullptr) {
      printf("Error: allocate data buffer\n");
      goto error;
    }

    cid = nvme_get_log(dev, 2, 512, getlog_buffer);

    pthread_mutex_lock(&dev->q_pair[0].sync_mutex);
    pthread_cond_wait(&dev->q_pair[0].sync_cond, &dev->q_pair[0].sync_mutex);
    pthread_mutex_unlock(&dev->q_pair[0].sync_mutex);
  
    identify_buffer = dma_alloc(dev, 4096);
    if(identify_buffer == nullptr){
        printf("Error: allocate data buffer\n");
        goto error;
    }
  
    cid = nvme_id_ctrl(dev, identify_buffer);
  
    pthread_mutex_lock(&dev->q_pair[0].sync_mutex);
    pthread_cond_wait(&dev->q_pair[0].sync_cond, &dev->q_pair[0].sync_mutex);
    pthread_mutex_unlock(&dev->q_pair[0].sync_mutex);



    cid = nvme_id_ns(dev, identify_buffer);

    pthread_mutex_lock(&dev->q_pair[0].sync_mutex);
    pthread_cond_wait(&dev->q_pair[0].sync_cond, &dev->q_pair[0].sync_mutex);
    pthread_mutex_unlock(&dev->q_pair[0].sync_mutex);

    io_cq_buffer = dma_alloc(dev, sizeof(nvme_cq_entry_t) * io_cq_size);
    if(io_cq_buffer == nullptr){
      printf("Error: allocate data buffer\n");
      goto error;
    }



#if defined(MSIX)
    cid = nvme_create_cq(dev, io_qid, io_cq_size, NVME_QUEUE_PHYS_CONTIG | NVME_CQ_IRQ_ENABLED, io_qid, io_cq_buffer);
#elif defined(INTx)
    cid = nvme_create_cq(dev, io_qid, io_cq_size, NVME_QUEUE_PHYS_CONTIG | NVME_CQ_IRQ_ENABLED, 0, io_cq_buffer);

#endif

    pthread_mutex_lock(&dev->q_pair[0].sync_mutex);
    pthread_cond_wait(&dev->q_pair[0].sync_cond, &dev->q_pair[0].sync_mutex);
    pthread_mutex_unlock(&dev->q_pair[0].sync_mutex);


    io_sq_buffer = dma_alloc(dev, sizeof(nvme_sq_entry_t) * io_sq_size);

    if(io_sq_buffer == nullptr){
      printf("Error: allocate data buffer\n");
      goto error;
    }


    cid = nvme_create_sq(dev, io_qid, io_sq_size, NVME_QUEUE_PHYS_CONTIG | NVME_SQ_PRIO_MEDIUM, io_qid, io_sq_buffer);

    pthread_mutex_lock(&dev->q_pair[0].sync_mutex);
    pthread_cond_wait(&dev->q_pair[0].sync_cond, &dev->q_pair[0].sync_mutex);
    pthread_mutex_unlock(&dev->q_pair[0].sync_mutex);




    /* Set IO Queue */

    pthread_mutex_init(&dev->q_pair[io_qid].cq_lock, nullptr);
    pthread_mutex_init(&dev->q_pair[io_qid].sq_lock, nullptr);


    if(pthread_cond_init(&dev ->q_pair[io_qid].sync_cond, nullptr)){
      perror("pthread_cond_init");
      goto error;
    }
    
    if(pthread_mutex_init(&dev ->q_pair[io_qid].sync_mutex, nullptr)){
      perror("pthread_mutex_init");
      goto error;
    }
    

    dev -> q_pair[io_qid].dev = dev;
    dev -> q_pair[io_qid].id = io_qid;
    dev -> q_pair[io_qid].sq_size = io_sq_size;
    dev -> q_pair[io_qid].cq_size = io_cq_size;
    dev -> q_pair[io_qid].cq_phase = 1;
    dev -> q_pair[io_qid].cq_head = 0;
    dev -> q_pair[io_qid].sq_tail = 0;
    dev -> q_pair[io_qid].sq = (nvme_sq_entry_t*) io_sq_buffer->user_buf;
    dev -> q_pair[io_qid].cq = (nvme_cq_entry_t*) io_cq_buffer->user_buf;
    dev -> q_pair[io_qid].sq_doorbell = dev -> ctrl_reg -> sq0tdbl + (1 << dev->dstrd) * (io_qid + 1);
    dev -> q_pair[io_qid].cq_doorbell = dev -> ctrl_reg -> sq0tdbl + (1 << dev->dstrd) * (io_qid + 2);

    dev -> current_io_q_pair_num = 1;


    cid = nvme_write(dev, io_qid, 1, 0, 8, identify_buffer);
    pthread_mutex_lock(&dev->q_pair[io_qid].sync_mutex);        
    pthread_cond_wait(&dev->q_pair[io_qid].sync_cond, &dev->q_pair[1].sync_mutex);
    pthread_mutex_unlock(&dev->q_pair[io_qid].sync_mutex);



    cid = nvme_write(dev, io_qid, 1, 0, 8, identify_buffer);
    pthread_mutex_lock(&dev->q_pair[io_qid].sync_mutex);
    pthread_cond_wait(&dev->q_pair[io_qid].sync_cond, &dev->q_pair[1].sync_mutex);
    pthread_mutex_unlock(&dev->q_pair[io_qid].sync_mutex);



    cid = nvme_write(dev, io_qid, 1, 0, 8, identify_buffer);
    pthread_mutex_lock(&dev->q_pair[io_qid].sync_mutex);
    pthread_cond_wait(&dev->q_pair[io_qid].sync_cond, &dev->q_pair[1].sync_mutex);
    pthread_mutex_unlock(&dev->q_pair[io_qid].sync_mutex);




    cid = nvme_delete_sq(dev, io_qid);
    pthread_mutex_lock(&dev->q_pair[0].sync_mutex);
    pthread_cond_wait(&dev->q_pair[0].sync_cond, &dev->q_pair[0].sync_mutex);
    pthread_mutex_unlock(&dev->q_pair[0].sync_mutex);




    cid = nvme_delete_cq(dev, io_qid);
    pthread_mutex_lock(&dev->q_pair[0].sync_mutex);
    pthread_cond_wait(&dev->q_pair[0].sync_cond, &dev->q_pair[0].sync_mutex);
    pthread_mutex_unlock(&dev->q_pair[0].sync_mutex);



error:

    /*  
        ここからは確保したメモリーの後始末  
        やらなくてもプログラムが終了したらOSが勝手に後始末してくれる。
   */

#if 1  
    if(io_sq_buffer != nullptr) dma_free(io_sq_buffer);
    if(io_cq_buffer != nullptr) dma_free(io_cq_buffer);

    if(identify_buffer != nullptr) dma_free(identify_buffer);
    if(getlog_buffer != nullptr) dma_free(getlog_buffer);

    if(dev -> admin_sq != nullptr) dma_free(dev ->admin_sq);
    if(dev -> admin_cq != nullptr) dma_free(dev->admin_cq);
    if(dev -> ctrl_reg != nullptr) unmap_bar0(dev);

    /* 割り込みの解除 */
#if defined(MSIx) 
    disable_msix(dev); 
#elif defined(INTx) 
    disable_intx(dev);
#endif

    disable_bus_master(dev);

    if(dev != nullptr) delete_instance(dev);

#endif
    printf("finish\n");


}