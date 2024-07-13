#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/vfio.h>
#include <pthread.h>

#include "nvme.h"
#include "vfio.h"
#include "irq.h"





int main(int argc, char* argv[]) {

    nvme_dev_t* dev = nullptr;
    vfio_dma_t* data_buffer = nullptr;
    pthread_t intr_th;

    nvme_get_log_page_command_t* cmd = {0};

    int cid = 0;
    bool ret = false;
    int segn, busn, devn, funcn;

    int counter_ = 0;

    if (argc < 2 || sscanf(argv[1], "%04x:%02x:%02x.%d", &segn, &busn, &devn,&funcn) != 4) {
        printf("Usage: %s ssss:bb:dd.f\n", argv[1]);
        return -1;
    }

    dev = create_instance(segn, busn, devn, funcn);

    if(dev == nullptr) return -1;

    /* Bus Master Enableにする */
    enable_bus_master(dev);

    /* コントローラレジスタをメモリにmmapする。 */
    ret = map_bar0(dev);
    if(ret == false) return -1;

    /* 割り込みの設定 */
#if defined(INTx)   
    enable_intx(dev);
#elif defined(MSIX)
    enable_msix(dev);
#else 
    enable_msix(dev);
#endif



    /* NVMeコントローラの初期化 */
    if(init_device(dev, 16, 16) != 0){
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

    data_buffer = dma_alloc(dev, 4096);
    if(data_buffer == nullptr) {
      printf("Error: allocate data buffer\n");
      goto error;
    }

    cid = dev->admin_q_pair.sq_tail;
    cmd = &dev->admin_q_pair.sq[cid].get_log_page;

    memset(cmd, 0, sizeof (*cmd));

    cmd->opcode = nvme_admin_get_log_page;
    cmd->command_id = cid;
    cmd->nsid = 0xffffffff;
    cmd->dptr.prp1 = data_buffer->dma_addr;

    cmd->lid = 2;
    cmd->numdl =  (data_buffer->size / sizeof(u32) - 1) & 0xff;
    cmd->numdu =  ((data_buffer->size / sizeof(u32) - 1) >> 16) & 0xff;


    nvme_submit_cmd(&dev->admin_q_pair);


    pthread_mutex_lock(&dev->admin_q_pair.sync_cmd_mutex);
    pthread_cond_wait(&dev->admin_q_pair.sync_cmd_cond, &dev->admin_q_pair.sync_cmd_mutex);
    pthread_mutex_unlock(&dev->admin_q_pair.sync_cmd_mutex);



  for(int i=0; i<512; ++i)
    printf("%x, ", data_buffer->user_buf[i]);

  printf("\n");


error:

    /*  
        ここからは確保したメモリーの後始末  
        やらなくてもプログラムが終了したらOSが勝手に後始末してくれる。
   */
    if(data_buffer != nullptr) dma_free(data_buffer);
    if(dev -> admin_sq != nullptr) dma_free(dev ->admin_sq);
    if(dev -> admin_cq != nullptr) dma_free(dev->admin_cq);
    if(dev -> ctrl_reg != nullptr) unmap_bar0(dev);

    /* 割り込みの解除 */
#if defined(INTx)   
    disable_intx(dev);

#elif defined(MSIX)
    disable_msix(dev);
#else 
    disable_msix(dev);
#endif

    disable_bus_master(dev);


    if(dev != nullptr) delete_instance(dev);

    printf("finish\n");


}