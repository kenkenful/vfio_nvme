
#include "command.h"

int nvme_get_log(nvme_dev_t *dev, int logid, int loglen, vfio_dma_t* data_buffer){
        printf("%s, %s, %d\n", __FILE__, __func__, __LINE__);

    nvme_get_log_page_command_t *cmd;

    int cid = dev ->q_pair[0].sq_tail;

    cmd = &dev ->q_pair[0].sq[cid].get_log_page;

    memset(cmd, 0 , sizeof(*cmd));

    cmd ->opcode = nvme_admin_get_log_page;
    cmd->command_id = cid;
    cmd->nsid = 0xffffffff;
    cmd->dptr.prp1 = data_buffer->dma_addr;
    cmd ->lid = logid;
    cmd ->numdl = (loglen / sizeof(u32) -1) & 0xff;
    cmd ->numdu = ((loglen / sizeof(u32) -1) >> 16) & 0xff;

    nvme_submit_cmd(&dev ->q_pair[0]);

    return cid;
}

int nvme_id_ns(nvme_dev_t *dev, vfio_dma_t* data_buffer){
        printf("%s, %s, %d\n", __FILE__, __func__, __LINE__);

    nvme_common_command_t *cmd;

    int cid = dev ->q_pair[0].sq_tail;

    cmd = &dev ->q_pair[0].sq[cid].common;

    memset(cmd, 0 , sizeof(*cmd));

    cmd->opcode = nvme_admin_identify;
    cmd->command_id = cid;
    cmd->nsid = 0;
    cmd->prp1 = data_buffer->dma_addr;
    cmd->cdw10 = NVME_ID_CNS_NS_ACTIVE_LIST;
    
    nvme_submit_cmd(&dev ->q_pair[0]);

    return cid;
}


int nvme_id_ctrl(nvme_dev_t *dev, vfio_dma_t* data_buffer){
        printf("%s, %s, %d\n", __FILE__, __func__, __LINE__);

    nvme_common_command_t *cmd;

    int cid = dev ->q_pair[0].sq_tail;

    cmd = &dev ->q_pair[0].sq[cid].common;

    memset(cmd, 0 , sizeof(*cmd));

    cmd ->opcode = nvme_admin_identify;
    cmd->command_id = cid;
    cmd->nsid = 0;
    cmd-> prp1 = data_buffer->dma_addr;
    cmd-> cdw10 = NVME_ID_CNS_CTRL;
    
    nvme_submit_cmd(&dev ->q_pair[0]);

    return cid;
}




int nvme_create_cq(nvme_dev_t *dev, int cqid, int qsize, int flags, int vector, vfio_dma_t* data_buffer){
        printf("%s, %s, %d\n", __FILE__, __func__, __LINE__);

    nvme_create_cq_t *cmd;

    int cid = dev ->q_pair[0].sq_tail;

    cmd = &dev ->q_pair[0].sq[cid].create_cq;

    memset(cmd, 0 , sizeof(*cmd));

    cmd ->opcode = nvme_admin_create_cq;
    cmd->command_id = cid;
    cmd-> prp1 = data_buffer->dma_addr;
    cmd-> cqid = cqid;
    cmd-> qsize = qsize -1;
    cmd-> cq_flags = flags;
    cmd->irq_vector = vector;
    
    nvme_submit_cmd(&dev ->q_pair[0]);

    return cid;
}

int nvme_create_sq(nvme_dev_t *dev, int sqid, int qsize, int flags, int related_cqid, vfio_dma_t* data_buffer){
        printf("%s, %s, %d\n", __FILE__, __func__, __LINE__);

    nvme_create_sq_t *cmd;

    int cid = dev ->q_pair[0].sq_tail;

    cmd = &dev ->q_pair[0].sq[cid].create_sq;

    memset(cmd, 0 , sizeof(*cmd));

    cmd ->opcode = nvme_admin_create_sq;
    cmd->command_id = cid;

    cmd-> prp1 = data_buffer->dma_addr;

    cmd-> sqid = sqid;
    cmd-> qsize = qsize -1;    
    cmd-> sq_flags = flags;
    cmd-> cqid = related_cqid;
    
    nvme_submit_cmd(&dev ->q_pair[0]);

    return cid;
}

int nvme_delete_cq(nvme_dev_t *dev, int cqid){
        printf("%s, %s, %d\n", __FILE__, __func__, __LINE__);

    nvme_delete_queue_t *cmd;

    int cid = dev ->q_pair[0].sq_tail;

    cmd = &dev ->q_pair[0].sq[cid].delete_queue;

    memset(cmd, 0 , sizeof(*cmd));

    cmd ->opcode = nvme_admin_delete_cq;
    cmd->command_id = cid;

    cmd-> qid = cqid;

    nvme_submit_cmd(&dev ->q_pair[0]);

    return cid;
}



int nvme_delete_sq(nvme_dev_t *dev, int sqid){
    printf("%s, %s, %d\n", __FILE__, __func__, __LINE__);

    nvme_delete_queue_t *cmd;

    int cid = dev ->q_pair[0].sq_tail;

    cmd = &dev ->q_pair[0].sq[cid].delete_queue;

    memset(cmd, 0 , sizeof(*cmd));

    cmd ->opcode = nvme_admin_delete_sq;
    cmd->command_id = cid;

    cmd-> qid = sqid;

    nvme_submit_cmd(&dev ->q_pair[0]);

    return cid;
}


int nvme_write(nvme_dev_t *dev, int io_qid, int nsid, int start, int len, vfio_dma_t *data_buffer){
    printf("%s, %s, %d\n", __FILE__, __func__, __LINE__);

    nvme_rw_command *cmd;

    int cid = dev ->q_pair[io_qid].sq_tail;

    cmd = &dev ->q_pair[io_qid].sq[cid].rw;

    memset(cmd, 0 , sizeof(*cmd));

    cmd ->opcode = nvme_cmd_write;
    cmd->command_id = cid;

    cmd-> nsid = nsid;
    cmd -> prp1 = data_buffer ->dma_addr;

    cmd -> slba = start;
    cmd -> length = len -1;

    nvme_submit_cmd(&dev ->q_pair[io_qid]);

    return cid;
}


int nvme_read(nvme_dev_t *dev, int io_qid, int nsid, int start, int len, vfio_dma_t *data_buffer){
        printf("%s, %s, %d\n", __FILE__, __func__, __LINE__);

    nvme_rw_command *cmd;

    int cid = dev ->q_pair[io_qid].sq_tail;

    cmd = &dev ->q_pair[io_qid].sq[cid].rw;

    memset(cmd, 0 , sizeof(*cmd));

    cmd ->opcode = nvme_cmd_read;
    cmd->command_id = cid;

    cmd-> nsid = nsid;
    cmd -> prp1 = data_buffer ->dma_addr;

    cmd -> slba = start;
    cmd -> length = len -1;

    nvme_submit_cmd(&dev ->q_pair[io_qid]);

    return cid;
}