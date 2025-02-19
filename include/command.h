#pragma once

#include "vfio.h"

int nvme_get_log(nvme_dev_t *dev, int logid, int loglen, vfio_dma_t* dma_buffer);

int nvme_id_ns(nvme_dev_t *dev, vfio_dma_t* dma_buffer);

int nvme_id_ctrl(nvme_dev_t *dev, vfio_dma_t* dma_buffer);

int nvme_create_cq(nvme_dev_t *dev, int cqid, int qsize, int flags, int vector, vfio_dma_t* dma_buffer);

int nvme_create_sq(nvme_dev_t *dev, int sqid, int qsize, int flags, int related_cqid, vfio_dma_t* dma_buffer);

int nvme_delete_cq(nvme_dev_t *dev, int cqid);

int nvme_delete_sq(nvme_dev_t *dev, int sqid);

int nvme_write(nvme_dev_t *dev, int io_qid, int nsid, int start, int len, vfio_dma_t* dma_buffer);

int nvme_read(nvme_dev_t *dev, int io_qid, int nsid, int start, int len, vfio_dma_t* dma_buffer);