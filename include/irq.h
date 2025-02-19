#pragma once


int enable_msix(nvme_dev_t* dev);

int disable_msix(nvme_dev_t* vdev);

static int unmask_intx(nvme_dev_t* dev);

int enable_intx(nvme_dev_t* dev);

int disable_intx(nvme_dev_t* dev);

void* interrupt_hadler(void* p);

