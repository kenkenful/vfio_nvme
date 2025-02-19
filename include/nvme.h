#pragma once

#include <stdint.h>


typedef int8_t              s8;         ///< 8-bit signed
typedef int16_t             s16;        ///< 16-bit signed
typedef int32_t             s32;        ///< 32-bit signed
typedef int64_t             s64;        ///< 64-bit signed
typedef uint8_t             u8;         ///< 8-bit unsigned
typedef uint16_t            u16;        ///< 16-bit unsigned
typedef uint32_t            u32;        ///< 32-bit unsigned
typedef uint64_t            u64;        ///< 64-bit unsigned


#define NVME_NVM_IOSQES		6
#define NVME_NVM_IOCQES		4


enum {
	NVME_CC_ENABLE		= 1 << 0,
	NVME_CC_CSS_NVM		= 0 << 4,
	NVME_CC_EN_SHIFT	= 0,
	NVME_CC_CSS_SHIFT	= 4,
	NVME_CC_MPS_SHIFT	= 7,
	NVME_CC_AMS_SHIFT	= 11,
	NVME_CC_SHN_SHIFT	= 14,
	NVME_CC_IOSQES_SHIFT	= 16,
	NVME_CC_IOCQES_SHIFT	= 20,
	NVME_CC_AMS_RR		= 0 << NVME_CC_AMS_SHIFT,
	NVME_CC_AMS_WRRU	= 1 << NVME_CC_AMS_SHIFT,
	NVME_CC_AMS_VS		= 7 << NVME_CC_AMS_SHIFT,
	NVME_CC_SHN_NONE	= 0 << NVME_CC_SHN_SHIFT,
	NVME_CC_SHN_NORMAL	= 1 << NVME_CC_SHN_SHIFT,
	NVME_CC_SHN_ABRUPT	= 2 << NVME_CC_SHN_SHIFT,
	NVME_CC_SHN_MASK	= 3 << NVME_CC_SHN_SHIFT,
	NVME_CC_IOSQES		= NVME_NVM_IOSQES << NVME_CC_IOSQES_SHIFT,
	NVME_CC_IOCQES		= NVME_NVM_IOCQES << NVME_CC_IOCQES_SHIFT,
	NVME_CSTS_RDY		= 1 << 0,
	NVME_CSTS_CFS		= 1 << 1,
	NVME_CSTS_NSSRO		= 1 << 4,
	NVME_CSTS_PP		= 1 << 5,
	NVME_CSTS_SHST_NORMAL	= 0 << 2,
	NVME_CSTS_SHST_OCCUR	= 1 << 2,
	NVME_CSTS_SHST_CMPLT	= 2 << 2,
	NVME_CSTS_SHST_MASK	= 3 << 2,
};



enum nvme_opcode {
	nvme_cmd_flush		= 0x00,
	nvme_cmd_write		= 0x01,
	nvme_cmd_read		= 0x02,
	nvme_cmd_write_uncor	= 0x04,
	nvme_cmd_compare	= 0x05,
	nvme_cmd_write_zeroes	= 0x08,
	nvme_cmd_dsm		= 0x09,
	nvme_cmd_verify		= 0x0c,
	nvme_cmd_resv_register	= 0x0d,
	nvme_cmd_resv_report	= 0x0e,
	nvme_cmd_resv_acquire	= 0x11,
	nvme_cmd_resv_release	= 0x15,
	nvme_cmd_zone_mgmt_send	= 0x79,
	nvme_cmd_zone_mgmt_recv	= 0x7a,
	nvme_cmd_zone_append	= 0x7d,
	nvme_cmd_vendor_start	= 0x80,
};

enum nvme_admin_opcode {
	nvme_admin_delete_sq		= 0x00,
	nvme_admin_create_sq		= 0x01,
	nvme_admin_get_log_page		= 0x02,
	nvme_admin_delete_cq		= 0x04,
	nvme_admin_create_cq		= 0x05,
	nvme_admin_identify		= 0x06,
	nvme_admin_abort_cmd		= 0x08,
	nvme_admin_set_features		= 0x09,
	nvme_admin_get_features		= 0x0a,
	nvme_admin_async_event		= 0x0c,
	nvme_admin_ns_mgmt		= 0x0d,
	nvme_admin_activate_fw		= 0x10,
	nvme_admin_download_fw		= 0x11,
	nvme_admin_dev_self_test	= 0x14,
	nvme_admin_ns_attach		= 0x15,
	nvme_admin_keep_alive		= 0x18,
	nvme_admin_directive_send	= 0x19,
	nvme_admin_directive_recv	= 0x1a,
	nvme_admin_virtual_mgmt		= 0x1c,
	nvme_admin_nvme_mi_send		= 0x1d,
	nvme_admin_nvme_mi_recv		= 0x1e,
	nvme_admin_dbbuf		= 0x7C,
	nvme_admin_format_nvm		= 0x80,
	nvme_admin_security_send	= 0x81,
	nvme_admin_security_recv	= 0x82,
	nvme_admin_sanitize_nvm		= 0x84,
	nvme_admin_get_lba_status	= 0x86,
	nvme_admin_vendor_start		= 0xC0,
};


typedef struct nvme_sgl_desc {
	__le64	addr;
	__le32	length;
	__u8	rsvd[3];
	__u8	type;
}nvme_sgl_desc_t;

typedef struct nvme_keyed_sgl_desc {
	__le64	addr;
	__u8	length[3];
	__u8	key[4];
	__u8	type;
}nvme_keyed_sgl_desc_t;

union nvme_data_ptr {
	struct {
		__le64	prp1;
		__le64	prp2;
	};
	struct nvme_sgl_desc	sgl;
	struct nvme_keyed_sgl_desc ksgl;
};


typedef struct nvme_common_command {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__le32			nsid;
	__le32			cdw2[2];
	__le64			metadata;
	__le64			prp1;
	__le64			prp2;
	__le32			cdw10;
	__le32			cdw11;
	__le32			cdw12;
	__le32			cdw13;
	__le32			cdw14;
	__le32			cdw15;
}nvme_common_command_t;

typedef struct nvme_rw_command {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__le32			nsid;
	__u64			rsvd2;
	__le64			metadata;
	__le64			prp1;
	__le64			prp2;
	__le64			slba;
	__le16			length;
	__le16			control;
	__le32			dsmgmt;
	__le32			reftag;
	__le16			apptag;
	__le16			appmask;
}nvme_rw_command_t;


typedef struct nvme_identify {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__le32			nsid;
	__u64			rsvd2[2];
	__le64			prp1;
	__le64			prp2;
	__le32			cns;
	__u32			rsvd11[5];
}nvme_identify_t;

typedef struct nvme_features {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__le32			nsid;
	__u64			rsvd2[2];
	__le64			prp1;
	__le64			prp2;
	__le32			fid;
	__le32			dword11;
	__u32			rsvd12[4];
}nvme_features_t;

typedef struct nvme_create_cq {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__u32			rsvd1[5];
	__le64			prp1;
	__u64			rsvd8;
	__le16			cqid;
	__le16			qsize;
	__le16			cq_flags;
	__le16			irq_vector;
	__u32			rsvd12[4];
}nvme_create_cq_t;

typedef struct nvme_create_sq {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__u32			rsvd1[5];
	__le64			prp1;
	__u64			rsvd8;
	__le16			sqid;
	__le16			qsize;
	__le16			sq_flags;
	__le16			cqid;
	__u32			rsvd12[4];
}nvme_create_sq_t;

typedef struct nvme_delete_queue {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__u32			rsvd1[9];
	__le16			qid;
	__u16			rsvd10;
	__u32			rsvd11[5];
}nvme_delete_queue_t;

typedef struct nvme_abort_cmd {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__u32			rsvd1[9];
	__le16			sqid;
	__u16			cid;
	__u32			rsvd11[5];
}nvme_abort_cmd_t;

typedef struct nvme_download_firmware {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__u32			rsvd1[5];
	__le64			prp1;
	__le64			prp2;
	__le32			numd;
	__le32			offset;
	__u32			rsvd12[4];
}nvme_download_firmware_t;

typedef struct nvme_format_cmd {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__le32			nsid;
	__u64			rsvd2[4];
	__le32			cdw10;
	__u32			rsvd11[5];
}nvme_format_cmd_t;

typedef struct nvme_dsm_cmd {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__le32			nsid;
	__u64			rsvd2[2];
	__le64			prp1;
	__le64			prp2;
	__le32			nr;
	__le32			attributes;
	__u32			rsvd12[4];
}nvme_dsm_cmd_t;



typedef struct nvme_get_log_page_command {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__le32			nsid;
	__u64			rsvd2[2];
	union nvme_data_ptr	dptr;
	__u8			lid;
	__u8			lsp; /* upper 4 bits reserved */
	__le16			numdl;
	__le16			numdu;
	__u16			rsvd11;
	union {
		struct {
			__le32 lpol;
			__le32 lpou;
		};
		__le64 lpo;
	};
	__u8			rsvd14[3];
	__u8			csi;
	__u32			rsvd15;
}nvme_get_log_page_command_t;



typedef struct nvme_write_zeroes_cmd {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__le32			nsid;
	__u64			rsvd2;
	__le64			metadata;
	union nvme_data_ptr	dptr;
	__le64			slba;
	__le16			length;
	__le16			control;
	__le32			dsmgmt;
	__le32			reftag;
	__le16			apptag;
	__le16			appmask;
}nvme_write_zeroes_cmd_t;


typedef struct nvme_zone_mgmt_send_cmd {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__le32			nsid;
	__le32			cdw2[2];
	__le64			metadata;
	union nvme_data_ptr	dptr;
	__le64			slba;
	__le32			cdw12;
	__u8			zsa;
	__u8			select_all;
	__u8			rsvd13[2];
	__le32			cdw14[2];
}nvme_zone_mgmt_send_cmd_t;


typedef struct nvme_zone_mgmt_recv_cmd {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__le32			nsid;
	__le64			rsvd2[2];
	union nvme_data_ptr	dptr;
	__le64			slba;
	__le32			numd;
	__u8			zra;
	__u8			zrasf;
	__u8			pr;
	__u8			rsvd13;
	__le32			cdw14[2];
}nvme_zone_mgmt_recv_cmd_t;

typedef struct nvmf_common_command {
	__u8	opcode;
	__u8	resv1;
	__u16	command_id;
	__u8	fctype;
	__u8	resv2[35];
	__u8	ts[24];
}nvmf_common_command_t;


typedef struct nvmf_connect_command {
	__u8		opcode;
	__u8		resv1;
	__u16		command_id;
	__u8		fctype;
	__u8		resv2[19];
	union nvme_data_ptr dptr;
	__le16		recfmt;
	__le16		qid;
	__le16		sqsize;
	__u8		cattr;
	__u8		resv3;
	__le32		kato;
	__u8		resv4[12];
}nvmf_connect_command_t;

typedef struct nvmf_property_set_command {
	__u8		opcode;
	__u8		resv1;
	__u16		command_id;
	__u8		fctype;
	__u8		resv2[35];
	__u8		attrib;
	__u8		resv3[3];
	__le32		offset;
	__le64		value;
	__u8		resv4[8];
}nvmf_property_set_command_t;


typedef struct nvmf_property_get_command {
	__u8		opcode;
	__u8		resv1;
	__u16		command_id;
	__u8		fctype;
	__u8		resv2[35];
	__u8		attrib;
	__u8		resv3[3];
	__le32		offset;
	__u8		resv4[16];
}nvmf_property_get_command_t;

typedef struct nvme_dbbuf {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__u32			rsvd1[5];
	__le64			prp1;
	__le64			prp2;
	__u32			rsvd12[6];
}nvme_dbbuf;

typedef struct streams_directive_params {
	__le16	msl;
	__le16	nssa;
	__le16	nsso;
	__u8	rsvd[10];
	__le32	sws;
	__le16	sgs;
	__le16	nsa;
	__le16	nso;
	__u8	rsvd2[6];
}streams_directive_params_t;

typedef struct nvme_directive_cmd {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__le32			nsid;
	__u64			rsvd2[2];
	union nvme_data_ptr	dptr;
	__le32			numd;
	__u8			doper;
	__u8			dtype;
	__le16			dspec;
	__u8			endir;
	__u8			tdtype;
	__u16			rsvd15;

	__u32			rsvd16[3];
}nvme_directive_cmd_t;

typedef struct nvme_command {
	union {
		struct nvme_common_command common;
		struct nvme_rw_command rw;
		struct nvme_identify identify;
		struct nvme_features features;
		struct nvme_create_cq create_cq;
		struct nvme_create_sq create_sq;
		struct nvme_delete_queue delete_queue;
		struct nvme_download_firmware dlfw;
		struct nvme_format_cmd format;
		struct nvme_dsm_cmd dsm;
		struct nvme_write_zeroes_cmd write_zeroes;
		struct nvme_zone_mgmt_send_cmd zms;
		struct nvme_zone_mgmt_recv_cmd zmr;
		struct nvme_abort_cmd abort;
		struct nvme_get_log_page_command get_log_page;
		struct nvmf_common_command fabrics;
		struct nvmf_connect_command connect;
		struct nvmf_property_set_command prop_set;
		struct nvmf_property_get_command prop_get;
		struct nvme_dbbuf dbbuf;
		struct nvme_directive_cmd directive;
	};
}nvme_command_t;



/* nvme controller register */

typedef union _nvme_version {
    u32                 val;            
    struct {
        u8              rsvd;           
        u8              mnr;            
        u16             mjr;            
    };
} nvme_version_t;

typedef union _nvme_adminq_attr {
    u32                 val;            
    struct {
        u16             asqs;           
        u16             acqs;           
    };
} nvme_adminq_attr_t;

typedef union _nvme_controller_cap {
    u64                 val;            
    struct {
        u16             mqes;           
        u8              cqr     : 1;    
        u8              ams     : 2;   
        u8              rsvd    : 5;    
        u8              to;             

        u32             dstrd   : 4;    
        u32             nssrs   : 1;    
        u32             css     : 8;    
        u32             rsvd2   : 3;    
        u32             mpsmin  : 4;    
        u32             mpsmax  : 4;   
        u32             rsvd3   : 8;    
    };
} nvme_controller_cap_t;

typedef union _nvme_controller_config {
    u32                 val;            
    struct {
        u32             en      : 1;    
        u32             rsvd    : 3;    
        u32             css     : 3;    
        u32             mps     : 4;    
        u32             ams     : 3;    
        u32             shn     : 2;    
        u32             iosqes  : 4;    
        u32             iocqes  : 4;    
        u32             rsvd2   : 8;    
    };
} nvme_controller_config_t;

typedef union _nvme_controller_status {
    u32                 val;            
    struct {
        u32             rdy     : 1;    
        u32             cfs     : 1;    
        u32             shst    : 2;    
        u32             rsvd    : 28;       
    };
} nvme_controller_status_t;

typedef struct _nvme_controller_reg {
    nvme_controller_cap_t   cap;            
    nvme_version_t          vs;             
    u32                     intms;          
    u32                     intmc;          
    nvme_controller_config_t cc;             
    u32                     rsvd;            
    nvme_controller_status_t csts;           
    u32                     nssr;            
    nvme_adminq_attr_t      aqa;           
    u64                     asq;             
    u64                     acq;             
    u32                     rcss[1010];      
    u32                     sq0tdbl[1024];   
} nvme_controller_reg_t;

#if 0
struct nvmf_auth_common_command {
	__u8		opcode;
	__u8		resv1;
	__u16		command_id;
	__u8		fctype;
	__u8		resv2[19];
	union nvme_data_ptr dptr;
	__u8		resv3;
	__u8		spsp0;
	__u8		spsp1;
	__u8		secp;
	__le32		al_tl;
	__u8		resv4[16];
}nvmf_auth_common_command_t;

struct nvmf_auth_send_command {
	__u8		opcode;
	__u8		resv1;
	__u16		command_id;
	__u8		fctype;
	__u8		resv2[19];
	union nvme_data_ptr dptr;
	__u8		resv3;
	__u8		spsp0;
	__u8		spsp1;
	__u8		secp;
	__le32		tl;
	__u8		resv4[16];
}nvmf_auth_send_command_t;

struct nvmf_auth_receive_command {
	__u8		opcode;
	__u8		resv1;
	__u16		command_id;
	__u8		fctype;
	__u8		resv2[19];
	union nvme_data_ptr dptr;
	__u8		resv3;
	__u8		spsp0;
	__u8		spsp1;
	__u8		secp;
	__le32		al;
	__u8		resv4[16];
}nvmf_auth_receive_command_t;

#endif

typedef union _nvme_sq_entry {
    
	struct nvme_common_command common;
	struct nvme_rw_command rw;
	struct nvme_identify identify;
	struct nvme_features features;
	struct nvme_create_cq create_cq;
	struct nvme_create_sq create_sq;
	struct nvme_delete_queue delete_queue;
	struct nvme_download_firmware dlfw;
	struct nvme_format_cmd format;
	struct nvme_dsm_cmd dsm;
	struct nvme_write_zeroes_cmd write_zeroes;
	struct nvme_zone_mgmt_send_cmd zms;
	struct nvme_zone_mgmt_recv_cmd zmr;
	struct nvme_abort_cmd abort;
	struct nvme_get_log_page_command get_log_page;
	struct nvmf_common_command fabrics;
	struct nvmf_connect_command connect;
	struct nvmf_property_set_command prop_set;
	struct nvmf_property_get_command prop_get;
	//struct nvmf_auth_common_command auth_common;
	//struct nvmf_auth_send_command auth_send;
	//struct nvmf_auth_receive_command auth_receive;
	struct nvme_dbbuf dbbuf;
	struct nvme_directive_cmd directive;

} nvme_sq_entry_t;



typedef struct _nvme_cq_entry {
    u32                     cs;        
    u32                     rsvd;       
    u16                     sqhd;      
    u16                     sqid;       
    u16                     cid;        
    union {
        u16                 psf;        
        struct {
            u16             p     : 1;      
            u16             sc    : 8;    
            u16             sct   : 3;    
            u16             rsvd3 : 2;  
            u16             m     : 1;      
            u16             dnr   : 1;   
        };
    };
} nvme_cq_entry_t;



struct nvme_smart_log {
  u8  critical_warning;
  u8  temperature[2];
  u8  avail_spare;
  u8  spare_thresh;
  u8  percent_used;
  u8  rsvd6[26];
  u8  data_units_read[16];
  u8  data_units_written[16];
  u8  host_reads[16];
  u8  host_writes[16];
  u8  ctrl_busy_time[16];
  u8  power_cycles[16];
  u8  power_on_hours[16];
  u8  unsafe_shutdowns[16];
  u8  media_errors[16];
  u8  num_err_log_entries[16];
  u32   warning_temp_time;
  u32   critical_comp_time;
  u16 temp_sensor[8];
  u32   thm_temp1_trans_count;
  u32   thm_temp2_trans_count;
  u32   thm_temp1_total_time;
  u32   thm_temp2_total_time;
  u8  rsvd232[280];
};

enum {
	NVME_ID_CNS_NS			= 0x00,
	NVME_ID_CNS_CTRL		= 0x01,
	NVME_ID_CNS_NS_ACTIVE_LIST	= 0x02,
	NVME_ID_CNS_NS_DESC_LIST	= 0x03,
	NVME_ID_CNS_CS_NS		= 0x05,
	NVME_ID_CNS_CS_CTRL		= 0x06,
	NVME_ID_CNS_NS_CS_INDEP		= 0x08,
	NVME_ID_CNS_NS_PRESENT_LIST	= 0x10,
	NVME_ID_CNS_NS_PRESENT		= 0x11,
	NVME_ID_CNS_CTRL_NS_LIST	= 0x12,
	NVME_ID_CNS_CTRL_LIST		= 0x13,
	NVME_ID_CNS_SCNDRY_CTRL_LIST	= 0x15,
	NVME_ID_CNS_NS_GRANULARITY	= 0x16,
	NVME_ID_CNS_UUID_LIST		= 0x17,
};

enum {
	NVME_QUEUE_PHYS_CONTIG	= (1 << 0),
	NVME_CQ_IRQ_ENABLED	= (1 << 1),
	NVME_SQ_PRIO_URGENT	= (0 << 1),
	NVME_SQ_PRIO_HIGH	= (1 << 1),
	NVME_SQ_PRIO_MEDIUM	= (2 << 1),
	NVME_SQ_PRIO_LOW	= (3 << 1),
	NVME_FEAT_ARBITRATION	= 0x01,
	NVME_FEAT_POWER_MGMT	= 0x02,
	NVME_FEAT_LBA_RANGE	= 0x03,
	NVME_FEAT_TEMP_THRESH	= 0x04,
	NVME_FEAT_ERR_RECOVERY	= 0x05,
	NVME_FEAT_VOLATILE_WC	= 0x06,
	NVME_FEAT_NUM_QUEUES	= 0x07,
	NVME_FEAT_IRQ_COALESCE	= 0x08,
	NVME_FEAT_IRQ_CONFIG	= 0x09,
	NVME_FEAT_WRITE_ATOMIC	= 0x0a,
	NVME_FEAT_ASYNC_EVENT	= 0x0b,
	NVME_FEAT_AUTO_PST	= 0x0c,
	NVME_FEAT_HOST_MEM_BUF	= 0x0d,
	NVME_FEAT_TIMESTAMP	= 0x0e,
	NVME_FEAT_KATO		= 0x0f,
	NVME_FEAT_HCTM		= 0x10,
	NVME_FEAT_NOPSC		= 0x11,
	NVME_FEAT_RRL		= 0x12,
	NVME_FEAT_PLM_CONFIG	= 0x13,
	NVME_FEAT_PLM_WINDOW	= 0x14,
	NVME_FEAT_HOST_BEHAVIOR	= 0x16,
	NVME_FEAT_SANITIZE	= 0x17,
	NVME_FEAT_SW_PROGRESS	= 0x80,
	NVME_FEAT_HOST_ID	= 0x81,
	NVME_FEAT_RESV_MASK	= 0x82,
	NVME_FEAT_RESV_PERSIST	= 0x83,
	NVME_FEAT_WRITE_PROTECT	= 0x84,
	NVME_FEAT_VENDOR_START	= 0xC0,
	NVME_FEAT_VENDOR_END	= 0xFF,
	NVME_LOG_ERROR		= 0x01,
	NVME_LOG_SMART		= 0x02,
	NVME_LOG_FW_SLOT	= 0x03,
	NVME_LOG_CHANGED_NS	= 0x04,
	NVME_LOG_CMD_EFFECTS	= 0x05,
	NVME_LOG_DEVICE_SELF_TEST = 0x06,
	NVME_LOG_TELEMETRY_HOST = 0x07,
	NVME_LOG_TELEMETRY_CTRL = 0x08,
	NVME_LOG_ENDURANCE_GROUP = 0x09,
	NVME_LOG_ANA		= 0x0c,
	NVME_LOG_DISC		= 0x70,
	NVME_LOG_RESERVATION	= 0x80,
	NVME_FWACT_REPL		= (0 << 3),
	NVME_FWACT_REPL_ACTV	= (1 << 3),
	NVME_FWACT_ACTV		= (2 << 3),
};