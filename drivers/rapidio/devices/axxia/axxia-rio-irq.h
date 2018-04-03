#ifndef __AXXIA_RIO_IRQ_H__
#define __AXXIA_RIO_IRQ_H__

/**************************sRIO SERDES *****************************/
#define SRIO_PHY_CONTROL0_OFFSET        (0x200)
#define APB2SER_SRIO_PHY0_CFG_OFFSET    (0x1e0)
#define SERDES_CMD0_OFFSET              (0x0)
#define SERDES_CMD1_OFFSET              (0x4)
#define SERDES_READDATA0_OFFSET         (0x8)
#define SERDES_READDATA1_OFFSET         (0xc)

#define SERDES_CMD1_VALID_SHIFT         (31)
#define SERDES_CMD1_HWRITE_SHIFT        (30)
#define SERDES_CMD1_TSHIFT_SHIFT        (26)
#define SERDES_CMD1_HSZIE_SHIFT         (23)
#define SERDES_CMD1_HTRANS_SHIFT        (21)
#define SERDES_CMD1_HADDR_MASK          (0xFFFF)

#define SERDES_READDATA1_TMO_SHIFT       (2)
#define SERDES_READDATA1_HRESP_MASK     (0x3)
/******************************************************************/

/* forward declaration */
struct rio_priv;

#define RIO_MSG_MAX_OB_MBOX_MULTI_ENTRIES  15
#define RIO_MSG_MULTI_SIZE                 0x1000 /* 4Kb */
#define RIO_MSG_SEG_SIZE                   0x0100 /* 256B */
#define RIO_MSG_MAX_MSG_SIZE               RIO_MSG_MULTI_SIZE
#define RIO_MSG_MAX_ENTRIES                1024   /* Default Max descriptor
						     table entries for internal
						     descriptor builds */
#define	RIO_MBOX_TO_IDX(mid)		\
	((mid <= RIO_MAX_RX_MBOX_4KB) ? 0 : 1)
#define	RIO_MBOX_TO_BUF_SIZE(mid)		\
	((mid <= RIO_MAX_RX_MBOX_4KB) ? RIO_MSG_MULTI_SIZE : RIO_MSG_SEG_SIZE)
#define	RIO_OUTB_DME_TO_BUF_SIZE(p, did)	\
	((did < p->num_outb_dmes[0]) ? RIO_MSG_MULTI_SIZE : RIO_MSG_SEG_SIZE)

#define DME_MAX_IB_ENGINES          32
#define     RIO_MAX_IB_DME_MSEG		32
#define     RIO_MAX_IB_DME_SSEG	        0
#define DME_MAX_OB_ENGINES          3
#define     RIO_MAX_OB_DME_MSEG		2
#define     RIO_MAX_OB_DME_SSEG	        1

#define RIO_MAX_TX_MBOX             64
#define     RIO_MAX_TX_MBOX_4KB		3
#define     RIO_MAX_TX_MBOX_256B	63
#define RIO_MAX_RX_MBOX             64
#define     RIO_MAX_RX_MBOX_4KB		3
#define     RIO_MAX_RX_MBOX_256B	63

#define RIO_MSG_MAX_LETTER          4


#define RIO_DESC_USED 0		/* Bit index for rio_msg_desc.state */

struct rio_msg_desc {
/*	unsigned long state;*/
/*	int desc_no;*/
	void __iomem *msg_virt;
	dma_addr_t msg_phys;
	int last;
};

struct rio_msg_dme {
	spinlock_t lock;
	unsigned long state;
	struct kref kref;
	struct rio_priv *priv;
	struct resource dres;
	int sz;
	int entries;
	int write_idx;
	int read_idx;
	int tx_dme_tmo;
	void *dev_id;
	int dme_no;
	int mbox;
	int letter;
	u32 dme_ctrl;
	struct rio_msg_desc *desc;
	struct rio_desc *descriptors;

#ifdef CONFIG_AXXIA_RIO_STAT
	unsigned int desc_done_count;
	unsigned int desc_error_count;
	unsigned int desc_rio_err_count;
	unsigned int desc_axi_err_count;
	unsigned int desc_tmo_err_count;
#endif
} ____cacheline_internodealigned_in_smp;

struct rio_rx_mbox {
	spinlock_t lock;
	unsigned long state;
	int mbox_no;
	struct kref kref;
	struct rio_mport *mport;
	void **virt_buffer[RIO_MSG_MAX_LETTER];
	int last_rx_slot[RIO_MSG_MAX_LETTER];
	int next_rx_slot[RIO_MSG_MAX_LETTER];
	int ring_size;
	struct rio_msg_dme *me[RIO_MSG_MAX_LETTER];
	unsigned int irq_state_mask;
	struct hrtimer tmr;
};

struct rio_tx_mbox {
	spinlock_t lock;
	unsigned long state;
	struct rio_mport *mport;
	int mbox_no;
	int dme_no;
	int ring_size;
	struct rio_msg_dme *me;
	void *dev_id;
	int tx_slot;
#ifdef CONFIG_AXXIA_RIO_STAT
	unsigned int sent_msg_count;
	unsigned int compl_msg_count;
#endif
} ____cacheline_internodealigned_in_smp;

struct rio_tx_dme {
	int	ring_size;
	int	ring_size_free;
	struct rio_msg_dme *me;
	struct hrtimer tmr;
};

#define PW_MSG_WORDS (RIO_PW_MSG_SIZE/sizeof(u32))

struct rio_pw_irq {
	/* Port Write */
	u32 discard_count;
	u32 msg_count;
	u32 msg_wc;
	u32 msg_buffer[PW_MSG_WORDS];
};

#define RIO_IRQ_ENABLED 0
#define RIO_IRQ_ACTIVE  1

#define RIO_DME_MAPPED  1
#define RIO_DME_OPEN    0

#define RIO_MB_OPEN	0
#define RIO_MB_MAPPED	1

struct rio_irq_handler {
	unsigned long state;
/*	struct rio_mport *mport;*/
	u32 irq_enab_reg_addr;
	u32 irq_state_reg_addr;
	u32 irq_state_mask;
	void (*thrd_irq_fn)(struct rio_irq_handler *h/*, u32 state*/);
	void (*release_fn)(struct rio_irq_handler *h);
	void *data;
};

extern unsigned int axxia_hrtimer_delay;
/**********************************************/
/* *********** External Functions *********** */
/**********************************************/

void axxia_rio_port_irq_init(struct rio_mport *mport);
void *axxia_get_inb_message(struct rio_mport *mport, int mbox, int letter,
			      int *sz/*, int *slot, u16 *destid*/);
int axxia_add_inb_buffer(struct rio_mport *mport, int mbox, void *buf);
void axxia_close_inb_mbox(struct rio_mport *mport, int mbox);
int axxia_open_inb_mbox(struct rio_mport *mport, void *dev_id,
			  int mbox, int entries);
int axxia_add_outb_message(struct rio_mport *mport, struct rio_dev *rdev,
			     int mbox_dest, int letter, int flags,
			     void *buffer, size_t len/*, void *cookie*/);
void axxia_close_outb_mbox(struct rio_mport *mport, int mbox_id);
int axxia_open_outb_mbox(struct rio_mport *mport, void *dev_id, int mbox_id,
			 int entries/*, int prio*/);
int axxia_rio_doorbell_send(struct rio_mport *mport,
			      int index, u16 destid, u16 data);
int axxia_rio_pw_enable(struct rio_mport *mport, int enable);
void axxia_rio_port_get_state(struct rio_mport *mport, int cleanup);
int axxia_rio_port_irq_enable(struct rio_mport *mport);
void axxia_rio_port_irq_disable(struct rio_mport *mport);

int axxia_ml_add_outb_message(struct rio_mport *mport, struct rio_dev *rdev,
			     int mbox_dest, void *buffer, size_t len);
void *axxia_ml_get_inb_message(struct rio_mport *mport, int mbox);
int alloc_irq_handler(
	struct rio_irq_handler *h,
	void *data,
	const char *name);

void release_mbox_resources(struct rio_priv *priv, int mbox_id);
void release_irq_handler(struct rio_irq_handler *h);
void db_irq_handler(struct rio_irq_handler *h, u32 state);
extern int axxia_rio_init_sysfs(struct platform_device *dev);
extern void axxia_rio_release_sysfs(struct platform_device *dev);

#if defined(CONFIG_RAPIDIO_HOTPLUG)

int axxia_rio_port_notify_cb(struct rio_mport *mport,
			       int enable,
			       void (*cb)(struct rio_mport *mport));
int axxia_rio_port_op_state(struct rio_mport *mport);

#endif

#endif /* __AXXIA_RIO_IRQ_H__ */
