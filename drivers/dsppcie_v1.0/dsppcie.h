#ifndef _DSPPCIE_H_
#define _DSPPCIE_H_

#ifndef DSPPCIE_CDEV_MAJOR
#define DSPPCIE_CDEV_MAJOR   0   /* dynamic major by default */
#endif

#define DSPPCIE_CDEV_NAME    "dsppcie"

/* Must select the endianess */
#define BIG_ENDIAN          0

#if BIG_ENDIAN
#define myIoread32  ioread32be
#define myIowrite32 iowrite32be
#else
#define myIoread32  ioread32 
#define myIowrite32 iowrite32
#endif

#define MAGIC_ADDR          0x0087FFFC

/* PCIE registers */
#define PCIE_BASE_ADDRESS            0x21800000
#define OB_SIZE                      0x30
#define PRIORITY                     0x3C
#define EP_IRQ_CLR                   0x68
#define EP_IRQ_STATUS                0x6C
#define LEGACY_A_IRQ_STATUS_RAW      0x180
#define LEGACY_A_IRQ_ENABLE_SET      0x188
#define LEGACY_A_IRQ_ENABLE_CLR      0x18C
#define OB_OFFSET_INDEX(n)           (0x200 + (8 * (n)))
#define OB_OFFSET_HI(n)              (0x204 + (8 * (n)))
#define IB_BAR(n)                    (0x300 + (0x10 * (n)))
#define IB_START_LO(n)               (0x304 + (0x10 * (n)))
#define IB_START_HI(n)               (0x308 + (0x10 * (n)))
#define IB_OFFSET(n)                 (0x30C + (0x10 * (n)))

#define PCIE_TI_VENDOR               0x104C
#define PCIE_TI_DEVICE               0xB005

#define LL2_START                    0x00800000
#define MSMC_START                   0x0C000000  /* Shared L2 */
#define DDR_START                    0x80000000
#define PCIE_DATA                    0x60000000 

 /* PSC registers */
#define PSC_BASE_ADDRESS             0x02350000
#define PTCMD                        0x120
#define PTSTAT                       0x128
#define PDSTAT(n)                    (0x200 + (4 * (n)))
#define PDCTL(n)                     (0x300 + (4 * (n)))
#define MDSTAT(n)                    (0x800 + (4 * (n)))
#define MDCTL(n)                     (0xA00 + (4 * (n))) 

/* EDMA registers */
#define EDMA_TPCC0_BASE_ADDRESS      0x02700000
#define DMAQNUM0                     0x0240  
#define ESR                          0x1010 
#define EESR                         0x1030                 
#define IESR                         0x1060
#define IPR                          0x1068 
#define ICR                          0x1070 
#define PARAM_0_OPT                  0x4000
#define PARAM_0_SRC                  0x4004
#define PARAM_0_A_B_CNT              0x4008
#define PARAM_0_DST                  0x400C
#define PARAM_0_SRC_DST_BIDX         0x4010
#define PARAM_0_LINK_BCNTRLD         0x4014
#define PARAM_0_SRC_DST_CIDX         0x4018
#define PARAM_0_CCNT                 0x401C

/* Chip level registers */
#define CHIP_LEVEL_BASE_ADDRESS      0x02620000
#define KICK0                        0x38    
#define KICK1                        0x3C
#define KICK0_UNLOCK                 0x83E70B13
#define KICK1_UNLOCK                 0x95A4F1E0 
#define KICK_LOCK                    0x0
#define DSP_BOOT_ADDR(n)             (0x040 + (4 * (n)))
#define IPCGR(n)                     (0x240 + (4 * (n)))


#define LL2_START                    0x00800000
#define MSMC_START                   0x0C000000  /* Shared L2 */
#define DDR_START                    0x80000000
#define PCIE_DATA                    0x60000000  

/* Block size in bytes when r/w data between GPP and DSP via DSP CPU */
#define BLOCK_TRANSFER_SIZE          0x100      

/* Data size in bytes when r/w data bewteen GPP and DSP via EDMA:
   GPP----PCIE link----PCIE data space----EDMA----DSP device memory (L2, DDR, ...) */
#define DMA_TRANSFER_SIZE            0x400000   /* 4MB */

/* Payload size in bytes over PCIE link. PCIe module supports 
   outbound payload size of 128 bytes and inbound payload size of 256 bytes */
#define PCIE_TRANSFER_SIZE           0x80               

/* For 1MB outbound translation window size */
#define PCIE_ADLEN_1MB               0x00100000
#define PCIE_1MB_BITMASK             0xFFF00000

#define PSC_SWRSTDISABLE             0x0
#define PSC_ENABLE                   0x3

#define LOC_RST_ASSERT               0x0
#define LOC_RST_DEASSERT             0x1

/* Power domains definitions */
#define PD0         0     // Power Domain-0
#define PD1         1     // Power Domain-1
#define PD2         2     // Power Domain-2
#define PD3         3     // Power Domain-3
#define PD4         4     // Power Domain-4
#define PD5         5     // Power Domain-5
#define PD6         6     // Power Domain-6
#define PD7         7     // Power Domain-7
#define PD8         8     // Power Domain-8
#define PD9         9     // Power Domain-9
#define PD10        10    // Power Domain-10
#define PD11        11    // Power Domain-11
#define PD12        12    // Power Domain-12
#define PD13        13    // Power Domain-13
#define PD14        14    // Power Domain-14
#define PD15        15    // Power Domain-15
#define PD16        16    // Power Domain-16
#define PD17        17    // Power Domain-17

/* Modules on power domain 0 */
#define LPSC_EMIF16_SPI  3  
#define LPSC_TSIP        4

/* Modules on power domain 1 */
#define LPSC_DEBUG       5
#define LPSC_TETB_TRC    6

/* Modules on power domain 2 */
#define LPSC_PA          7  
#define LPSC_SGMII       8  
#define LPSC_SA          9  

/* Modules on power domain 3 */
#define LPSC_PCIE        10

/* Modules on power domain 4 */
#define LPSC_SRIO        11

/* Modules on power domain 5 */
#define LPSC_HYPER       12

/* Modules on power domain 6 */
#define LPSC_RESERV      13

/* Modules on power domain 7 */
#define LPSC_MSMCRAM     14

/* Modules on power domain 8 */
#define LPSC_C0_TIM0     15

/* Modules on power domain 9 */
#define LPSC_C1_TIM1     16

/* Modules on power domain 10 */
#define LPSC_C2_TIM2     17

/* Modules on power domain 11 */
#define LPSC_C3_TIM3     18

/* Modules on power domain 12 */
#define LPSC_C4_TIM4     19

/* Modules on power domain 13 */
#define LPSC_C5_TIM5     20

/* Modules on power domain 14 */
#define LPSC_C6_TIM6     21

/* Modules on power domain 15 */
#define LPSC_C7_TIM7     22

struct dsppcie_dev {
    /*
     * 注册PCI设备和字符驱动设备的时候使用
     */
    struct pci_dev  *pci_dev;   /* the kernel pci device data structure proviced by probe() */
    /* character device */
    dev_t cdevno;
    struct cdev cdev;

    int irqNo;    /* 中断号 */
    int irq_count;  /* interrupt count, increamented by the interrupt handler */

    /*
     * BAR0-BAR5寄存器值是PC在通过PCIe链路枚举DSP板卡时写入的，这是PC为DSP板卡在PC端
     * 分配的地址空间(PCI总线地址空间，也就是PC内存中实际的物理地址)，它的大小和DSP板卡
     * 内部对应的BAR Mask寄存器配置有关。DSP板卡设置为EP(End Point)模式时共有6组BAR
     * 可用，下面只用到了BAR0-BAR3四组，对应的关系如下：
     */
    resource_size_t appreg_bus_addr;    /* BAR0: 4K for PCIE application registers */
    resource_size_t localL2_bus_addr;   /* BAR1: 512K for core 0 local L2 RAM */
    resource_size_t msmc_bus_addr;      /* BAR2: 4M for shared L2 */
    resource_size_t ddr_bus_addr;       /* BAR3: 16M for DDR3 */

    /*
     * BAR0-BAR3四组寄存器映射的PCI总线地址对应的PC端的虚拟地址
     */
    uint32_t *appreg_virt_addr;
    uint32_t *localL2_virt_addr;
    uint32_t *msmc_virt_addr;
    uint32_t *ddr_vrit_addr;

    /* BAR0-BAR3四组寄存器映射的PCI总线地址空间的长度，字节为单位 */
    resource_size_t appreg_len;
    resource_size_t localL2_len;
    resource_size_t msms_len;
    resource_size_t ddr_len;

    dma_addr_t data_bus_addr;      /* bus address of the allocated buffer */
    uint8_t *data_virt_addr;        /* virtual address of the allocated buffer */
    dma_addr_t rcv_data_bus_addr;      /* bus address of the allocated buffer */
    uint8_t *rcv_data_virt_addr;        /* virtual address of the allocated buffer */uint32_t data_entry_from_dsp;
    uint32_t data_buf_size;
};

static void PCI_readBAR(struct pci_dev *dev);
static void PCI_set_master(struct pci_dev *dev);
static irqreturn_t dsppcie_isr(int irq, void *dev_id);
static void HAL_PciEnableDspInterrupt(struct dsppcie_dev *dpe);
static void HAL_PciDisableDspInterrupt(struct dsppcie_dev *dpe);
static int probe(struct pci_dev *dev, const struct pci_device_id *id_tables);
static void remove(struct pci_dev *dev);
static void dspLocalReset(struct dsppcie_dev *dpe);
static void coreLocalReset(struct dsppcie_dev *dpe, uint32_t pid, uint32_t mid, uint32_t state);
static void setPscState(struct dsppcie_dev *dpe, uint32_t pid, uint32_t mid, uint32_t state);
static uint32_t setBootAddrIpcgr(struct dsppcie_dev *dpe, uint32_t core, uint32_t addr);
static void pushData(struct dsppcie_dev *dpe, uint8_t *pDspCode, uint8_t coreNum, uint32_t *bootEntryAddr);
static uint32_t byteTo32bits(uint8_t *pDspCode);
static void swap4bytes(uint32_t *pDspCode, uint32_t size);
static uint32_t writeDSPMemory(struct dsppcie_dev *dpe, uint32_t coreNum, uint32_t DSPMemAddr, uint32_t *buffer, uint32_t length);
static uint32_t readDSPMemory(struct dsppcie_dev *dpe, uint32_t coreNum, uint32_t DSPMemAddr, uint32_t *buffer, uint32_t length);
static int dpe_cdev_init(struct dsppcie_dev *dpe);
static void dpe_cdev_exit(struct dsppcie_dev *dpe);
static int dpe_cdev_open(struct inode *inode, struct file *filp);
static int dpe_cdev_release(struct inode *inode, struct file *filp);
static void HAL_readDMA(struct dsppcie_dev *dpe, uint32_t srcAddr, uint32_t dstAddr, uint32_t size, uint32_t flag);
static ssize_t dpe_cdev_read(struct file *filp, char __user *buf, size_t count,loff_t *pos);
static void HAL_writeDMA(struct dsppcie_dev *dpe, uint32_t srcAddr, uint32_t dstAddr, uint32_t size, uint32_t flag);
static ssize_t dpe_cdev_write(struct file *filp, const char __user *buf, size_t count, loff_t *pos);
static bool HAL_CheckPciInterrupt(struct dsppcie_dev *dpe);
static void HAL_PciClearDspInterrupt(struct dsppcie_dev *dpe);
//static void HAL_writeDMA_origin(uint32_t srcAddr, uint32_t dstAddr, uint32_t size, uint32_t flag);

#endif  /* _DSPPCIE_H_ */
