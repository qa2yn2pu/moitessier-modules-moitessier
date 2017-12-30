/*
    Reading from the device
    ***********************
    
    (1)
    reading from the device using the SPI device file:
    target> head -c 49 /dev/naviDev.spi
    target> cat /dev/naviDev.spi
    
    (2)
    reading from the device using the SPI to PTS bridge:
    target> sudo apt-get install socat
    target> socat -d -d pty,raw,echo=0 pty,raw,echo=0
    
    Output:
    2017/11/14 19:00:15 socat[1924] N PTY is /dev/pts/5
    2017/11/14 19:00:15 socat[1924] N PTY is /dev/pts/6
    2017/11/14 19:00:15 socat[1924] N starting data transfer loop with FDs [5,5] and [7,7]
    
    Open your navigation program (e.g OpenCPN) and define /dev/pts/6 (depending of socat output) as input interface and transfer data
    from the SPI device to the PTS device.
    target> cat /dev/naviDev.spi > /dev/pts/5
    
    (3)
    read from the device using the TTY device file:
    target> cat /dev/naviDev.tty
    
    
    Compiling the device tree
    *************************
    
    host> make -j2 ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- dtbs
    host> sudo scp arch/arm/boot/dts/*.dtb pi@10.0.0.55:/media/pi/boot
    
    Required device tree entry in bcm2710-rpi-3-b.dts: 
        &spi0 {
            /delete-node/ spidev@0;
            navidev0: navidev@0{
        		compatible = "pe,navidev";
        		#address-cells = <1>;
        		#size-cells = <0>;
                reg = <0>;
        		spi-max-frequency = <12000000>;
                spi-min-frequency = <1000000>;
                status = "okay";

                boot: boot{
                    gpios = <&gpio 17 GPIO_ACTIVE_LOW>;
                    pe,name = "IMC BOOT";
                    pe,direction = "output";
                    status = "okay";   
                };

                reset: reset{
                    gpios = <&gpio 18 GPIO_ACTIVE_LOW>;
                    pe,name = "IMC RESET";
                    pe,direction = "output";
                    status = "okay";   
                };

                imc_req: imc_req{
                    gpios = <&gpio 24 GPIO_ACTIVE_LOW>, <&gpio 22 GPIO_ACTIVE_LOW>;
                    pe,name = "IMC REQ";
                    pe,direction = "output";
                    status = "okay";   
                };

                imc_irq: imc_irq{
                    gpios = <&gpio 23 GPIO_ACTIVE_LOW>, <&gpio 27 GPIO_ACTIVE_LOW>;
                    pe,name = "IMC IRQ";
                    pe,direction = "input";
                    status = "okay";   
                };
        	};
        };
        
        
        Compiling the driver
        ********************
        
        Makefile:
        obj-m += naviDev_driver.o
        PWD := $(shell pwd)
        all:
        	make -C $(KDIR) M=$(PWD) clean
        	reset
        	make -C $(KDIR) M=$(PWD) modules
        clean:
        	make -C $(KDIR) M=$(PWD) clean

        
        
        host> make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- all KDIR=../kernel/linux


*/

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////// DRIVER CONFIGURATION
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define USE_TTY                 /* Enables the SPI to TTY bridge. Data can be read from either /dev/naviDev.spi or /dev/naviDev.tty. */
#define USE_STATIC_BUF_SIZE     /* enables static buffer allocation, otherwise buffers will be dynamically allocated */
                                /* use static implementation, might need to fix issues with dynamic memory allocation */
#define USE_THREAD              /* enables a kernel thread for reading data via SPI, otherwise data is read in a tasklet */
#define USE_STATIC_FIFO_SIZE    /* use static implementation, might need to fix issues with dynamic memory allocation */
//#define USE_MSG_CNT_TTY
#define MIN_DATA_LENGTH         10
#define USE_STATISTICS          /* collecting statistics enabled */

#define DEVICE_NAME_SPI         "naviDev.spi"           /* the name of the device using SPI --> /dev/naviDev.spi */
#define DEVICE_NAME_CTRL        "naviDev.ctrl"          
#if defined(USE_TTY)
#define DEVICE_NAME_TTY         "naviDev.tty"           /* the name of the device using TTY --> /dev/naviDev.tty */
#endif /* USE_TTY */

#define USE_REQ_FOR_RESET       /* Uses one of the REQ signals to reset the HAT. If not defined the reset signal will be used. */
                                /* You can't use the reset signal if you have a STLink attached to the HAT. */

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////// INCLUDES
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/kthread.h>
#include <linux/jiffies.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>  
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <stddef.h>

#if defined(USE_TTY)
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#endif /* USE_TTY */

#include "header.h"

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////// DEFINES
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#ifdef __GNUC__
#define FUNC_NOT_USED __attribute__ ((unused))
#else
#define FUNC_NOT_USED
#endif

#define DIM(x)  (sizeof(x)/sizeof(*(x)))
#define DIM_ELEMENT(type, element) sizeof(((type *)0)->element)

/* defines the logic level for the request signals */
#define REQ_LEVEL_ACTIVE        1
#define REQ_LEVEL_INACTIVE      0

#define RESET_LEVEL_ACTIVE          0
#define RESET_LEVEL_INACTIVE       1


#if defined(USE_STATIC_BUF_SIZE)
#define BUF_SIZE    1024                /* this must have the same size as used by the HAT microcontroller and relates to the
                                           static SPI DMA transmission size in bytes */
#else /* !USE_STATIC_BUF_SIZE */
static unsigned BUF_SIZE = 1024;
module_param(BUF_SIZE, uint, S_IRUGO);
MODULE_PARM_DESC(BUF_SIZE, "data bytes in a SPI DMA transmission");
#endif /* USE_STATIC_BUF_SIZE */


#if defined(USE_STATIC_FIFO_SIZE)
#define FIFO_SIZE    4096               /* Defines the size in bytes of the FIFO used to buffer received data from the HAT. The FIFO
                                           is read and reset by a read(...) operation from the user on device specified by DEVICE_NAME_SPI. 
                                           It should be larger than BUF_SIZE. */
#else /* !USE_STATIC_FIFO_SIZE */
static unsigned int FIFO_SIZE = 4096;
module_param(FIFO_SIZE, uint, 0444);
MODULE_PARM_DESC(FIFO_SIZE, "the size of the FIFO to buffer received SPI data");
#endif /* USE_STATIC_FIFO_SIZE */

/* defines the debug level --> the higher the level the more kernel messages will be written */
#define LEVEL_NONE          14
#define LEVEL_DEBUG         13
#define LEVEL_INFO          12
#define LEVEL_WARNING       11
#define LEVEL_CRITICAL      10
#define LEVEL_DEBUG_TTY     1
#define LEVEL_DEBUG_SPI     0

int DEBUG_LEVEL = LEVEL_WARNING;

/* creates the file /sys/module/<FILE_NAME>/parameters/DEBUG_LEVEL */
/* The parameter DEBUG_LEVEL can be written and read during runtime.
   To set the debug level for example to LEVEL_CRITICAL, execute the following command:
   target> echo 4 > /sys/module/<FILE_NAME>/parameters/DEBUG_LEVEL
   
   DEBUG_LEVEL might be directly set during module loading:
   target> insmod <FILE_NAME>.ko DEBUG_LEVEL=4 */  
module_param(DEBUG_LEVEL, int, 0644);
/* The description of the module parameters can be shown using modinfo <FILE_NAME>.ko */
MODULE_PARM_DESC(DEBUG_LEVEL, "Defines the level of debugging. \
                               \n\t\t\t14...no debug messages \
                               \n\t\t\t13...debug, info, warning, critical messages \
                               \n\t\t\t12...info, warning, critical messages \
                               \n\t\t\t11...warning, critical messages \
                               \n\t\t\t10...critical messages\n");     

/* Defines the debounce timeout for the GPIOs. This is only required, if the GPIOs are connected
   to push buttons. */                              
long DEBOUNCE_MS = 0;
module_param(DEBOUNCE_MS, long, 0664);
MODULE_PARM_DESC(DEBOUNCE_MS, "GPIO input debounce in ms");  

/* error codes */
#define ERR_NO                  0        
#define ERR_NO_DAT_AVAIL        1
#define ERR_SIG_CHANCEL         2
#define ERR_EOF                 3

#define IOC_MAGIC 'N'
//#define IOCTL_GET_STATISTICS    _IO(IOC_MAGIC,0)
//#define IOCTL_RESET_HAT         _IO(IOC_MAGIC,1)
#define IOCTL_GET_STATISTICS        _IO(IOC_MAGIC,0)
#define IOCTL_GET_INFO              _IO(IOC_MAGIC,1)
#define IOCTL_RESET_HAT             _IO(IOC_MAGIC,2)
#define IOCTL_RESET_STATISTICS      _IO(IOC_MAGIC,3)
#define IOCTL_GNSS                  _IO(IOC_MAGIC,4)

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////// DATA TYPE DEFINITIONS
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

struct ctrl_gpio{
    int gpio;
    int flags;
    int irq;
    int activeState;
    int initState;
    const char* name;
    int nr;
    unsigned long jiffiesEvent;
};

struct st_naviDevSpi{
	struct spi_device	    *spi;               /* the SPI device used for this driver */
#if defined(USE_STATIC_BUF_SIZE)	
    unsigned char	        txBuf[BUF_SIZE];    /* tx buffer used for SPI transmissions */
	unsigned char			rxBuf[BUF_SIZE];    /* rx buffer used for SPI receptions */
#else /* !USE_STATIC_BUF_SIZE */
	unsigned char	        *txBuf;             /* tx buffer used for SPI transmissions */
	unsigned char			*rxBuf;             /* rx buffer used for SPI receptions */
#endif /* !USE_STATIC_BUF_SIZE */
	unsigned int			speed_hz;           /* the baudrate used for the SPI */
    struct ctrl_gpio        req_gpio[2];        /* request signals (outputs) */
    struct ctrl_gpio        irq_gpio[2];        /* interrupt signals (inputs) */
    struct ctrl_gpio        boot_gpio;          /* signal to enable boot mode for the HAT */
    struct ctrl_gpio        reset_gpio;         /* signal to reset the HAT */
    spinlock_t              spinlock;           /* locks this structure */
    bool                    initialized;        /* is true if the device has been properly configured/initialized */
    bool                    opened;             /* is true if the device has been opened by a user */
};

#if defined(USE_STATISTICS)
struct st_statistics{
    uint64_t                spiTxCycles;   
    uint64_t                spiRxCycles;
    uint64_t                totalRxPayloadBytes;
    uint64_t                fifoOverflows;
    uint64_t                fifoBytesProcessed;
    uint64_t                payloadCrcErrors;
    uint64_t                headerCrcErrors;
    spinlock_t              spinlock;		        /* locks this structure */
};

#define NUM_RCV_CHANNELS            2

typedef struct st_receiverConfig
{
    uint8_t         metaDataMask;                       
    uint32_t        afcRange;                           
    uint32_t        afcRangeDefault;                    
    uint32_t        tcxoFreq;                           
    uint32_t        channelFreq[NUM_RCV_CHANNELS];      
};

struct st_info_rcv{
    struct st_receiverConfig    config;
    uint8_t                     rng[2];
};

struct st_info_serial{
    uint32_t    h;      /* bits 95...64 */
    uint32_t    m;      /* bits 63...32 */
    uint32_t    l;      /* bits 31...0 */
};

struct st_info{
    uint8_t                     mode;
    uint8_t                     hwId[16];
    uint8_t                     hwVer[8];
    uint8_t                     bootVer[22];
    uint8_t                     appVer[22];
    uint32_t                    functionality;
    struct st_info_serial       serial;
    struct st_info_rcv          rcv[2];
    bool        valid;
    spinlock_t  spinlock;
};

struct st_config{
    uint8_t     gnssEnabled;
    spinlock_t  spinlock;
};
#endif /* USE_STATISTICS */

struct st_fifo{
#if defined(USE_STATIC_FIFO_SIZE)
    unsigned char           data[FIFO_SIZE];
#else /* !USE_STATIC_FIFO_SIZE */
    unsigned char           *data;
#endif /* !USE_STATIC_FIFO_SIZE */       
    unsigned int            size;
    unsigned int            cnt;
    spinlock_t              spinlock;
};

#if defined(USE_TTY)
struct st_naviDevSpi_serial {
	struct tty_struct	    *tty;		        /* pointer to the tty for this device */
	struct mutex	        lock;		        /* locks this structure */
	bool                    initialized;        /* is true if the device has been properly configured/initialized */
	bool                    opened;             /* is true if the device has been opened by a user */
};
#endif /* USE_TTY */

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////// GLOBAL LOCAL VARIABLES
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

struct st_naviDevSpi	        *naviDev_spi = NULL;
static dev_t                    naviDev_spi_nr;
static struct cdev              *naviDev_spi_object;
static struct class             *naviDev_spi_class;
static struct device            *naviDev_spi_dev;

#if defined(USE_STATISTICS)
static dev_t                    naviDev_stat_nr;
static struct cdev              *naviDev_stat_object;
static struct class             *naviDev_stat_class;
static struct device            *naviDev_stat_dev;
static struct st_statistics     statistics;
static struct st_info           info;
static struct st_config         config;
#endif /* USE_STATISTICS */


DEFINE_MUTEX(mutex_adcVal);
static DECLARE_COMPLETION(on_exit);

static wait_queue_head_t        wq_read;
static wait_queue_head_t        wq_thread;
static struct task_struct       *thread_id;

static void irq_tasklet(unsigned long data);
DECLARE_TASKLET(irq_tl, irq_tasklet, 0);

static atomic_t irqOccured=ATOMIC_INIT(0);

struct st_fifo rxFifo;

static atomic_t reqReceived = ATOMIC_INIT(0);
#define IS_SLAVE_REQ_RECEIVED           (atomic_read(&reqReceived) != 0)
#define SLAVE_REQ_CONFIRMED             atomic_set(&reqReceived, 0) 
#define SLAVE_REQ_RECEIVED              atomic_set(&reqReceived, 1)

static atomic_t dataAvailForUser = ATOMIC_INIT(0);
#define DATA_AVAILABLE_USER             (atomic_read(&dataAvailForUser) != 0)
#define RESET_DATA_AVAILABLE_USER       atomic_set(&dataAvailForUser, 0) 
#define SET_DATA_AVAILABLE_USER         atomic_set(&dataAvailForUser, 1)


#if defined(USE_TTY)
static struct st_naviDevSpi_serial *naviDev_serial;	/* initially all NULL */
static struct class *naviDev_tty_class;
static struct device *naviDev_tty_dev;
#define NAVIDEV_TTY_MAJOR		240	/* experimental range */
#endif /* USE_TTY */


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////// FUNCTION PROTOTYPES
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static int naviDev_transmitSpiData(struct st_naviDevSpi *dev, char* data, unsigned int len, bool devBuf);
static int naviDev_receiveSpiData(struct st_naviDevSpi *dev, size_t len);


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////// FUNCTIONS
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/* this parameter must correlate with the used device tree, otherwise the device will not initialized */
static const struct of_device_id naviDev_dt_ids[] = {
	{ .compatible = "pe,navidev" },
	{},
};
MODULE_DEVICE_TABLE(of, naviDev_dt_ids);

/* user triggered function, read from the device */
/* returns the number of bytes read, or 0 if EOF reached */
static ssize_t naviDev_read(struct file *filp, char __user *buf, size_t maxBytesToRead, loff_t *f_pos)
{
    ssize_t status = 0;
    size_t to_copy, not_copied;
    struct st_naviDevSpi *dev;

    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
        pr_info("%s - max bytes to read: %d\n", __func__, maxBytesToRead);
    
    dev = filp->private_data;
    //maxBytesToRead = 10;
    
	/* chipselect only toggles at start or end of operation */
	/*if (maxBytesToRead > BUF_SIZE)
		return -EMSGSIZE;
	*/	

#if !defined(USE_TTY)
   //    gpio_set_value(naviDev_spi->req_gpio[1].gpio, REQ_LEVEL_INACTIVE);
#endif /* !USE_TTY */
    		
    if(!DATA_AVAILABLE_USER)
    {
        if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
            pr_info("%s - currently no data available\n", __func__);
        //no data available and device opened in non blocking mode
        if((filp->f_flags & O_NONBLOCK))
        {
            put_user(ERR_NO_DAT_AVAIL, buf); 
            status = -EAGAIN;                                                           
            goto exit_read;
        }
    
        //interrupted by a signal while sleeping
        if(wait_event_interruptible(wq_read, DATA_AVAILABLE_USER))
        {
            put_user(ERR_SIG_CHANCEL, buf); 
    	    status = -ERESTARTSYS;                                                           
            goto exit_read;  
    	}
    }	
    
    RESET_DATA_AVAILABLE_USER;
    
#if !defined(USE_TTY)
    spin_lock_irq(&dev->spinlock);
    gpio_set_value(dev->req_gpio[1].gpio, REQ_LEVEL_INACTIVE);	
    spin_unlock_irq(&dev->spinlock);
#endif /* !USE_TTY */		

	spin_lock_irq(&rxFifo.spinlock);
	to_copy = min(rxFifo.cnt, maxBytesToRead);
	not_copied = copy_to_user(buf, rxFifo.data, to_copy);
	
#if defined(USE_STATISTICS)	
	spin_lock_irq(&statistics.spinlock);
    statistics.fifoBytesProcessed += to_copy;
    spin_unlock_irq(&statistics.spinlock);
#endif /* USE_STATISTICS */
	
	/* complete FIFO read? */
	if(to_copy != rxFifo.cnt && to_copy < rxFifo.cnt)
	{
	   memcpy(&rxFifo.data[0], &rxFifo.data[to_copy], rxFifo.cnt - to_copy);
	   rxFifo.cnt -= to_copy;
	}
	else
	{ 
	    rxFifo.cnt = 0;
	}
	spin_unlock_irq(&rxFifo.spinlock);
	
	if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
	    pr_info("%s - bytes read: %d\n", __func__, to_copy);
	
	
	if(not_copied == to_copy)
		status = -EFAULT;
	else
		status = to_copy - not_copied;
		
	return status;
	
exit_read:       
    return status;     	
}

/* user triggered function, write data to the device */
/* returns the actual number of bytes written */
static ssize_t naviDev_write(struct file *filp, const char __user *buf, size_t maxBytesToWrite, loff_t *f_pos)
{
    unsigned long not_copied;
    ssize_t status = 0;
    struct st_naviDevSpi *dev;
    
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
	    pr_info("%s\n", __func__);
	
	dev = filp->private_data;
	
	if (maxBytesToWrite > BUF_SIZE)
		return -EMSGSIZE;

	spin_lock_irq(&dev->spinlock);
	not_copied = copy_from_user(dev->txBuf, buf, maxBytesToWrite);
	if(not_copied == 0)
		status = naviDev_transmitSpiData(dev, NULL, maxBytesToWrite, true);
	else
		status = -EFAULT;
	spin_unlock_irq(&dev->spinlock);
	
	return maxBytesToWrite;
}

/* user triggered function, open the device */
/* returns 0 on success or < 0 on an error */
static int naviDev_open(struct inode *inode, struct file *filp)
{
    struct st_naviDevSpi *dev = naviDev_spi;
    int status = -EIO;
      
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)  
	    pr_info("%s\n", __func__);
	
	spin_lock_irq(&dev->spinlock);
	
	/* we can only open the device once */
	if(dev->opened)
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
		    pr_err("%s - device already opened\n", __func__);
		status = -EIO;
		goto open_failed;
	}
	
	/* we store the device structure to this instance so we can later access it in read(...) and write(...) */
	filp->private_data = dev;
	
#if !defined(USE_STATIC_BUF_SIZE)	
	if(!dev->txBuf)
	{
		dev->txBuf = kmalloc(BUF_SIZE, GFP_KERNEL);
		if(!dev->txBuf)
		{
			dev_dbg(&dev->spi->dev, "open/ENOMEM\n");
			status = -ENOMEM;
			if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
			    pr_err("%s - allocating memory for txBuf failed\n", __func__);
			goto open_failed;
		}
	}

	if(!dev->rxBuf)
	{
		dev->rxBuf = kmalloc(BUF_SIZE, GFP_KERNEL);
		if(!dev->rxBuf)
		{
			dev_dbg(&dev->spi->dev, "open/ENOMEM\n");
			status = -ENOMEM;
			if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
			    pr_err("%s - allocating memory for rxBuf failed\n", __func__);
			goto free_txBuf;
		}
	}
#endif /* !USE_STATIC_BUF_SIZE */	

    dev->opened = true;
	
	spin_unlock_irq(&dev->spinlock);
	
	return 0;

#if !defined(USE_STATIC_BUF_SIZE)	
free_txBuf:
	kfree(dev->txBuf);
	dev->txBuf = NULL;
	dev->rxBuf = NULL;
#endif /* !USE_STATIC_BUF_SIZE */	
open_failed:
    spin_unlock_irq(&dev->spinlock);
	return status;		
}

void naviDev_resetHAT(uint32_t ms)
{
    if(DEBUG_LEVEL >= LEVEL_DEBUG)
        pr_info("%s\n", __func__);
    
    spin_lock_irq(&naviDev_spi->spinlock);
    /* reset the HAT microcontroller */
#if defined(USE_REQ_FOR_RESET)    
    gpio_set_value(naviDev_spi->req_gpio[0].gpio, REQ_LEVEL_INACTIVE);
    gpio_set_value(naviDev_spi->req_gpio[0].gpio, REQ_LEVEL_ACTIVE);
    mdelay(ms);
    gpio_set_value(naviDev_spi->req_gpio[0].gpio, REQ_LEVEL_INACTIVE);
#else
    gpio_set_value(naviDev_spi->reset_gpio.gpio, RESET_LEVEL_INACTIVE);
    gpio_set_value(naviDev_spi->reset_gpio.gpio, RESET_LEVEL_ACTIVE);
    mdelay(ms);
    gpio_set_value(naviDev_spi->reset_gpio.gpio, RESET_LEVEL_INACTIVE);
#endif /* !USE_REQ_FOR_RESET */       
    spin_unlock_irq(&naviDev_spi->spinlock);
}

#if defined(USE_STATISTICS)
void naviDev_resetStatistics(struct st_statistics *stat)
{
    if(DEBUG_LEVEL >= LEVEL_DEBUG)
        pr_info("%s\n", __func__);
        
    spin_lock_irq(&stat->spinlock);
    stat->spiTxCycles = 0;   
    stat->spiRxCycles = 0;
    stat->totalRxPayloadBytes = 0;
    stat->fifoOverflows = 0;
    stat->fifoBytesProcessed = 0;
    stat->payloadCrcErrors = 0;
    stat->headerCrcErrors = 0;
    spin_unlock_irq(&stat->spinlock);
}

static long naviDev_stat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    uint8_t dat[256];
    size_t rc = 0;
    int32_t size = 0;
    
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
        pr_info("%s - cmd: %u\n", __func__, cmd);
    
    /* TODO: need to be implemented */    
    
    memset(dat, 0, sizeof(dat));
            
    switch(cmd)
    {
        case IOCTL_GET_STATISTICS:
            spin_lock_irq(&statistics.spinlock);
            memcpy(dat, &statistics, sizeof(struct st_statistics));
            spin_unlock_irq(&statistics.spinlock);
            rc = copy_to_user((void*)arg, dat, sizeof(struct st_statistics));  
            size = sizeof(struct st_statistics);
            break;
        case IOCTL_RESET_HAT:
            naviDev_resetHAT(10);
            break;
        case IOCTL_RESET_STATISTICS:
            naviDev_resetStatistics(&statistics);
            break;
        case IOCTL_GET_INFO:
            spin_lock_irq(&info.spinlock);
            memcpy(dat, &info, sizeof(struct st_info));
            spin_unlock_irq(&info.spinlock);
            rc = copy_to_user((void*)arg, dat, sizeof(struct st_info));
            size = sizeof(struct st_info);
            break;
        case IOCTL_GNSS:
            rc = copy_from_user(dat, (void*)arg, sizeof(uint8_t));
            spin_lock_irq(&config.spinlock);
            config.gnssEnabled = dat[0];
            spin_unlock_irq(&config.spinlock);
            break;
        default:
            if(DEBUG_LEVEL >= LEVEL_CRITICAL)
            {
                pr_err("%s: Unknown parameter.\n", __func__);
            }
            return -EFAULT;
    }
    
    return (size - rc);
}
#endif /* USE_STATISTICS */

static long naviDev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
        pr_info("%s\n", __func__);
	
	/* TODO: need to be implemented */
	return 0;
}

static long naviDev_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
        pr_info("%s\n", __func__);
	
	/* TODO: need to be implemented */
	return 0;
}

/* user triggered function, close the device */
/* returns 0 on success or < 0 on an error */
static int naviDev_release(struct inode *inode, struct file *filp)
{
    struct st_naviDevSpi *dev;
    
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
	    pr_info("%s\n", __func__);

	dev = filp->private_data;
	
    spin_lock_irq(&dev->spinlock);
    
    filp->private_data = NULL;
	dev->opened = false;
	
#if !defined(USE_STATIC_BUF_SIZE)	
	kfree(dev->txBuf);
	dev->txBuf = NULL;

	kfree(dev->rxBuf);
	dev->rxBuf = NULL;
#endif /* !USE_STATIC_BUF_SIZE */
	
	spin_unlock_irq(&dev->spinlock);
		
	return 0;
}

/* user triggered file operations */
static const struct file_operations naviDev_fops = {
	.owner              = THIS_MODULE,
	.write              = naviDev_write,
	.read               = naviDev_read,
	.open               = naviDev_open,
	.release            = naviDev_release,
	.unlocked_ioctl     = naviDev_ioctl,
	.compat_ioctl       = naviDev_compat_ioctl,
};

/* user triggered file operations */
static const struct file_operations naviDev_statfops = {
	.owner              = THIS_MODULE,
	//.open =		naviDev_stat_open,
	//.release =	naviDev_stat_release,
	.unlocked_ioctl     = naviDev_stat_ioctl,
};

/*
static int FUNC_NOT_USED send_byte(struct st_naviDevSpi *dev, int data)
{
	int status;
	struct spi_message msg = { };
	struct spi_transfer transfer = { };

    if(DEBUG_LEVEL >= LEVEL_DEBUG)
	    pr_info("%s\n", __func__);

	spi_message_init(&msg);
	dev->txBuf[0] = data;
	transfer.tx_buf = dev->txBuf;
	transfer.rx_buf = dev->rxBuf;
	transfer.len = 1;
	transfer.speed_hz = dev->speed_hz;
	spi_message_add_tail(&transfer, &msg);
	status = spi_sync(dev->spi, &msg);

	return status;
};
*/

static int FUNC_NOT_USED naviDev_transmitSpiData(struct st_naviDevSpi *dev, char* data, unsigned int len, bool devBuf)
{
	int status;
	struct spi_message msg = { };
	struct spi_transfer transfer = { };

    if(DEBUG_LEVEL >= LEVEL_DEBUG)
	    pr_info("%s\n", __func__);

	spi_message_init(&msg);
	if(!devBuf)
        memcpy(dev->txBuf, data, len);
	transfer.tx_buf = dev->txBuf;
	transfer.rx_buf = dev->rxBuf;
	transfer.len = len;
	transfer.speed_hz = dev->speed_hz;
	spi_message_add_tail(&transfer, &msg);
	status = spi_sync(dev->spi, &msg);

    if (status == 0)
		status = msg.actual_length;
	return status;
};


static int FUNC_NOT_USED naviDev_receiveSpiData(struct st_naviDevSpi *dev, size_t len)
{
    int status;
	struct spi_message msg = { };
	struct spi_transfer transfer = { };
	
	if(DEBUG_LEVEL >= LEVEL_DEBUG)
	    pr_info("%s\n", __func__);
	    
	spi_message_init(&msg);
	/* set the byte that will be used for the transmit signal */
	/* we might change this, if data should be excanged and not only read */
	memset(dev->txBuf, '0', BUF_SIZE);
	
	transfer.tx_buf = dev->txBuf;
	transfer.rx_buf = dev->rxBuf;
	transfer.len = len;
	transfer.speed_hz = dev->speed_hz;
	spi_message_add_tail(&transfer, &msg);
	status = spi_sync(dev->spi, &msg);
	
	if (status == 0)
		status = msg.actual_length;
	
	return status;
}

void naviDev_processReqData(void)
{
    //uint32_t i = 0;
    uint32_t headerPos = 0;
    uint32_t lastHeaderPos = 0;
    header_t header;
    bool dataProcessed = false;
    bool fifoFull = false;
    uint8_t *msg;
    uint32_t headerCnt = 0;
    uint16_t crc;
    bool processData = false;
    uint32_t posPayload = 0;
#if defined(USE_TTY)   
    unsigned int k = 0; 
    struct st_naviDevSpi_serial *serial = NULL;
    struct tty_struct *tty = NULL;
	struct tty_port *port = NULL;	
#if defined(USE_MSG_CNT_TTY)  	
    static unsigned long cnt = 0;
    char cntBuf[100];
#endif /* USE_MSG_CNT_TTY */
#endif /* USE_TTY */	

    if(!naviDev_spi)
        return;
     
    
    spin_lock_irq(&naviDev_spi->spinlock);
    
    /* read data from the SPI interface */
    //naviDev_receiveSpiData(naviDev_spi, BUF_SIZE);

#if defined(USE_STATISTICS)     
    /* we've not received the info of the connected HAT yet */
    if(!info.valid)
    {
        /*HEADER_uint8_tToAsciiHex(CMD_CATEGORY_REQUEST, &naviDev_spi->txBuf[HEADER_size()  + (sizeof(uint8_t) * 2) * 0], NOT_NULL_TERMINATED);
        HEADER_uint8_tToAsciiHex(CMD_INFO, &naviDev_spi->txBuf[HEADER_size() + (sizeof(uint8_t) * 2) * 1], NOT_NULL_TERMINATED);
        HEADER_uint8_tToAsciiHex(CMD_INFO_SYSTEMa, &naviDev_spi->txBuf[HEADER_size() + (sizeof(uint8_t) * 2) * 2], NOT_NULL_TERMINATED);
        HEADER_uint8_tToAsciiHex(CMD_FIELD_NOT_USED, &naviDev_spi->txBuf[HEADER_size() + (sizeof(uint8_t) * 2) * 3], NOT_NULL_TERMINATED);*/
        
        /* create the command/payload that should be transmitted to the SPI slave */
        HEADER_uint8_tToAsciiHex(CMD_CATEGORY_REQUEST, &naviDev_spi->txBuf[HEADER_size()  + (offsetof(request_t, category) * 2)], NOT_NULL_TERMINATED);
        HEADER_uint8_tToAsciiHex(CMD_INFO, &naviDev_spi->txBuf[HEADER_size() + (offsetof(request_t, cmd) * 2)], NOT_NULL_TERMINATED);
        HEADER_uint8_tToAsciiHex(CMD_INFO_SYSTEM, &naviDev_spi->txBuf[HEADER_size() + (offsetof(request_t, cmdSub) * 2)], NOT_NULL_TERMINATED);
        HEADER_uint8_tToAsciiHex(CMD_FIELD_NOT_USED, &naviDev_spi->txBuf[HEADER_size() + (offsetof(request_t, param) * 2)], NOT_NULL_TERMINATED);
        
        /* fill the header for the payload with default settings */
        HEADER_setDefaults(&header);
        
        /* change the required fields in the header */
        header.source = SOURCE_SYSTEM;
        header.target = SOURCE_SYSTEM;
        header.payloadLen = (sizeof(request_t) * 2);
        
        /* calculate the CRC for the payload */
        HEADER_crc(&naviDev_spi->txBuf[HEADER_size()], header.payloadLen, &crc);
        header.payloadCRC = crc;
        
        /* calculate the CRC of the header */
        HEADER_crc((uint8_t*)&header, sizeof(header_t) - sizeof(uint16_t), &crc);
        header.headerCRC = crc;
        
        /* convert the header to ASCII HEX format so it can be transmitted and write it to the appropriate transmit buffer */
        HEADER_set(&header, naviDev_spi->txBuf, BUF_SIZE);      
    }
    /*
    else if(...)
    {
        you might want to add here some additional commands that should be transmitted to the SPI slave 
    }
    */
    else
#endif /* USE_STATISTICS */        
    {
        memset(naviDev_spi->txBuf, 0, BUF_SIZE);
    }
    
    /* start SPI transmission */
    naviDev_transmitSpiData(naviDev_spi, NULL, BUF_SIZE, true);
    
    
#if defined(USE_STATISTICS)    
    spin_lock_irq(&statistics.spinlock);
    statistics.spiRxCycles++;
    spin_unlock_irq(&statistics.spinlock);
#endif /* USE_STATISTICS */
    
    lastHeaderPos = 0;
    headerCnt = 0;
    do
    {
        if(lastHeaderPos >= BUF_SIZE)
            break;
            
        /* search for a header in the received data, if available process the data otherwise leave this function */
        if(HEADER_find(naviDev_spi->rxBuf, BUF_SIZE, lastHeaderPos, &headerPos))
        {
            /* error handling */
            if(DEBUG_LEVEL >= LEVEL_WARNING && !headerCnt)
                pr_err("%s - HEADER_find(...) failed - %u\n", __func__, headerCnt);
            goto naviDev_processReqData_exit;
        }
        
        headerCnt++;
        
        /* read the header data from the receiver buffer and convert the ASCII only data to the appropriate header structure */
        if(HEADER_get(&naviDev_spi->rxBuf[headerPos], BUF_SIZE, &header))
        {
            if(DEBUG_LEVEL >= LEVEL_WARNING)
                pr_err("%s - HEADER_get(...) failed\n", __func__);
            lastHeaderPos += HEADER_SOF_FIELD_LEN;
            continue;
        }
        
        /* check if the header is valid at all, if this fails we can assume data corruption and restart searching for a valid
           header */
        if(HEADER_isValid(&header))
        {
            if(DEBUG_LEVEL >= LEVEL_WARNING)
                pr_err("%s - HEADER_isValid(...) failed\n", __func__);
            lastHeaderPos += HEADER_SOF_FIELD_LEN;
            
#if defined(USE_STATISTICS)            
            spin_lock_irq(&statistics.spinlock);
            statistics.headerCrcErrors++;
            spin_unlock_irq(&statistics.spinlock);
#endif /* USE_STATISTICS */            
            continue;
        }
        
        /* check if the payload is valid, otherwise drop data and restart searching for a valid header */
        if(HEADER_payloadIsValid(&header, &naviDev_spi->rxBuf[headerPos + HEADER_size()]))
        {
            if(DEBUG_LEVEL >= LEVEL_WARNING)
                pr_err("%s - HEADER_payloadIsValid(...) failed\n", __func__);
            
            lastHeaderPos += HEADER_size();

#if defined(USE_STATISTICS)            
            spin_lock_irq(&statistics.spinlock);
            statistics.payloadCrcErrors++;
            spin_unlock_irq(&statistics.spinlock);
#endif /* USE_STATISTICS */            
            continue;
        }
        
        /* this is an optional check... we might comment it out someday */
        if(header.payloadLen < MIN_DATA_LENGTH)
        {
            if(DEBUG_LEVEL >= LEVEL_WARNING)
                pr_err("%s - received too less data bytes (%u)\n", __func__, header.payloadLen);
            
            lastHeaderPos += HEADER_size();
            continue;
        }
        
        /* check if the received message is for the system */
        /* all none system related messages are NMEA messages and start either with $ or ! */
        if(naviDev_spi->rxBuf[headerPos + HEADER_size()] != '$' && 
            naviDev_spi->rxBuf[headerPos + HEADER_size()] != '!')
        {
#if defined(USE_STATISTICS)             
            if(!info.valid)
            {
                posPayload = 0;
                /* the response header is stored in ASCII HEX format in the receive buffer, so we need to multiply by 2 (sizeof(response_t) * 2) to 
                   get the index to the payload */ 
                memcpy(&info.mode, (uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + sizeof(response_t) * 2 + offsetof(struct st_info, mode)], 
                        DIM_ELEMENT(struct st_info, hwId));
                memcpy(info.hwId, (uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + sizeof(response_t) * 2 + offsetof(struct st_info, hwId)], 
                        DIM_ELEMENT(struct st_info, hwId));
                memcpy(info.hwVer, (uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + sizeof(response_t) * 2 + offsetof(struct st_info, hwVer)], 
                        DIM_ELEMENT(struct st_info, hwVer));
                memcpy(info.bootVer, (uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + sizeof(response_t) * 2 + offsetof(struct st_info, bootVer)], 
                        DIM_ELEMENT(struct st_info, bootVer));
                memcpy(info.appVer, (uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + sizeof(response_t) * 2 + offsetof(struct st_info, appVer)],
                        DIM_ELEMENT(struct st_info, appVer));
                posPayload = sizeof(response_t) * 2 + offsetof(struct st_info, appVer) + DIM_ELEMENT(struct st_info, appVer);                       
                info.functionality = HEADER_asciiHexToUint32_t((uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                posPayload += DIM_ELEMENT(struct st_info, functionality) * 2;
                info.serial.h = HEADER_asciiHexToUint32_t((uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                posPayload += DIM_ELEMENT(struct st_info_serial, h) * 2;
                info.serial.m = HEADER_asciiHexToUint32_t((uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                posPayload += DIM_ELEMENT(struct st_info_serial, m) * 2;
                info.serial.l = HEADER_asciiHexToUint32_t((uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                posPayload += DIM_ELEMENT(struct st_info_serial, l) * 2;
                info.rcv[0].config.channelFreq[0] = HEADER_asciiHexToUint32_t((uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                posPayload += DIM_ELEMENT(struct st_receiverConfig, channelFreq[0]) * 2;
                info.rcv[0].config.channelFreq[1] = HEADER_asciiHexToUint32_t((uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                posPayload += DIM_ELEMENT(struct st_receiverConfig, channelFreq[1]) * 2;
                info.rcv[0].config.metaDataMask = HEADER_asciiHexToUint8_t((uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                posPayload += DIM_ELEMENT(struct st_receiverConfig, metaDataMask) * 2;
                info.rcv[0].config.afcRange = HEADER_asciiHexToUint32_t((uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                posPayload += DIM_ELEMENT(struct st_receiverConfig, afcRange) * 2;
                info.rcv[0].config.afcRangeDefault = HEADER_asciiHexToUint32_t((uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                posPayload += DIM_ELEMENT(struct st_receiverConfig, afcRangeDefault) * 2;
                info.rcv[0].config.tcxoFreq = HEADER_asciiHexToUint32_t((uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                posPayload += DIM_ELEMENT(struct st_receiverConfig, tcxoFreq) * 2;
                info.rcv[0].rng[0] = HEADER_asciiHexToUint8_t((uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                posPayload += DIM_ELEMENT(struct st_info_rcv, rng[0]) * 2;
                info.rcv[0].rng[1] = HEADER_asciiHexToUint8_t((uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                posPayload += DIM_ELEMENT(struct st_info_rcv, rng[1]) * 2;
                info.rcv[1].config.channelFreq[0] = HEADER_asciiHexToUint32_t((uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                posPayload += DIM_ELEMENT(struct st_receiverConfig, channelFreq[0]) * 2;
                info.rcv[1].config.channelFreq[1] = HEADER_asciiHexToUint32_t((uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                posPayload += DIM_ELEMENT(struct st_receiverConfig, channelFreq[1]) * 2;
                info.rcv[1].config.metaDataMask = HEADER_asciiHexToUint8_t((uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                posPayload += DIM_ELEMENT(struct st_receiverConfig, metaDataMask) * 2;
                info.rcv[1].config.afcRange = HEADER_asciiHexToUint32_t((uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                posPayload += DIM_ELEMENT(struct st_receiverConfig, afcRange) * 2;
                info.rcv[1].config.afcRangeDefault = HEADER_asciiHexToUint32_t((uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                posPayload += DIM_ELEMENT(struct st_receiverConfig, afcRangeDefault) * 2;
                info.rcv[1].config.tcxoFreq = HEADER_asciiHexToUint32_t((uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                posPayload += DIM_ELEMENT(struct st_receiverConfig, tcxoFreq) * 2;
                info.rcv[1].rng[0] = HEADER_asciiHexToUint8_t((uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                posPayload += DIM_ELEMENT(struct st_info_rcv, rng[0]) * 2;
                info.rcv[1].rng[1] = HEADER_asciiHexToUint8_t((uint8_t*)&naviDev_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                posPayload += DIM_ELEMENT(struct st_info_rcv, rng[1]) * 2;
                
                
                info.valid = true;
            }
#endif /* USE_STATISTICS */            
            lastHeaderPos += (HEADER_size() + header.payloadLen);
            continue;
        }
        
        
        
        
#if defined(USE_STATISTICS)        
        spin_lock_irq(&statistics.spinlock);
        statistics.totalRxPayloadBytes += header.payloadLen;
        spin_unlock_irq(&statistics.spinlock);
#endif /* USE_STATISTICS */    
        
        if(DEBUG_LEVEL >= LEVEL_DEBUG)
        {
            msg = kmalloc(header.payloadLen + 1, GFP_KERNEL);
            if(msg)
            {
                memcpy(msg, &naviDev_spi->rxBuf[headerPos + HEADER_size()], header.payloadLen);
                msg[header.payloadLen] = '\0';
                pr_info("%s - %s", __func__, msg);
                kfree(msg);      
            }
        }
        
        processData = false;
        if(header.payloadLen > 3)
        {
            spin_lock_irq(&config.spinlock);
            if(naviDev_spi->rxBuf[headerPos + HEADER_size() + 0] == '$' && \
               naviDev_spi->rxBuf[headerPos + HEADER_size() + 1] == 'G' && \
               (naviDev_spi->rxBuf[headerPos + HEADER_size() + 2] == 'N' || \
                naviDev_spi->rxBuf[headerPos + HEADER_size() + 2] == 'P') &&\
                !config.gnssEnabled)
            {
                
            }
            else
            {
                processData = true;
            }
            spin_unlock_irq(&config.spinlock);
        }
        
        
        /* we process the data further only, if the device specified in DEVICE_NAME_SPI is opened at all, otherwise data will
           be dropped */
        if(naviDev_spi->opened)
        {
            fifoFull = false;
            
            spin_lock_irq(&rxFifo.spinlock);
    
            /* still space left in FIFO? */
            if((rxFifo.cnt + header.payloadLen) > rxFifo.size)
            {
                if(DEBUG_LEVEL >= LEVEL_WARNING || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
                    pr_err("%s - received too many bytes (%u) to store in fifo (%u bytes left)\n", __func__, header.payloadLen, rxFifo.size - rxFifo.cnt);
                 
                fifoFull = true;
                 
#if defined(USE_STATISTICS)                 
                spin_lock_irq(&statistics.spinlock);
                statistics.fifoOverflows++;
                spin_unlock_irq(&statistics.spinlock);
#endif /* USE_STATISTICS */                
            }

            if(!fifoFull && processData)
            {
                /* append the new data to the FIFO */
                memcpy(&rxFifo.data[rxFifo.cnt], &naviDev_spi->rxBuf[headerPos + HEADER_size()], header.payloadLen); 
                rxFifo.cnt += header.payloadLen;
                dataProcessed = true;
            }
            
            spin_unlock_irq(&rxFifo.spinlock);
            
            if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
            {
                pr_info("%s - FIFO cnt: %u", __func__, rxFifo.cnt);
            }
        }
        
#if defined(USE_TTY)
        serial = naviDev_serial;
                
        mutex_lock(&serial->lock);       
        if(serial)
        {
            /* we need to check if the tty/serial device is already initialized and opened, because this function will probably
               be executed earlier */
            if(serial->initialized && serial->opened && processData)
            {
                tty = serial->tty;
             	port = tty->port;
           
                for (k = 0; k < (header.payloadLen); ++k)
            	{
            		if (!tty_buffer_request_room(port, 1))
                        tty_flip_buffer_push(port);
                    tty_insert_flip_char(port, naviDev_spi->rxBuf[headerPos + HEADER_size() + k], TTY_NORMAL);
                    
            	}
            	//tty_flip_buffer_push(port);
#if defined(USE_MSG_CNT_TTY)          	
            	cnt++;              	
            	sprintf(cntBuf, "%lu\r\n", cnt);
            	for (k = 0; k < strlen(cntBuf); ++k)
            	{
            		if (!tty_buffer_request_room(port, 1))
                        tty_flip_buffer_push(port);
                    tty_insert_flip_char(port, cntBuf[k], TTY_NORMAL);
                    
            	}
#endif /* USE_MSG_CNT_TTY */                	
        	    tty_flip_buffer_push(port);
        	}
        }
        mutex_unlock(&serial->lock);
#endif /* USE_TTY */       
        
        lastHeaderPos += (HEADER_size() + header.payloadLen);
    }while(1);
 
naviDev_processReqData_exit:    
#if defined(USE_TTY)
    gpio_set_value(naviDev_spi->req_gpio[1].gpio, REQ_LEVEL_INACTIVE);	
#endif /* USE_TTY */            
    
    if(dataProcessed)
        SET_DATA_AVAILABLE_USER;

    spin_unlock_irq(&naviDev_spi->spinlock);
    
    if(dataProcessed)
        wake_up_interruptible(&wq_read);
}

static int naviDev_thread(void *data)
{
    allow_signal(SIGTERM);    
    
#if !defined(USE_STATIC_BUF_SIZE)   
    spin_lock_irq(&naviDev_spi->spinlock)     
    naviDev_spi->txBuf = kmalloc(BUF_SIZE, GFP_KERNEL);
    naviDev_spi->rxBuf = kmalloc(BUF_SIZE, GFP_KERNEL);	    
    spin_unlock_irq(&naviDev_spi->spinlock)
#endif /* USE_STATIC_BUF_SIZE */    
    while(kthread_should_stop() == 0)
    {       
        if(wait_event_interruptible(wq_thread, IS_SLAVE_REQ_RECEIVED))
        {
            if(DEBUG_LEVEL >= LEVEL_DEBUG)
                pr_info("%s - signal break\n", __func__);
            break;
        }
        SLAVE_REQ_CONFIRMED;
      
#if !defined(USE_STATIC_BUF_SIZE)
        spin_lock_irq(&naviDev_spi->spinlock)        
        if(naviDev_spi->txBuf && naviDev_spi->rxBuf)
#endif /* !USE_STATIC_BUF_SIZE */            
        {
#if !defined(USE_STATIC_BUF_SIZE)
            spin_unlock_irq(&naviDev_spi->spinlock)            
#endif /* !USE_STATIC_BUF_SIZE */            
            naviDev_processReqData();
        }                 
#if !defined(USE_STATIC_BUF_SIZE)
        spin_unlock_irq(&naviDev_spi->spinlock)            
#endif /* !USE_STATIC_BUF_SIZE */ 
    }
#if !defined(USE_STATIC_BUF_SIZE)
    spin_lock_irq(&naviDev_spi->spinlock)           
    if(naviDev_spi->txBuf)
    {
        kfree(naviDev_spi->txBuf);
        naviDev_spi->txBuf = NULL;
    }
    if(naviDev_spi->rxBuf)
    {
        kfree(naviDev_spi->rxBuf);
        naviDev_spi->rxBuf = NULL;
    }
    spin_unlock_irq(&naviDev_spi->spinlock)        
#endif /* !USE_STATIC_BUF_SIZE */   
    complete_and_exit(&on_exit, 0);
}
   
static void irq_tasklet(unsigned long data)
{
    unsigned int irq = atomic_read(&irqOccured);
    unsigned int i = 0;
    
//    if(DEBUG_LEVEL >= LEVEL_DEBUG)
//        pr_info("%s - %d\n", __func__, irq);
    
    if(!naviDev_spi)
        return;
    
    if(!naviDev_spi->initialized)
        return;
        
    spin_lock(&naviDev_spi->spinlock);    
    
    /* determine which GPIO has caused the interrupt */
    for(i = 0; i < DIM(naviDev_spi->irq_gpio); i++)
    {
        if(naviDev_spi->irq_gpio[i].irq == irq)
            break;
    }
    
    if(i <= DIM(naviDev_spi->irq_gpio))
    {
        if(jiffies < naviDev_spi->irq_gpio[i].jiffiesEvent)
            naviDev_spi->irq_gpio[i].jiffiesEvent = jiffies;
            
        if(((((unsigned long)naviDev_spi->irq_gpio[i].jiffiesEvent + (unsigned long)msecs_to_jiffies(DEBOUNCE_MS)) < (unsigned long)jiffies)) || DEBOUNCE_MS == 0)
        {
            naviDev_spi->irq_gpio[i].jiffiesEvent = jiffies;
        }
            
        if(i == 1 && naviDev_spi->spi)
        {
            gpio_set_value(naviDev_spi->req_gpio[i].gpio, REQ_LEVEL_ACTIVE);
            SLAVE_REQ_RECEIVED;
#if defined(USE_THREAD)            
            wake_up_interruptible(&wq_thread);
#else /* !USE_THREAD */                       
#if !defined(USE_STATIC_BUF_SIZE)            
            naviDev_spi->txBuf = kmalloc(BUF_SIZE, GFP_KERNEL);
            naviDev_spi->rxBuf = kmalloc(BUF_SIZE, GFP_KERNEL);		          
            if(naviDev_spi->txBuf && naviDev_spi->rxBuf)
#endif /* USE_STATIC_BUF_SIZE */                
            {
                spin_unlock(&naviDev_spi->spinlock); 
                naviDev_processReqData();
                spin_lock(&naviDev_spi->spinlock); 
            }

#if !defined(USE_STATIC_BUF_SIZE)            
            if(naviDev_spi->txBuf)
            {
                kfree(naviDev_spi->txBuf);
                naviDev_spi->txBuf = NULL;
            }
            if(naviDev_spi->rxBuf)
            {
                kfree(naviDev_spi->rxBuf);
                naviDev_spi->rxBuf = NULL;
            }
#endif /* !USE_STATIC_BUF_SIZE */                
#endif /* !USE_THREAD */                
            
        }     
    }
    
    spin_unlock(&naviDev_spi->spinlock); 
}

static irqreturn_t ctrl_IrqHandler(int irq, void* data)
{
    //pr_info("%s - %d\n", __func__, irq);
    atomic_set(&irqOccured, irq);
    
    tasklet_hi_schedule(&irq_tl);
    
    return IRQ_HANDLED;
}

static int naviDev_probe(struct spi_device *spi)
{  
    struct device_node *naviDevNode = spi->dev.of_node;
    //struct device_node *pp;
    struct device_node *reqNodes;
    struct device_node *irqNodes;
    struct device_node *bootNode;
    struct device_node *resetNode;
    int childCnt = 0;
    int i = 0;
    int size;
    int id = 0;
    int rc = 0;	
    int k = 0;	
    enum of_gpio_flags flags;
    const void *property = NULL;

/*
include/linux/of.h

*/	
    if(DEBUG_LEVEL >= LEVEL_INFO)
        pr_info("%s\n", __func__);
	
    
    /* allocate driver data */
	naviDev_spi = kzalloc(sizeof(*naviDev_spi), GFP_KERNEL);
	if (!naviDev_spi)
		return -ENOMEM;
    
    naviDev_spi->initialized = false;
    naviDev_spi->opened = false;
    
	/* initialize the driver data */
	naviDev_spi->spi = spi;
#if !defined(USE_STATIC_BUF_SIZE)	
	naviDev_spi->txBuf = NULL;
	naviDev_spi->rxBuf = NULL;
#endif /* !USE_STATIC_BUF_SIZE */	

    /* determine how many child nodes are available, this is for debugging only */
    childCnt = of_get_available_child_count(naviDevNode);
    if(DEBUG_LEVEL >= LEVEL_INFO)
        pr_info("%s - childCnt: %d\n", __func__, childCnt);
    
    
    property = of_get_property(naviDevNode, "spi-max-frequency", &size);
    naviDev_spi->speed_hz = be32_to_cpup(property);
    if(DEBUG_LEVEL >= LEVEL_INFO)
        pr_info("%s - SPI speed: %d\n", __func__, naviDev_spi->speed_hz);
        
    /* get a specific child node from the device tree */    
    reqNodes = of_get_child_by_name(naviDevNode, "imc_req");
    if(reqNodes == NULL)
    {
        if(DEBUG_LEVEL >= LEVEL_CRITICAL)
            pr_err("%s - could not find node imc_req in device tree\n", __func__);
        goto free_memory;  
    }
    
    /* get a specific child node from the device tree */
    irqNodes = of_get_child_by_name(naviDevNode, "imc_irq");
    if(irqNodes == NULL)
    {
        if(DEBUG_LEVEL >= LEVEL_CRITICAL)
            pr_err("%s - could not find node imc_irq in device tree\n", __func__);
        goto free_memory;  
    }
    
    /* get a specific child node from the device tree */
    bootNode = of_get_child_by_name(naviDevNode, "boot");
    if(bootNode == NULL)
    {
        if(DEBUG_LEVEL >= LEVEL_CRITICAL)
            pr_err("%s - could not find node boot in device tree\n", __func__);
        goto free_memory;  
    }
    
    /* get a specific child node from the device tree */
    resetNode = of_get_child_by_name(naviDevNode, "reset");
    if(resetNode == NULL)
    {
        if(DEBUG_LEVEL >= LEVEL_CRITICAL)
            pr_err("%s - could not find node reset in device tree\n", __func__);
        goto free_memory;  
    }
    
    if(DEBUG_LEVEL >= LEVEL_DEBUG)
        pr_info("%s - REQ GPIO Cnt: %d\n", __func__, of_gpio_count(reqNodes));
    if(DEBUG_LEVEL >= LEVEL_DEBUG)
        pr_info("%s - IRQ GPIO Cnt: %d\n", __func__, of_gpio_count(irqNodes));
    
    for(i = 0; i < of_gpio_count(reqNodes); i++)
    {
        naviDev_spi->req_gpio[i].gpio = of_get_gpio_flags(reqNodes, i, &flags);
		naviDev_spi->req_gpio[i].flags = (int)flags;
        naviDev_spi->req_gpio[i].name = of_get_property(reqNodes, "pe,name", &size);
        naviDev_spi->req_gpio[i].irq = -1;
        naviDev_spi->req_gpio[i].nr = i;
        
        gpio_request(naviDev_spi->req_gpio[i].gpio, naviDev_spi->req_gpio[i].name);
		gpio_direction_output(naviDev_spi->req_gpio[i].gpio, !naviDev_spi->req_gpio[i].flags);
        gpio_export(naviDev_spi->req_gpio[i].gpio, false); 
        
        if(DEBUG_LEVEL >= LEVEL_INFO)
        { 
            pr_info("%s - ID: %u\tGPIO Nr.: %u\tGPIO Dir: %u\tGPIO IRQ: %d\tGPIO Name: %s%u\n", \
    		        __func__,                           \
    		        id,                                 \
    		        naviDev_spi->req_gpio[i].gpio,          \
    		        naviDev_spi->req_gpio[i].flags,         \
    		        naviDev_spi->req_gpio[i].irq,           \
    		        naviDev_spi->req_gpio[i].name,          \
    		        naviDev_spi->req_gpio[i].nr             \
		      );
		}
    }
    
    naviDev_spi->boot_gpio.gpio = of_get_gpio_flags(bootNode, 0, &flags);
	naviDev_spi->boot_gpio.flags = (int)flags;
    naviDev_spi->boot_gpio.name = of_get_property(bootNode, "pe,name", &size);
    naviDev_spi->boot_gpio.irq = -1;
    
    gpio_request(naviDev_spi->boot_gpio.gpio, naviDev_spi->boot_gpio.name);
	gpio_direction_output(naviDev_spi->boot_gpio.gpio, !naviDev_spi->boot_gpio.flags);
    gpio_export(naviDev_spi->boot_gpio.gpio, false);         
    
    if(DEBUG_LEVEL >= LEVEL_INFO)
    {
        pr_info("%s - ID: %u\tGPIO Nr.: %u\tGPIO Dir: %u\tGPIO IRQ: %d\tGPIO Name: %s\n", \
    		        __func__,                           \
    		        id,                                 \
    		        naviDev_spi->boot_gpio.gpio,          \
    		        naviDev_spi->boot_gpio.flags,         \
    		        naviDev_spi->boot_gpio.irq,           \
    		        naviDev_spi->boot_gpio.name           \
    	      );
	}
	      
	naviDev_spi->reset_gpio.gpio = of_get_gpio_flags(resetNode, 0, &flags);
	naviDev_spi->reset_gpio.flags = (int)flags;
    naviDev_spi->reset_gpio.name = of_get_property(resetNode, "pe,name", &size);
    naviDev_spi->reset_gpio.irq = -1;
    
    /* TODO: uncomment if you want to use reset signal */
#if !defined(USE_REQ_FOR_RESET)    
    gpio_request(naviDev_spi->reset_gpio.gpio, naviDev_spi->reset_gpio.name);
	gpio_direction_output(naviDev_spi->reset_gpio.gpio, naviDev_spi->reset_gpio.flags);
    gpio_export(naviDev_spi->reset_gpio.gpio, false);         
#endif /* !USE_REQ_FOR_RESET */    
    
     
    if(DEBUG_LEVEL >= LEVEL_INFO)
    { 
        pr_info("%s - ID: %u\tGPIO Nr.: %u\tGPIO Dir: %u\tGPIO IRQ: %d\tGPIO Name: %s\n", \
    		        __func__,                           \
    		        id,                                 \
    		        naviDev_spi->reset_gpio.gpio,          \
    		        naviDev_spi->reset_gpio.flags,         \
    		        naviDev_spi->reset_gpio.irq,           \
    		        naviDev_spi->reset_gpio.name           \
    	      );
    }
    
    for(i = 0; i < of_gpio_count(irqNodes); i++)
    {
        naviDev_spi->irq_gpio[i].gpio = of_get_gpio_flags(irqNodes, i, &flags);
		naviDev_spi->irq_gpio[i].flags = (int)flags;
        naviDev_spi->irq_gpio[i].name = of_get_property(irqNodes, "pe,name", &size);
        naviDev_spi->irq_gpio[i].irq = gpio_to_irq(naviDev_spi->irq_gpio[i].gpio);
        naviDev_spi->irq_gpio[i].jiffiesEvent = jiffies;
        naviDev_spi->irq_gpio[i].nr = i;
        
        gpio_request(naviDev_spi->irq_gpio[i].gpio, naviDev_spi->irq_gpio[i].name);
		gpio_direction_input(naviDev_spi->irq_gpio[i].gpio);
		gpio_set_debounce(naviDev_spi->irq_gpio[i].gpio, 10000);
        gpio_export(naviDev_spi->irq_gpio[i].gpio, false);         
        
        if(DEBUG_LEVEL >= LEVEL_INFO)
        { 
            pr_info("%s - ID: %u\tGPIO Nr.: %u\tGPIO Dir: %u\tGPIO IRQ: %d\tGPIO Name: %s%u\n", \
        		        __func__,                           \
        		        id,                                 \
        		        naviDev_spi->irq_gpio[i].gpio,          \
        		        naviDev_spi->irq_gpio[i].flags,         \
        		        naviDev_spi->irq_gpio[i].irq,           \
        		        naviDev_spi->irq_gpio[i].name,          \
        		        naviDev_spi->irq_gpio[i].nr             \
    		      );
        }
		      
		if(naviDev_spi->irq_gpio[i].irq < 0)
        {
            goto free_gpios;
        }
        
        rc = request_irq(naviDev_spi->irq_gpio[i].irq, ctrl_IrqHandler, IRQF_TRIGGER_RISING/* IRQF_TRIGGER_FALLING*/, naviDev_spi->irq_gpio[i].name, (void*)naviDev_spi);
        if(rc) 
        {
            if(DEBUG_LEVEL >= LEVEL_CRITICAL)
                pr_err("%s - request_irq() failed with error = %d irq = %d\n", __func__, rc, naviDev_spi->irq_gpio[i].irq);
            goto free_irq;
        }
    }
    
    naviDev_resetHAT(1);

    naviDev_spi->initialized = true;
	return 0;
free_irq:
    for(k = 0; k < i; k++)
    {
        disable_irq(naviDev_spi->irq_gpio[k].irq);
        free_irq(naviDev_spi->irq_gpio[k].irq, naviDev_spi);	 
    }
free_gpios:
    for(i = 0; i < of_gpio_count(reqNodes); i++)
    {
        gpio_unexport(naviDev_spi->req_gpio[k].gpio);
        gpio_free(naviDev_spi->req_gpio[k].gpio); 
    }
    
    gpio_unexport(naviDev_spi->boot_gpio.gpio);
    gpio_free(naviDev_spi->boot_gpio.gpio); 
    gpio_unexport(naviDev_spi->reset_gpio.gpio);
    gpio_free(naviDev_spi->reset_gpio.gpio); 
    
    for(k = 0; k <= i; k++)    
    {
        gpio_unexport(naviDev_spi->irq_gpio[k].gpio);
        gpio_free(naviDev_spi->irq_gpio[k].gpio); 
    }
free_memory:
    kfree(naviDev_spi);
    return -EIO;          
}

static int naviDev_remove(struct spi_device *spi)
{
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
	    pr_info("%s\n", __func__);
	return 0;
}

//static const struct spi_device_id naviDev_id[] = {                                
//       {"hat", 0},                                               
//       {}                                                                       
//};                                                                               
//MODULE_DEVICE_TABLE(spi, naviDev_id);                                             

static struct spi_driver naviDev_spi_driver = {
	.driver = {
		.name =		"navigation device driver",
		.of_match_table = of_match_ptr(naviDev_dt_ids),
	},
	.probe =	naviDev_probe,
	.remove =	naviDev_remove,
    //.id_table = naviDev_id,
};

/* set the proper access rights for the device specified in DEVICE_NAME_SPI */
static int naviDev_spi_uevent(struct device *dev, struct kobj_uevent_env *env)
{
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
        pr_info("%s\n", __func__);
        
    if(dev && env)    
        add_uevent_var(env, "DEVMODE=%#o", 0666);
    return 0;
}

/* set the proper access rights for the device specified in DEVICE_NAME_CTRL */
static int naviDev_stat_uevent(struct device *dev, struct kobj_uevent_env *env)
{
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
        pr_info("%s\n", __func__);
        
    if(dev && env)    
        add_uevent_var(env, "DEVMODE=%#o", 0666);
    return 0;
}

#if defined(USE_TTY)
/* user triggered function, open the device */
/* returns 0 on success or < 0 on an error */
static int naviDev_tty_open(struct tty_struct *tty, struct file *file)
{
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
	    pr_info("%s\n", __func__);

	/* initialize the pointer in case something fails */
	tty->driver_data = NULL;
	
	if (!naviDev_serial)
		return -ENODEV;

	mutex_lock(&naviDev_serial->lock);

	/* save our structure within the tty structure */
	naviDev_serial->opened = true;
	tty->driver_data = naviDev_serial;
	naviDev_serial->tty = tty;	
	
	mutex_unlock(&naviDev_serial->lock);
	return 0;
}

/* user triggered function, close the device */
static void naviDev_tty_close(struct tty_struct *tty, struct file *file)
{
    struct st_naviDevSpi_serial *serial = tty->driver_data;
    
	if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
        pr_info("%s\n", __func__);
    
    mutex_lock(&serial->lock);    
    serial->opened = false;
    mutex_unlock(&serial->lock);
}	

static void naviDev_tty_throttle(struct tty_struct *tty)
{
	if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
        pr_info("%s\n", __func__);
        
    /* TODO: need to be implemented */                
}	

static void naviDev_tty_unthrottle(struct tty_struct *tty)
{
	if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
        pr_info("%s\n", __func__);
    
    /* TODO: need to be implemented */            
}

static int naviDev_tty_putchar(struct tty_struct *tty, unsigned char ch)
{
	struct st_naviDevSpi_serial *serial = tty->driver_data;
	
    if (!serial)
		return -ENODEV;

	mutex_lock(&serial->lock);

	if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
    {
        pr_info("%s - %u", __func__, ch);
    }	     
	
	/* TODO: need to be implemented */    
    		
	mutex_unlock(&serial->lock);
	
	return 1;
}

/* user triggered function, write data to the device */
/* returns the actual number of bytes written */
static int naviDev_tty_write(struct tty_struct *tty, const unsigned char *buffer, int count)
{
	struct st_naviDevSpi_serial *serial = tty->driver_data;
	int i;
	
    if (!serial)
		return -ENODEV;

	mutex_lock(&serial->lock);

    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
    {
    	printk(KERN_DEBUG "%s - ", __func__);
    	for (i = 0; i < count; ++i)
    	{
    		printk("%02x ", buffer[i]);        
    	}
    	printk("\n");
    }
			
	mutex_unlock(&serial->lock);
	
	return count;
}

static int naviDev_tty_write_room(struct tty_struct *tty) 
{
	struct st_naviDevSpi_serial *serial = tty->driver_data;
	int room = -EINVAL;

    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
        pr_info("%s\n", __func__);

	if (!serial)
		return -ENODEV;

	mutex_lock(&serial->lock);

	/* calculate how much room is left in the device */
	/* TODO: implement calculation */
	room = 255;

	mutex_unlock(&serial->lock);
	return room;
}

static void naviDev_tty_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
{
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
        pr_info("%s\n", __func__);    
        
    /* TODO: need to be implemented */
}

/* Will be called if the device specified in DEVICE_NAME_TTY is opened the first time. This is acutally executed prior open(...). */
static int naviDev_tty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	int retval = -ENOMEM;
    
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
        pr_info("%s\n", __func__);

    mutex_lock(&naviDev_serial->lock);
	tty->port = kmalloc(sizeof *tty->port, GFP_KERNEL);
	if (!tty->port)
		goto err;

	tty_init_termios(tty);
	driver->ttys[0] = tty;
	
	tty_port_init(tty->port);
	tty_buffer_set_limit(tty->port, 8192);
	tty_driver_kref_get(driver);
	tty->count++;	
	
	naviDev_serial->initialized = true;
    mutex_unlock(&naviDev_serial->lock);
	return 0;

err:
    if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
        pr_err("%s - err\n", __func__);
	kfree(tty->port);
	return retval;
}

static struct tty_operations serial_ops = {
	.open           = naviDev_tty_open,
	.close          = naviDev_tty_close,
	.write          = naviDev_tty_write,
	.write_room     = naviDev_tty_write_room,
	.set_termios    = naviDev_tty_set_termios,
    .install        = naviDev_tty_install,
    .throttle       = naviDev_tty_throttle,
    .unthrottle     = naviDev_tty_unthrottle,
    .put_char       = naviDev_tty_putchar,
};

static char *naviDev_tty_devnode(struct device *dev, umode_t *mode)
{
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
        pr_info("%s\n", __func__);
        
	if (!mode)
		return NULL;
	
	*mode = 0666;
	return NULL;
}

static int naviDev_tty_uevent(struct device *dev, struct kobj_uevent_env *env)
{
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
        pr_info("%s\n", __func__);
    add_uevent_var(env, "DEVMODE=%#o", 0666);
    return 0;
}

static struct tty_driver *naviDev_tty_driver;
#endif /* USE_TTY */

static int __init naviDev_init(void)
{
    if(DEBUG_LEVEL >= LEVEL_INFO)
        pr_info("%s\n", __func__);

#if defined(USE_TTY)	
	naviDev_serial = NULL;
#endif /* USE_TTY */	
	
#if !defined(USE_STATIC_FIFO_SIZE)	
	rxFifo.data = kmalloc(FIFO_SIZE, GFP_KERNEL);
	if(!rxFifo.data)
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL)
	        pr_err("%s - allocating memory for fifo failed\n", __func__);
	    return -ENOMEM;
	}
#endif /* !USE_STATIC_FIFO_SIZE */	
	rxFifo.size = FIFO_SIZE;
	rxFifo.cnt = 0;
	memset(rxFifo.data, 0, rxFifo.size);		
	
	/* reserve device number */	
	if(alloc_chrdev_region(&naviDev_spi_nr, 0, 1, DEVICE_NAME_SPI) < 0)
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
	        pr_err("%s - failed to reserve device number for \"%s\"\n", __func__, DEVICE_NAME_SPI);
        goto free_mem;
	}
	
	/* allocate memory for the cdev structure */
	naviDev_spi_object = cdev_alloc(); 
	if(naviDev_spi_object == NULL)
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
	        pr_err("%s - failed to allocate memory for device \"%s\"\n", __func__, DEVICE_NAME_SPI);
		goto free_device_number;
	}
		
    naviDev_spi_object->owner = THIS_MODULE;
	naviDev_spi_object->ops = &naviDev_fops; 
	
	/* register cdev object at the kernel */
	if(cdev_add(naviDev_spi_object, naviDev_spi_nr, 1))
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
	        pr_err("%s - failed to register cdev object at kernel\n", __func__);
		goto free_cdev;		
	}
	
    /* create new device class */
	naviDev_spi_class = class_create(THIS_MODULE, DEVICE_NAME_SPI);
	if(naviDev_spi_class == NULL)
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
	        pr_err("%s - failed to create device \"%s\"\n", __func__, DEVICE_NAME_SPI);
		goto free_cdev;
	}
	
	/* define function to set access permissions for the device file */
    naviDev_spi_class->dev_uevent = naviDev_spi_uevent;
    		
    /* create device file */
	naviDev_spi_dev = device_create(naviDev_spi_class, NULL, naviDev_spi_nr, NULL, "%s", DEVICE_NAME_SPI);

#if defined(USE_TTY)
    /* allocate the tty driver */
	naviDev_tty_driver = alloc_tty_driver(1);
	if (!naviDev_tty_driver)
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
	        pr_err("%s - failed to allocate memory for TTY driver\n", __func__);
	    goto free_class;
	}
	
	if(naviDev_serial == NULL)
	{
		naviDev_serial = kmalloc(sizeof(*naviDev_serial), GFP_KERNEL);
		if (!naviDev_serial)
	    {
            if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
	            pr_err("%s - failed to allocate memory\n", __func__);
	        goto free_tty;
	    }
	    naviDev_serial->initialized = false;
	    naviDev_serial->opened = false;
	    naviDev_serial->tty = NULL;
	    //naviDev_serial->tty->port = NULL;
	}
		

	/* initialize the tty driver */
	naviDev_tty_driver->owner = THIS_MODULE;
	naviDev_tty_driver->driver_name = DEVICE_NAME_TTY;
	naviDev_tty_driver->name = DEVICE_NAME_TTY;
	naviDev_tty_driver->major = NAVIDEV_TTY_MAJOR,
	naviDev_tty_driver->type = TTY_DRIVER_TYPE_SYSTEM,
	naviDev_tty_driver->subtype = SYSTEM_TYPE_CONSOLE,
	naviDev_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV | TTY_DRIVER_UNNUMBERED_NODE,
	naviDev_tty_driver->init_termios = tty_std_termios;
	naviDev_tty_driver->init_termios.c_cflag = B115200 | CS8 | CREAD | HUPCL | CLOCAL;
	naviDev_tty_driver->init_termios.c_lflag &= ~ECHO;
	naviDev_tty_driver->init_termios.c_iflag |= INLCR;
	tty_set_operations(naviDev_tty_driver, &serial_ops);
	
	/* register the tty driver */
	if(tty_register_driver(naviDev_tty_driver))
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
		    pr_err("%s - failed to register TTY driver\n", __func__);
		goto free_tty_mem;
	}
	
	naviDev_tty_dev = tty_register_device(naviDev_tty_driver, 0, NULL);
	naviDev_tty_class = naviDev_tty_dev->class;
	if(naviDev_tty_class)
    {
        /* currently settings permissions does not work, why not??? */
        /* it works for the SPI device file but not for TTY */
	    naviDev_tty_class->dev_uevent = naviDev_tty_uevent;
	    naviDev_tty_class->devnode = naviDev_tty_devnode;
	}   
	else
    {
        if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
            pr_err("%s - failed to register TTY device\n", __func__);
        goto free_tty_driver;
    }
#endif /* USE_TTY */    
		
     /* register driver at the SPI core */
	if(spi_register_driver(&naviDev_spi_driver) < 0)	
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
	        pr_err("%s - failed to register SPI driver\n", __func__);
		//goto free_all;
		goto free_all;
	}
	
#if defined(USE_STATISTICS)	
    if(alloc_chrdev_region(&naviDev_stat_nr, 0, 1, DEVICE_NAME_CTRL) < 0)
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL)
	        pr_err("%s - failed to reserve device number for \"%s\"\n", __func__, DEVICE_NAME_CTRL);
        goto free_spi_driver;
	}
	
	/* allocate memory for the cdev structure */
	naviDev_stat_object = cdev_alloc(); 
	if(naviDev_stat_object == NULL)
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL)
	        pr_err("%s - failed to allocate memory for device \"%s\"\n", __func__, DEVICE_NAME_CTRL);
		goto free_stat_device_number;
	}
		
    naviDev_stat_object->owner = THIS_MODULE;
	naviDev_stat_object->ops = &naviDev_statfops; 
	
	/* register cdev object at the kernel */
	if(cdev_add(naviDev_stat_object, naviDev_stat_nr, 1))
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL)
	        pr_err("%s - failed to register cdev object at kernel\n", __func__);
		goto free_stat_cdev;		
	}
	
    /* create new device class */
	naviDev_stat_class = class_create(THIS_MODULE, DEVICE_NAME_CTRL);
	if(naviDev_stat_class == NULL)
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL)
	        pr_err("%s - failed to create device \"%s\"\n", __func__, DEVICE_NAME_CTRL);
		goto free_stat_cdev;
	}
	
	/* define function to set access permissions for the device file */
    naviDev_stat_class->dev_uevent = naviDev_stat_uevent;
    		
    /* create device file */
	naviDev_stat_dev = device_create(naviDev_stat_class, NULL, naviDev_stat_nr, NULL, "%s", DEVICE_NAME_CTRL);
	
#endif /* USE_STATISTICS */
	
	RESET_DATA_AVAILABLE_USER;
	SLAVE_REQ_CONFIRMED;
	
    spin_lock_init(&naviDev_spi->spinlock);
    spin_lock_init(&rxFifo.spinlock);
#if defined(USE_STATISTICS)    
    spin_lock_init(&statistics.spinlock);
    spin_lock_init(&info.spinlock);
    spin_lock_init(&config.spinlock);
#endif /* USE_STATISTICS */    
#if defined(USE_TTY)    
    mutex_init(&naviDev_serial->lock);
#endif /* USE_TTY */    
    
    init_waitqueue_head(&wq_read);

#if defined(USE_STATISTICS)    
    naviDev_resetStatistics(&statistics);
    info.valid = false;
#endif /* USE_STATISTICS */    
    /* initialize kernel thread */
    /* the kernel thread is used for transmitting data via SPI, this only for debugging purpose */
#if defined(USE_THREAD)    
    init_waitqueue_head(&wq_thread);
    thread_id = kthread_create(naviDev_thread, NULL, "naviDev thread");
    wake_up_process(thread_id);
#endif /* USE_THREAD */    
    return 0;
    
    /* to avoid compiler errors */
    goto free_class;
    goto free_all;
    goto free_tty_driver;
    goto free_tty_mem;
    goto free_tty;
#if defined(USE_STATISTICS)	    
    goto free_spi_driver;
    goto free_stat_device_number;
	goto free_stat_class;
	goto free_stat_cdev;
#endif /* USE_STATISTICS */
	
/* cleaning up... */
#if defined(USE_STATISTICS)	
free_stat_class:
    naviDev_stat_class->dev_uevent = NULL;
    device_destroy(naviDev_stat_class, naviDev_stat_nr);
    class_destroy(naviDev_stat_class);
free_stat_cdev:
    kobject_put(&naviDev_stat_object->kobj);
free_stat_device_number:
    unregister_chrdev_region(naviDev_stat_nr, 1);
free_spi_driver:
    spi_unregister_driver(&naviDev_spi_driver);
#endif /* USE_STATISTICS */    
free_all:
#if defined(USE_TTY)    
    tty_unregister_device(naviDev_tty_driver, 0);
#endif /* USE_TTY */    
free_tty_driver:
#if defined(USE_TTY)    
    tty_unregister_driver(naviDev_tty_driver);
#endif /* USE_TTY */ 
free_tty_mem:
#if defined(USE_TTY)  
    if(naviDev_serial)
    {
        kfree(naviDev_serial);
        naviDev_serial = NULL;    
	}
#endif /* USE_TTY */	
free_tty:
#if defined(USE_TTY)      
    put_tty_driver(naviDev_tty_driver);    
#endif /* USE_TTY */ 	    
free_class:
    naviDev_spi_class->dev_uevent = NULL;
    device_destroy(naviDev_spi_class, naviDev_spi_nr);
    class_destroy(naviDev_spi_class);
free_cdev:
    kobject_put(&naviDev_spi_object->kobj);
free_device_number:
    unregister_chrdev_region(naviDev_spi_nr, 1);
free_mem:
#if !defined(USE_STATIC_FIFO_SIZE)    
    kfree(rxFifo.data);
    rxFifo.data = NULL;
#endif /* !USE_STATIC_FIFO_SIZE */    
    rxFifo.size = 0;
	return -EIO;	
}

static void __exit naviDev_exit(void)
{
    int i = 0;
    
    if(DEBUG_LEVEL >= LEVEL_INFO)
        pr_info("%s\n", __func__);
    
    for(i = 0; i < DIM(naviDev_spi->irq_gpio); i++)   
	{
	    disable_irq(naviDev_spi->irq_gpio[i].irq);
        free_irq(naviDev_spi->irq_gpio[i].irq, naviDev_spi);   
    }
    
    naviDev_spi->initialized = false;
    naviDev_spi->opened = false;
#if defined(USE_STATISTICS)     
    info.valid = false;
#endif /* USE_STATISTICS */    
    naviDev_serial->initialized = false;
    naviDev_serial->opened = false;
    
    kill_pid(task_pid(thread_id), SIGTERM, 1);
    wait_for_completion(&on_exit);
    thread_id = NULL;
#if defined(USE_TTY)    
    naviDev_tty_class->dev_uevent = NULL;
    naviDev_tty_class->devnode = NULL;

    tty_unregister_device(naviDev_tty_driver, 0);
    tty_unregister_driver(naviDev_tty_driver);
	put_tty_driver(naviDev_tty_driver); 
	if(naviDev_serial)
	{
	    if(naviDev_serial->tty)
	    {
    		if(naviDev_serial->tty->port)
    		{
    		    kfree(naviDev_serial->tty->port);
    		    naviDev_serial->tty->port = NULL;
    		}
    	}
		kfree(naviDev_serial);
		naviDev_serial = NULL;
	}
#endif /* USE_TTY */     
    
    spi_unregister_driver(&naviDev_spi_driver);
	naviDev_spi_class->dev_uevent = NULL;
	device_destroy(naviDev_spi_class, naviDev_spi_nr);
	class_destroy(naviDev_spi_class);
	cdev_del(naviDev_spi_object);
	unregister_chrdev_region(naviDev_spi_nr, 1);
	for(i = 0; i < DIM(naviDev_spi->req_gpio); i++)   
	{
	    gpio_unexport(naviDev_spi->req_gpio[i].gpio);         
        gpio_free(naviDev_spi->req_gpio[i].gpio); 
    }
    
    gpio_unexport(naviDev_spi->boot_gpio.gpio);
    gpio_free(naviDev_spi->boot_gpio.gpio); 
    gpio_unexport(naviDev_spi->reset_gpio.gpio);
    gpio_free(naviDev_spi->reset_gpio.gpio); 
    
    for(i = 0; i < DIM(naviDev_spi->irq_gpio); i++)   
	{
	    gpio_unexport(naviDev_spi->irq_gpio[i].gpio);            
        gpio_free(naviDev_spi->irq_gpio[i].gpio); 
    }
    
#if !defined(USE_STATIC_BUF_SIZE)    
    if(naviDev_spi->txBuf)
    {
        kfree(naviDev_spi->txBuf);
        naviDev_spi->txBuf = NULL;
    }
    if(naviDev_spi->rxBuf)
    {
        kfree(naviDev_spi->rxBuf);
        naviDev_spi->rxBuf = NULL;
    }
#endif /* !USE_STATIC_BUF_SIZE */  
    kfree(naviDev_spi);
    
#if !defined(USE_STATIC_FIFO_SIZE)    
    if(rxFifo.data)
    {
        kfree(rxFifo.data);
        rxFifo.data = NULL;
    }
#endif /* !USE_STATIC_FIFO_SIZE */    
    rxFifo.size = 0;

#if defined(USE_STATISTICS)    
    naviDev_stat_class->dev_uevent = NULL;
	device_destroy(naviDev_stat_class, naviDev_stat_nr);
	class_destroy(naviDev_stat_class);
	cdev_del(naviDev_stat_object);
	unregister_chrdev_region(naviDev_stat_nr, 1);
#endif /* USE_STATISTICS */	
}

module_init(naviDev_init);
module_exit(naviDev_exit);
MODULE_AUTHOR("Thomas POMS, <hwsw.development@gmail.com>");
MODULE_DESCRIPTION("NAVI HAT Device Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("navidev");