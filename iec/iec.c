/* Copyright 2013 by Chris Osborn <fozztexx@fozztexx.com>
 *
 * This file is part of ninepin.
 *
 * ninepin is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * ninepin is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ninepin; see the file COPYING. If not see
 * <http://www.gnu.org/licenses/>.
 */

#include "iec.h"
#include "gpio.h"

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <asm/io.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/poll.h>

#define DRIVER_AUTHOR   "Chris Osborn <fozztexx@fozztexx.com>"
#define DRIVER_DESC     "Commodore IEC serial driver"

//
#define IEC_BUFSIZE     1024

/*
 * Device ID    Device Type     Serial Bus
 * 0            Keyboard        NO      
 * 1            Cassette        NO
 * 2            RS232           NO
 * 3            Screen          NO
 * 4            Printer         YES   <--- IEC_FIRSTDEV
 * 5            Printer         YES
 * 6            Plotter         YES
 * 7            Plotter         YES
 * 8            Disk primary    YES
 * 9            Disk            YES
 * 10           Disk            YES
 * 11           Disk            YES
 * 12           Disk            YES  
 * 13           Disk            YES
 * 14           Disk            YES
 * 15           Disk            YES   <--- IEC_LASTDEV
 * 16-30        UNKNOW          ???
 * 31           Reserved as command to all devices ( on seial bus )
 */

#define IEC_FIRSTDEV    4
#define IEC_LASTDEV     15
#define IEC_ALLDEV      (IEC_LASTDEV - IEC_FIRSTDEV)
// when talker sends this device number it is addressing all the devices
#define IEC_BROADCAST   31
/*
 * GPIO pins
 *
 * it's better to use more gpio in a unidirectional way, it does not need
 * time to switch between input and output mode
 * Also, using more than 3 gpio make the code more readable.
 *
 *   +-----+-----+---------+------+---+---Pi 3B+-+---+------+---------+-----+-----+
 *   | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
 *   +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
 *   |     |     |    3.3v |      |   |  1 || 2  |   |      | 5v      |     |     |
 *   |   2 |   8 |   SDA.1 |   IN | 1 |  3 || 4  |   |      | 5v      |     |     |
 *   |   3 |   9 |   SCL.1 |   IN | 1 |  5 || 6  |   |      | 0v      |     |     |
 *   |   4 |   7 | GPIO. 7 |   IN | 1 |  7 || 8  | 1 | ALT0 | TxD     | 15  | 14  |
 *   |     |     |      0v |      |   |  9 || 10 | 1 | ALT0 | RxD     | 16  | 15  |
 *   |  17 |   0 | GPIO. 0 |   IN | 0 | 11 || 12 | 0 | IN   | GPIO. 1 | 1   | 18  |
 *   |  27 |   2 | GPIO. 2 |   IN | 0 | 13 || 14 |   |      | 0v      |     |     |
 *   |  22 |   3 | GPIO. 3 |   IN | 0 | 15 || 16 | 0 | IN   | GPIO. 4 | 4   | 23  |
 *   |     |     |    3.3v |      |   | 17 || 18 | 0 | IN   | GPIO. 5 | 5   | 24  |
 *   |  10 |  12 |    MOSI |   IN | 0 | 19 || 20 |   |      | 0v      |     |     |
 *   |   9 |  13 |    MISO |   IN | 0 | 21 || 22 | 0 | IN   | GPIO. 6 | 6   | 25  |
 *   |  11 |  14 |    SCLK |   IN | 0 | 23 || 24 | 1 | IN   | CE0     | 10  | 8   |
 *   |     |     |      0v |      |   | 25 || 26 | 1 | IN   | CE1     | 11  | 7   |
 *   |   0 |  30 |   SDA.0 |   IN | 1 | 27 || 28 | 1 | IN   | SCL.0   | 31  | 1   |
 *   |   5 |  21 | GPIO.21 |   IN | 1 | 29 || 30 |   |      | 0v      |     |     |
 *   |   6 |  22 |*CLKIN** |   IN | 1 | 31 || 32 | 0 | IN   | GPIO.26 | 26  | 12  |
 *   |  13 |  23 |*DATAIN* |   IN | 0 | 33 || 34 |   |      | 0v      |     |     |
 *   |  19 |  24 | GPIO.24 |   IN | 1 | 35 || 36 | 0 | IN   | GPIO.27 | 27  | 16  |
 *   |  26 |  25 |*ATNIN** |   IN | 0 | 37 || 38 | 1 | OUT  |*CLKOUT**| 28  | 20  |
 *   |     |     |      0v |      |   | 39 || 40 | 1 | OUT  |*DATAOUY*| 29  | 21  |
 *   +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
 *   | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
 *   +-----+-----+---------+------+---+---Pi 3B+-+---+------+---------+-----+-----+
 *
 */
// commodore 64 will drive the ATN line all the time
#define IEC_ATNIN         26 

#define IEC_DATAIN        13
#define IEC_DATAOUT       21
#define IEC_CLKIN          6
#define IEC_CLKOUT        20
// new entries.
// reset allow this device when drop anything
// IEQ_SQR is used by the C128
#define IEC_RESETIN       addme
#define IEC_SQR           addme


#define LABEL_ATNIN       "IEC attention pin (IN)"
#define LABEL_CLKIN       "IEC clock pin (IN)"
#define LABEL_DATAIN      "IEC data pin (IN)"
#define LABEL_CLKOUT      "IEC clock pin (OUT)"
#define LABEL_DATAOUT     "IEC data pin (OUT)"
#define LABEL_DEVICE      "iec"
#define LABEL_WRITE       "IEC write queue"
#define LABEL_READ        "IEC read queue"

#define DATA_EOI        0x100
#define DATA_ATN        0x200

enum {
  IECWaitState = 1,
  IECAttentionState,
  IECAttentionIgnoreState,
  IECListenState,
  IECInputState,
  IECTalkState,
  IECOutputState,
};

typedef struct {
  iec_data      header;
  unsigned char data[IEC_BUFSIZE];
  void          *next;
  int           outpos;
} iec_io;

typedef struct {
  iec_io *head, *tail, *cur;
} iec_chain;

typedef struct {
  iec_chain in;
  iec_io out;
  int flags;
  struct semaphore lock;
} iec_device;

static irqreturn_t  iec_handleCLK(int irq, void *dev_id, struct pt_regs *regs);
static irqreturn_t  iec_handleATN(int irq, void *dev_id, struct pt_regs *regs);
static void         iec_processData(struct work_struct *work);
       int          iec_open(struct inode *inode, struct file *filp);
       int          iec_close(struct inode *inode, struct file *filp);
       unsigned int iec_poll(struct file *filp, poll_table *wait);
       ssize_t      iec_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
       ssize_t      iec_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);

struct file_operations iec_fops = {
    owner:   THIS_MODULE,
    read:    iec_read,
    write:   iec_write,
    poll:    iec_poll,
    open:    iec_open,
    release: iec_close
};

/* This array holds the status of the open devices.
 * Every time a process opens a device, we store the information of the
 * device inside this structure. The minor of the device is the used to
 * address the record in it.
 */
static iec_device              *iec_openDevices[IEC_ALLDEV+1];
// how much devices have been opened
static int                      iec_curDevice, iec_curChannel;

static int                      iec_readAvail = 0;

static short int                irq_atn = 0, irq_clk = 0;
static dev_t                    iec_devno;
static struct cdev              iec_cdev;
static struct class            *iec_class;
static uint16_t                *iec_buffer;
static short                    iec_inpos, iec_outpos;
static int                      iec_state = 0, iec_atnState = 0;
static struct workqueue_struct *iec_writeQ, *iec_readQ;
static int                      c64slowdown = 10;

DECLARE_WORK(iec_readWork, iec_processData);
static DECLARE_WAIT_QUEUE_HEAD(iec_canRead);

/*
 * wait no more than delay us that pin goes to level val or pin2 to level val2
 */
static inline int iec_waitForSignals(int pin, int val, int pin2, int val2, int delay)
{
    struct timeval start, now;
    int elapsed, abort = 0;

    do_gettimeofday(&start);
    for (;;) {
        if (gpio_get_value(pin) == val)
          break;
        if (pin2 && gpio_get_value(pin2) == val2)
          break;

        do_gettimeofday(&now);
        elapsed = (now.tv_sec - start.tv_sec) * 1000000 + (now.tv_usec - start.tv_usec);
        if (elapsed >= delay) {
          abort = 1;
          break;
        }
    }
    return abort;
}

// TODO check this state
void iec_releaseBus(void)
{
    gpio_set_value(IEC_CLKOUT,  HIGH);
    gpio_set_value(IEC_DATAOUT, HIGH);
    udelay(c64slowdown);
    return;
}

/*
 * If the commodore 64 triggers the ATN, all devices must be switch
 * in LISTENER STATE.
 *
 * The listener has to pull the DATA line to low ( TRUE ) the commodore 64 will
 * read a TRUE ( low value ) after at least a listener put this line low.
 */
static irqreturn_t iec_handleATN(int irq, void *dev_id, struct pt_regs *regs)
{
    int atn;
    // TRUE = LOW -> 0
    atn = !gpio_get_value(IEC_ATNIN);
    // if the ATN is LOW (TRUE) we have to switch in LISTENER MODE
    if (atn) {
        iec_atnState = IECAttentionState;
        iec_state = IECWaitState;
        // clock will be TRUE if and only if the TALKER will change its state
        gpio_set_value(IEC_CLKOUT,HIGH); // FALSE
        // setting data to low we will allow the commodore 64 to understand that
        // there is at least a listener on the bus
        gpio_set_value(IEC_DATAOUT, LOW); // TRUE
    }
    else
        iec_atnState = IECWaitState;
    return IRQ_HANDLED;
}

/*
 * This function will read a byte from the bus.
 * and it's strictly timed with no interrupts.
 */
static inline int iec_readByte(void)
{
    unsigned long flags;
    int eoi, abort;
    int len, bits;
    struct timeval start, now;
    int elapsed = 0;

    // DISABLE INTERRUPT and save the status in flag.
    local_irq_save(flags);

    // we are here because the talker relesed the CLOCK.
    // releasing the DATA we are telling: ok I'm ready to receive the data
    gpio_set_value(IEC_DATAOUT, HIGH); // FALSE

    // From now the talker must send the data within 200us.
    // if this does not happens then the talker is telling us that
    // the next charater will be the last one.
    // after 200us of inactivity we notify to the talker that the
    // EOI has been acknowledged.
    // After this we wait the last char.
    
    do_gettimeofday(&start);
    
    /* this lopp runs until:
     *
     *  1. the CLOCK change state
     *  2. we are waiting for more 100ms
     */
    for (abort = eoi = 0; !abort && gpio_get_value(IEC_CLKIN); ) {
        do_gettimeofday(&now);
        
        elapsed = (now.tv_sec - start.tv_sec) * 1000000 + (now.tv_usec - start.tv_usec);

        // acknoledge the EOI
        // hopefully, 60 us seconds after this ACK the talker will send the last char
        if (!eoi && elapsed >= 200) {
            gpio_set_value(IEC_DATAOUT, LOW);
            udelay(c64slowdown<<1);
            gpio_set_value(IEC_DATAOUT, HIGH);
            eoi = DATA_EOI;
        }

        if (elapsed > 100000) {
          abort = 1;
          break;
        }
    }
    
    // we are talking to the commodore 64, so the answer should be
    // provided in around 60 us.
    if (elapsed < 60) {
        len = elapsed - c64slowdown;
        if (len > 0 && c64slowdown / 10 < len && len > 5) {
          c64slowdown = elapsed;
        }
    }

    /*
     * gets 8 bits from the bus.
     */
    for (len = 0, bits = eoi; !abort && len < 8; len++) {
        // wait CLOCK to go FALSE in 150us  ____-----
        if ((abort = iec_waitForSignals(IEC_CLKIN, 1, 0, 0, 150))) {
          break;
        }

        if (gpio_get_value(IEC_DATAIN))
            bits |= 1 << len;
        // wait CLOCK to go TRUE in 150us -----_____
        if (iec_waitForSignals(IEC_CLKIN, 0, 0, 0, 150)) {
              if (len < 7)
                  abort = 1;
        }
    }

    gpio_set_value(IEC_DATAOUT, LOW);
    
    // ENABLE AGAIN THE INTERRUPTS
    local_irq_restore(flags);

    udelay(c64slowdown);

    if (abort)
        return -1;

    return bits;
}



/*
 * The clock signal.
 *
 * When the talker has something to say, it will release the CLOCK.
 * This can happens also when the ATN is stil active, in this case the data
 * are "service data"
 */
static irqreturn_t iec_handleCLK(int irq, void *dev_id, struct pt_regs *regs)
{
    int atn, val;
    int cmd, dev;
    int abort;

    // Talker is ready to send us data, is the ATN released?
    atn = !gpio_get_value(IEC_ATNIN);
    if (!atn)
        iec_atnState = IECWaitState;

    if (
        iec_atnState != IECAttentionState &&
        (   iec_state == IECWaitState
         || iec_state == IECOutputState
         || iec_state == IECTalkState)
        )
    return IRQ_HANDLED;

    if (atn && iec_atnState == IECAttentionIgnoreState)
        return IRQ_HANDLED;

    abort = 0;

    /* FIXME - if buffer is full waiting for userspace to read, we need to block! */
    /* FIXME - if iec_readAvail is set then buffer is being sent */

    // read the byte ( of course )
    val = iec_readByte();
    
    //printk(KERN_NOTICE "IEC: Read: %03x %i %i\n", val, atn, iec_atnState);
    
    // we got a byte, ha been is sent in ATN ?
    if (val >= 0) {
        if (atn) {
            // this will sign the data as "received under ATN
            val |= DATA_ATN;

            /* Partially processing commands that may need to release bus
             here because of timing issues. */
            
            cmd = val & 0xe0; // command
            dev = val & 0x1f; // device addressed
            
            /*
             * Releases the bus if the command is not for the devices
             * handled by this driver
             */
            switch (cmd) {
                case IECListenCommand:
                    if ( dev == IEC_BROADCAST || !iec_openDevices[dev]) {
                          iec_atnState = IECAttentionIgnoreState;
                          if (dev == IEC_BROADCAST)
                              iec_state = IECWaitState;
                          udelay(c64slowdown);
                          iec_releaseBus();
                    }
                    break;

                case IECTalkCommand:
                    if (dev == IEC_BROADCAST || !iec_openDevices[dev]) {
                        iec_atnState = IECAttentionIgnoreState;
                        if (dev == IEC_BROADCAST)
                            iec_state = IECWaitState;
                        udelay(c64slowdown);
                        iec_releaseBus();
                    }
                break;
            }
        }

        // Stores the command,
        iec_buffer[iec_inpos] = val;
        iec_inpos = ++iec_inpos % IEC_BUFSIZE;
        queue_work(iec_readQ, &iec_readWork);
    }
    return IRQ_HANDLED;
}


/*
 * opens a new IO
 *
 * That is something like a file, something that can collect
 * data sent from the c64.
 * c64 sends:
 * 0x28 ( listent unit 8 ) and this driver will allocate a new
 *      iec_io struct for the device 8
 *
 * iec_io store the information of a whole communication
 */
void iec_newIO(int val)
{
    int dev;
    iec_device *device;
    iec_chain *chain;
    iec_io *io;
    static unsigned char serial = 0;

    dev = val & 0x1f;
    // dev is sent on the BUS, and it's allocate in
    // iec_open() and it's allocate using the minor of the
    // device. So, if you open /dev/iec8, you will able to
    // answer as unit #8, messages to other devices will be
    // discarded.
    device = iec_openDevices[dev];
    if (!device)
        return;

    chain = &device->in;

    /* FIXME - this shouldn't happen */
    if (chain->cur)
        printk(KERN_NOTICE "IEC: new IO with IO still open\n");

    //printk(KERN_NOTICE "IEC: new IO\n");

    io = kmalloc(sizeof(iec_io), GFP_KERNEL);
    io->header.command = val & 0xff;
    io->header.channel = 0;
    io->header.len = 0;
    io->header.eoi = 0;
    io->header.serial = serial++;
    io->next = NULL;
    io->outpos = 0;

    if (!chain->head)
        chain->head = chain->tail = io;
    else {
        chain->tail->next = io;
        chain->tail = io;
    }
    chain->cur = io;

    return;
}


void iec_channelIO(int val)
{
    int chan;
    iec_device *device;
    iec_chain *chain;

    chan = val & 0x0f;

    if (!(device = iec_openDevices[iec_curDevice]))
        return;
    //printk(KERN_NOTICE "IEC: changing channel to %i\n", chan);
    iec_curChannel = chan;
    chain = &device->in;
    if (!chain->cur)
        return;
    
    chain->cur->header.channel = val & 0xff;
    return;
}


void iec_appendByte(int val)
{
    iec_device *device;
    iec_chain *chain;
    iec_io *io;

    if (!(device = iec_openDevices[iec_curDevice]))
        return;

    chain = &device->in;
    if (!(io = chain->cur))
        return;

    val = val & 0xff;
    io->data[io->header.len] = val;
    io->header.len++;

    return;
}

void iec_sendInput(void)
{
    iec_device *device;
    iec_chain *chain;
    iec_io *io;

    if (!(device = iec_openDevices[iec_curDevice]))
        return;

    /* FIXME - if we are holding the bus because of this device, release the bus */

    chain = &device->in;
    if (!chain->cur)
        return;

    //printk(KERN_NOTICE "IEC: sending input\n");

    io = chain->cur;
    chain->cur = NULL;
    iec_readAvail = 1;
    wake_up_interruptible(&iec_canRead);

    return;
}
  

int iec_setupTalker(void)
{
    int abort = 0;

    //printk(KERN_NOTICE "IEC: switching to talk\n");
    abort = iec_waitForSignals(IEC_ATNIN, 1, 0, 0, 1000000);
    if (!abort && (abort = iec_waitForSignals(IEC_CLKIN, 1, IEC_ATNIN, 0, 100000)))
        printk(KERN_NOTICE "IEC: Timeout waiting for start of talk\n");

    if (!abort) {
        // TALKED holds CLOCK to true (LOW)
        // LISTENER hold DATA to true
        gpio_set_value(IEC_DATAOUT, HIGH);
        gpio_set_value(IEC_CLKOUT,LOW);
        udelay(c64slowdown);
        iec_state = IECOutputState;
    }

    return abort;
}

/*
 * this function processes the commands, part of them are also
 * processed elsewhere just to check if the bus must be released
 */
static void iec_processData(struct work_struct *work)
{
    int avail;
    int val, atn;
    int cmd, dev;


    for (;;) {
        avail = (IEC_BUFSIZE + iec_inpos - iec_outpos) % IEC_BUFSIZE;
        if (!avail)
            return;

        val = iec_buffer[iec_outpos];
        atn = val & DATA_ATN;
        iec_outpos = ++iec_outpos % IEC_BUFSIZE;
        printk(KERN_NOTICE "IEC: processing data %02x\n", val);

        // if the command has been sent in ATN mode it's something
        // that addresses the driver
        if (atn) {
            cmd = val & 0xe0;
            dev = val & 0x1f;

            switch (cmd) {
                case IECListenCommand:
                    if (dev == IEC_BROADCAST || !iec_openDevices[dev])
                        iec_sendInput();
                    else {
                        iec_state = IECListenState;
                        iec_newIO(val);
                        iec_curDevice = dev;
                    }
                    break;

                case IECTalkCommand:
                    if (dev == IEC_BROADCAST || !iec_openDevices[dev])
                        iec_sendInput();
                    else {
                        iec_state = IECTalkState;
                        iec_newIO(val);
                        iec_curDevice = dev;
                    }
                    break;

                case IECChannelCommand:
                    iec_channelIO(val);
                    /* Take a break driver 8. We can reach our destination, but we're still a ways away */
                    if (iec_state == IECTalkState) {
                        iec_setupTalker();
                        iec_sendInput();
                    }
                    break;

                case IECFileCommand:
                    iec_channelIO(val);
                    if (dev == 0x00)
                        iec_sendInput();
                    break;

                default:
                    printk(KERN_NOTICE "IEC: unknown command %02x\n", val);
                    break;
            }
        }
        else {
            iec_device *device;
            iec_appendByte(val);
            device = iec_openDevices[iec_curDevice];
            if (val & DATA_EOI)
                device->in.cur->header.eoi = 1;
            if (device->in.cur->header.len == sizeof(device->in.cur->data))
                iec_sendInput();
        }
    }
    return;
}


int iec_writeByte(int bits)
{
    int len;
    int abort = 0;
    unsigned long flags;

    if (!gpio_get_value(IEC_ATNIN) || iec_state != IECOutputState) {
        return 1;
    }

    // disable the IRQ on che CLOCK line
    disable_irq(irq_clk);
    // Disable all the interrupts?
    local_irq_save(flags);

    //printk(KERN_NOTICE "IEC: Write: %03x data: %i\n", bits, gpio_get_value(IEC_DATA));
    gpio_set_value(IEC_CLKOUT, HIGH);

    abort = iec_waitForSignals(IEC_DATAIN, 1, IEC_ATNIN, 0, 100000)

    /* Because interrupts are disabled it's possible to miss the ATN pause signal */
    if (!gpio_get_value(IEC_ATNIN)) {
        iec_state = IECWaitState;
        iec_atnState = IECAttentionState;
        abort = 1;
    }

    if (!abort && (bits & DATA_EOI)) {
        abort = iec_waitForSignals(IEC_DATAIN, 0, IEC_ATNIN, 0, 100000)
        abort = iec_waitForSignals(IEC_DATAIN, 1, IEC_ATNIN, 0, 100000)
    }

    gpio_set_value(IEC_CLKOUT,LOW);
    local_irq_restore(flags);
    udelay(c64slowdown);

    for (len = 0; !abort && len < 8; len++, bits >>= 1) {
        if (!gpio_get_value(IEC_ATNIN) || iec_state != IECOutputState) {
            abort = 1;
            break;
        }
        gpio_set_value(IEC_DATAOUT, bits & 0X01);
        udelay(c64slowdown<<1);
        gpio_set_value(IEC_CLKOUT, HIGH);
        udelay(c64slowdown<<1);
        
        gpio_set_value(IEC_DATAOUT, HIGH);
        gpio_set_value(IEC_CLKOUT, LOW);
    }
    enable_irq(irq_clk);

    if (!abort && (abort = iec_waitForSignals(IEC_DATAIN, 0, IEC_ATNIN, 0, 10000))) {
        if (gpio_get_value(IEC_ATNIN)) {
            abort = 0;
        }
        else {
            iec_state = IECWaitState;
            iec_atnState = IECAttentionState;
        }
    }

    udelay(c64slowdown);

    if (abort && !gpio_get_value(IEC_ATNIN))
        gpio_set_value(IEC_CLKOUT, HIGH);
    
    return abort;
}


int iec_init(void)
{
    int result;
    int minor;

    /* http://www.makelinux.com/ldd3/ */
    /* http://stackoverflow.com/questions/5970595/create-a-device-node-in-code */


    if (!(iec_buffer = kmalloc(IEC_BUFSIZE * sizeof(uint16_t), GFP_KERNEL))) {
        printk(KERN_NOTICE "IEC: failed to allocate buffer\n");
        result = -ENOMEM;
        goto fail_buffer;
    }
    
    iec_inpos = iec_outpos = 0;

    if ((result = gpio_request(IEC_ATNIN, LABEL_ATNIN))) {
        printk(KERN_NOTICE "IEC: GPIO request faiure: %s\n", LABEL_ATNIN);
        goto fail_atnin;
    }
    if ((result = gpio_request(IEC_CLKIN, LABEL_CLKIN))) {
        printk(KERN_NOTICE "IEC: GPIO request faiure: %s\n", LABEL_CLKIN);
        goto fail_clkin;
    }
    if ((result = gpio_request(IEC_DATAIN, LABEL_DATAIN))) {
        printk(KERN_NOTICE "IEC: GPIO request faiure: %s\n", LABEL_DATAIN);
        goto fail_datain;
    }
    if ((result = gpio_request(IEC_CLKOUT, LABEL_CLKOUT))) {
        printk(KERN_NOTICE "IEC: GPIO request faiure: %s\n", LABEL_CLKIN);
        goto fail_clkout;
    }
    if ((result = gpio_request(IEC_DATAOUT, LABEL_DATAOUT))) {
        printk(KERN_NOTICE "IEC: GPIO request faiure: %s\n", LABEL_DATAIN);
        goto fail_dataout;
    }

    if (gpio_direction_input(IEC_ATNIN)<0) {
	printk(KERN_NOTICE "Cannot set ATN as INPUT");
        goto fail_gpio_set;
    }
    if (gpio_direction_input(IEC_CLKIN)<0) {
	printk(KERN_NOTICE "Cannot set CLK as INPUT");
        goto fail_gpio_set;
    }
    if (gpio_direction_input(IEC_DATAIN)<0) {
	printk(KERN_NOTICE "Cannot set DATA as INPUT");
        goto fail_gpio_set;
    }
    if (gpio_direction_input(IEC_DATAOUT)<0) {
	printk(KERN_NOTICE "Cannot set DATA as OUTPUT");
        goto fail_gpio_set;
    }
    if (gpio_direction_input(IEC_CLKOUT)<0) {
	printk(KERN_NOTICE "Cannot set CLK as OUTPUT");
        goto fail_gpio_set;
    }
    gpio_direction_output(IEC_DATAOUT, HIGH);
    gpio_direction_output(IEC_CLKOUT, HIGH);
    printk(KERN_NOTICE "IEC: GPIO SET");
    

    /*
    * interrups on ATN and CLK
    * this avoids to poll the lines for state change
    *
    * When the computer will pull down ATN we will start listeing
    * WHen clock changes, We can sample the bits.
    */
    if ((irq_atn = gpio_to_irq(IEC_ATNIN)) < 0) {
        printk(KERN_NOTICE "IEC: GPIO to IRQ mapping faiure %s\n", LABEL_ATNIN);
        result = irq_atn;
        goto fail_mapatn;
    }

    if ((irq_clk = gpio_to_irq(IEC_CLKIN)) < 0) {
        printk(KERN_NOTICE "IEC: GPIO to IRQ mapping faiure %s\n", LABEL_CLKIN);
        result = irq_clk;
        goto fail_mapclk;
    }

    /* FIXME - don't enable interrupts until user space opens driver */
    if ((result = request_irq(irq_atn, (irq_handler_t) iec_handleATN,
                            IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                            LABEL_ATNIN, LABEL_DEVICE))) {
        printk(KERN_NOTICE "IEC: IRQ Request failure\n");
        goto fail_irqatn;
    }

    if ((result = request_irq(irq_clk, (irq_handler_t) iec_handleCLK,
                            IRQF_TRIGGER_RISING, LABEL_CLKIN, LABEL_DEVICE))) {
        printk(KERN_NOTICE "IEC: IRQ Request failure\n");
        goto fail_irqclk;
    }

    if ((result = alloc_chrdev_region(&iec_devno, IEC_FIRSTDEV, IEC_ALLDEV, LABEL_DEVICE)) < 0) {
        printk(KERN_NOTICE "IEC: cannot register device\n");
        goto fail_chrdev;
    }

    cdev_init(&iec_cdev, &iec_fops);
    iec_cdev.owner = THIS_MODULE;
    if ((result = cdev_add(&iec_cdev, iec_devno, IEC_ALLDEV))) {
        printk(KERN_NOTICE "IEC: failed to add device\n");
        goto fail_cdevadd;
    }

    iec_class = class_create(THIS_MODULE, LABEL_DEVICE);
    if (IS_ERR(iec_class)) {
        printk(KERN_NOTICE "IEC: faile to create class\n");
        goto fail_class;
    }

    device_create(iec_class, NULL, MKDEV(MAJOR(iec_devno), 0), NULL, "iec%i", 0);
    for (minor = IEC_FIRSTDEV; minor < IEC_LASTDEV; minor++)
        device_create(iec_class, NULL, MKDEV(MAJOR(iec_devno), minor), NULL, "iec%i", minor);

    printk(KERN_NOTICE "IEC module loaded. Major: %i\n", MAJOR(iec_devno));

    iec_writeQ = create_singlethread_workqueue(LABEL_WRITE);
    iec_readQ = create_singlethread_workqueue(LABEL_READ);

    return 0;

    fail_class:
        cdev_del(&iec_cdev);
    fail_cdevadd:
        unregister_chrdev_region(iec_devno, IEC_ALLDEV);
    fail_chrdev:
	free_irq(irq_clk, LABEL_DEVICE);
    fail_irqclk:
        free_irq(irq_atn, LABEL_DEVICE);
    fail_irqatn:
    fail_mapclk:
    fail_mapatn:
    fail_gpio_set:
    fail_dataout:
        gpio_free(IEC_DATAOUT);
    fail_clkout:
        gpio_free(IEC_CLKOUT);
    fail_datain:
        gpio_free(IEC_DATAIN);
    fail_clkin:
        gpio_free(IEC_CLKIN);
    fail_atnin:
        gpio_free(IEC_ATNIN);
    fail_buffer:
        kfree(iec_buffer);

    return result;
}

void iec_cleanupDevice(int minor)
{
  iec_device *device;
  iec_io *io, *ioNext;

  if ((device = iec_openDevices[minor])) {
    if (down_interruptible(&device->lock)) {
      printk(KERN_NOTICE "IEC: unable to cleanup device %i\n", minor);
      return;
    }

    iec_openDevices[minor] = NULL;
    up(&device->lock);
    io = device->in.head;
    while (io) {
      ioNext = io->next;
      kfree(io);
      io = ioNext;
    }

    kfree(device);
  }
  
  return;
}
    
void iec_cleanup(void)
{
    int minor;

    destroy_workqueue(iec_writeQ);
    destroy_workqueue(iec_readQ);
    device_destroy(iec_class, MKDEV(MAJOR(iec_devno), 0));
    for (minor = IEC_FIRSTDEV; minor < IEC_LASTDEV; minor++) {
        device_destroy(iec_class, MKDEV(MAJOR(iec_devno), minor));
        iec_cleanupDevice(minor);
    }
    class_destroy(iec_class);
    cdev_del(&iec_cdev);
    unregister_chrdev_region(iec_devno, IEC_ALLDEV);
    kfree(iec_buffer);

    free_irq(irq_atn, LABEL_DEVICE);
    free_irq(irq_clk, LABEL_DEVICE);
    gpio_free(IEC_ATNIN);
    gpio_free(IEC_CLKIN);
    gpio_free(IEC_DATAIN);
    gpio_free(IEC_CLKOUT);
    gpio_free(IEC_DATAOUT);

    printk(KERN_NOTICE "IEC module removed\n");
    return;
}

/*
 * /dev/iec<minor>
 *
 * A device can be open by ONE process.
 *
 *
 */
int iec_open(struct inode *inode, struct file *filp)
{
    int minor = iminor(inode);
    iec_device *device;

    printk(KERN_NOTICE "IEC: opening device %i\n", minor);
    device = iec_openDevices[minor];

    /* FIXME - allow more than one process to open at once */
    if (device) {
        printk(KERN_NOTICE "IEC: another process already has device %i open\n", minor);
        return 0;
    }

    device = iec_openDevices[minor] = kmalloc(sizeof(iec_device), GFP_KERNEL);
    device->in.head = device->in.tail = device->in.cur = NULL;
    device->out.outpos = 0;
    device->flags = filp->f_flags;
    sema_init(&device->lock, 1);
    filp->private_data = device;
    return 0;
}

int iec_close(struct inode *inode, struct file *filp)
{
    int minor = iminor(inode);
    iec_cleanupDevice(minor);
    return 0;
}


unsigned int iec_poll(struct file *filp, poll_table *wait)
{
    int avail = 0;
    iec_device *device = filp->private_data;
    iec_io *io;
    unsigned int pollMask;

    io = device->in.head;
    poll_wait(filp, &iec_canRead, wait);
    if (io && io != device->in.cur)
        avail = (io->header.len + sizeof(io->header)) - io->outpos;

    //printk(KERN_NOTICE "IEC: polling %i %x %i\n", avail, (size_t) io, io ? io->outpos : -1);
    pollMask = POLLOUT | POLLWRNORM;
    
    if (avail)
        pollMask |= POLLIN | POLLRDNORM;

    return pollMask;
}

int iec_unlinkIO(iec_device *device)
{
    iec_chain *chain;
    iec_io *io;


    if (down_interruptible(&device->lock))
        return 0;
    
    chain = &device->in;
    io = chain->head;
    chain->head = io->next;
    kfree(io);
    up(&device->lock);
    return 1;
}
    
/*
 * Reads the data, if avaialable and returns the data to the caller. 
 * Blocking call
 *
 */
ssize_t iec_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
    unsigned long remaining;
    int avail = 0;
    iec_device *device = filp->private_data;
    iec_io *io;
    unsigned char *wbuf;


    do {
        io = device->in.head;
        if (io && io != device->in.cur)
            avail = (io->header.len + sizeof(io->header)) - io->outpos;
        //printk(KERN_NOTICE "IEC: waiting for data %i\n", avail);
        if (!avail && wait_event_interruptible(iec_canRead, iec_readAvail))
            return -ERESTARTSYS;
        iec_readAvail = 0;
    } while (!avail);

    //printk(KERN_NOTICE "IEC: %i bytes avail:", avail);
    if (down_interruptible(&device->lock)) {
        printk(KERN_NOTICE "IEC: unable to lock IO\n");
        return 0;
    }

    if (io->outpos < sizeof(io->header)) {
        wbuf = (unsigned char *) &io->header;
        wbuf += io->outpos;
        if (count > sizeof(io->header) - io->outpos)
            count = sizeof(io->header) - io->outpos;
    }
    else {
        int pos;
        pos = io->outpos - sizeof(io->header);
        if (count > io->header.len - pos)
            count = io->header.len - pos;
        wbuf = &io->data[pos];
    }

    {
        int i;
        for (i = 0; i < count; i++)
            printk(" %02x", wbuf[i]);
        printk("\n");
    }

    remaining = copy_to_user(buf, wbuf, count);
    up(&device->lock);

    count -= remaining;
    io->outpos += count;
    *f_pos += count;

    if (io->outpos == io->header.len + sizeof(io->header) &&
        !iec_unlinkIO(device))
        return -ERESTARTSYS;

    return count;
}


ssize_t iec_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    iec_device *device = filp->private_data;
    iec_io *io;
    unsigned long remaining;
    unsigned char *wbuf;
    size_t len, offset;
    int abort, val;
    char data[2];

    printk(KERN_NOTICE "IEC: request to write %i bytes\n", count);

    /* FIXME - really only care about EOI unless minor is 0 and acting as master */
    /* FIXME - how to detect EOI? */
  
    offset = 0;
    abort = 0;
    while (!abort && count > 0) {
        len = count;
        io = &device->out;

        if (io->outpos < sizeof(io->header)) {
            wbuf = (unsigned char *) &io->header;
            wbuf += io->outpos;
            if (len > sizeof(io->header) - io->outpos)
                len = sizeof(io->header) - io->outpos;

            printk(KERN_NOTICE "IEC: write header %i\n", len);
            remaining = copy_from_user(wbuf, buf+offset, len);
            len -= remaining;
            io->outpos += len;

            if (io->outpos == sizeof(io->header)) {
                int i;
                unsigned char *p = (unsigned char *) &io->header;
                printk(KERN_NOTICE "IEC: header");
                for (i = 0; i < io->outpos; i++)
                    printk(" %02x", p[i]);
                printk("\n");
            }
      
            if (io->outpos == sizeof(io->header) && !io->header.len) {
                //printk(KERN_NOTICE "IEC: file not found\n");
                iec_state = IECWaitState;
                iec_releaseBus();
            }
        }
        else {
            if (iec_state != IECOutputState) {
                len = 0;
                abort = 1;
            }
            else {
                //printk(KERN_NOTICE "IEC: sending bytes\n");
                len = 1;
                remaining = copy_from_user(data, buf+offset, len);
                len -= remaining;
                if (len) {
                    val = data[0];
                    if (io->header.eoi && io->outpos + 1 == io->header.len + sizeof(io->header))
                        val |= DATA_EOI;
#if 1
                    printk(KERN_NOTICE "IEC: sending %02x, %i of %i, %i\n", val, io->outpos - sizeof(io->header), io->header.len, io->header.eoi);
#endif
                    abort = iec_writeByte(val);
                    if (abort) {
                        printk(KERN_NOTICE "IEC: write abort %i\n", gpio_get_value(IEC_CLKIN));
                        len = 0;
                    }
                    else
                        io->outpos += len;
                }
            }
        }
        if (abort || io->outpos == io->header.len + sizeof(io->header))
            io->outpos = 0;
        count -= len;
        offset += len;
    }
    printk(KERN_NOTICE "IEC: write used %i bytes\n", offset);
    *f_pos += offset;
    return offset;
}

module_init(iec_init);
module_exit(iec_cleanup);

/****************************************************************************/
/* Module licensing/description block.                                      */
/****************************************************************************/
MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
