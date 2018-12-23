#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/input/touchscreen.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/poll.h>

#define GPIO 37    //PB5
#define GPIO_INT_NAME  "pg5_int5"

#define GPIO_HIGH gpio_get_value(GPIO)
#define GPIO_LOW (gpio_get_value(GPIO) == 0)
short int irq_any_gpio    = 0;
int count =0;
static struct timer_list buttons_timer;
static DECLARE_WAIT_QUEUE_HEAD(button_waitq);
enum { falling, rising } type;
/* 中断事件标志, 中断服务程序将它置1，drv_read将它清0 */
static volatile int ev_press = 0;
static struct fasync_struct *button_async;

/* 键值: 按下时, 0x01,*/
/* 键值: 松开时, 0x81, */
static unsigned char key_val;

//static DECLARE_MUTEX(button_lock);     //定义互斥锁
struct semaphore button_lock;

static struct class *drv_class;
static struct device	*drv_class_dev;

static int major;
static irqreturn_t r_irq_handler(int irq, void *dev_id)
 {
    count++;
    printk(KERN_DEBUG "interrupt received (irq: %d)\n", irq);
	mod_timer(&buttons_timer, jiffies+HZ/100);
	/*
	if (irq == gpio_to_irq(GPIO)) 
    {

        type = GPIO_LOW ? falling : rising;

        if(type == falling)
        {
            printk("gpio pin is low\n");    
        }
        else
            printk("gpio pin is high\n");

    }
	*/
    return IRQ_HANDLED;
}
static void buttons_timer_function(unsigned long data)
{
	int pinval = -1;
	
	pinval = gpio_get_value(GPIO);

	if (pinval)
	{
		printk("gpio pin is high\n");
		key_val  = 0x81;
	}
	else
	{
		 printk("gpio pin is low\n");
		 key_val = 0x00;
	}

    ev_press = 1;                  /* 表示中断发生了 */
    wake_up_interruptible(&button_waitq);   /* 唤醒休眠的进程 */
	
	kill_fasync (&button_async, SIGIO, POLL_IN);
}
void r_int_config(void) {

   if (gpio_request(GPIO, GPIO_INT_NAME )) 
   {
      printk("GPIO request failure: %s\n", GPIO_INT_NAME );
      return;
   }

   if ( (irq_any_gpio = gpio_to_irq(GPIO)) < 0 ) {
      printk("GPIO to IRQ mapping failure %s\n",GPIO_INT_NAME );
      return;
   }

   printk(KERN_NOTICE "Mapped int %d\n", irq_any_gpio);

   if (request_irq(irq_any_gpio,(irq_handler_t ) r_irq_handler, IRQF_TRIGGER_FALLING , GPIO_INT_NAME, NULL)) 
   {
      printk("Irq Request failure\n");
      return;
   }

   return;
}
/* fo --read */
ssize_t drv_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	int n = 0;	
	if (size != 1)
		return -EINVAL;

	if (file->f_flags & O_NONBLOCK)
	{
		if (!ev_press)
			return -EAGAIN;
	}
	else
	{
		/* 如果没有按键动作, 休眠 */
		wait_event_interruptible(button_waitq, ev_press);
	}

	n = copy_to_user(buf, &key_val, 1);
	if(n < 0)
		printk("cpy no data\n");
	ev_press = 0;
	
	return 1;
}
/* fo --open */
static int drv_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	
#if 0	
	if (!atomic_dec_and_test(&canopen))
	{
		atomic_inc(&canopen);
		return -EBUSY;
	}
#endif		

	if (file->f_flags & O_NONBLOCK)
	{
		if (down_trylock(&button_lock))
			return -EBUSY;
	}
	else
	{
		/* 获取信号量 */
		down(&button_lock);
	}

    r_int_config();


	return ret;
}
/* fo --close */
int drv_close(struct inode *inode, struct file *file)
{
	int ret = 0;

	if(irq_any_gpio)
		free_irq(irq_any_gpio, NULL);
	gpio_free(GPIO);
	up(&button_lock);
	return ret;
}
void r_int_release(void) {
   gpio_free(GPIO);
   del_timer(&buttons_timer); 
   unregister_chrdev(major, "init_drv");
   device_destroy(drv_class, MKDEV(major, 0));
   class_destroy(drv_class);
   return;
}
static int drv_fasync (int fd, struct file *filp, int on)
{
	printk("driver: drv_fasync\n");
	return fasync_helper (fd, filp, on, &button_async);
}

static unsigned drv_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
	poll_wait(file, &button_waitq, wait); // 不会立即休眠

	if (ev_press)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}


static struct file_operations drv_fops = {
    .owner   =  THIS_MODULE,    /* 这是一个宏，推向编译模块时自动创建的__this_module变量 */
    .open    =  drv_open,     
	.read	 =	drv_read,	   
	.release =  drv_close,
	.poll    =  drv_poll,
	.fasync	 =  drv_fasync,
};

static int drv_init(void)
{
	init_timer(&buttons_timer);
	buttons_timer.function = buttons_timer_function;
	//buttons_timer.expires  = 0;
	//setup_timer(&buttons_timer, buttons_timer_function, 0);
	
	add_timer(&buttons_timer); 

	major = register_chrdev(0, "init_drv", &drv_fops);

	drv_class = class_create(THIS_MODULE, "init_drv");

	/* 为了让mdev根据这些信息来创建设备节点 */
	drv_class_dev = device_create(drv_class, NULL, MKDEV(major, 0), NULL, "init_tes"); /* /dev/buttons */

	sema_init(&button_lock, 1);

	return 0;
}

static int init_module_gpio(void)
{
        printk("init_module_gpio\n");
		drv_init();
        return 0;
}

module_init(init_module_gpio);    // Do some better naming
module_exit(r_int_release);
MODULE_LICENSE("GPL");
