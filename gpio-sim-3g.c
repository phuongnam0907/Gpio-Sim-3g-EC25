#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>       // for fs function like alloc_chrdev_region / operation file
#include <linux/types.h>
#include <linux/device.h>   // for device_create and class_create
#include <linux/uaccess.h>  // for copy to/from user function
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/of.h>       // access device tree file
#include <linux/delay.h>
#include <linux/slab.h>     // kmalloc, kcallloc, ....
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/kthread.h>
#include <linux/kdev_t.h>

#define DRIVER_NAME "gpio-sim-3g"
#define FIRST_MINOR 0

// new code - time unit: milisecond
#define DEFAULT_POWER_ON_PREPARE_TIME 30
#define DEFAULT_POWER_ON_TIME 110
#define DEFAULT_POWER_OFF_TIME 660
#define DEFAULT_WAIT_OFF_TIME 30000
#define DEFAULT_RESET_TIME 440

#define CONVERT_MICROSECONDS 1000

#define PDEBUG(fmt,args...) printk(KERN_DEBUG"%s: "fmt,DRIVER_NAME, ##args)
#define PERR(fmt,args...) printk(KERN_ERR"%s: "fmt,DRIVER_NAME,##args)
#define PINFO(fmt,args...) printk(KERN_INFO"%s: "fmt,DRIVER_NAME, ##args)

typedef struct gpio_data{
    unsigned gpio;
    bool active_low;
} gpio_data_t;

typedef struct dev_private_data {
	struct mutex lock;
    struct device *dev;
    const char *name;
    gpio_data_t reset;
    gpio_data_t power;
    gpio_data_t usb;
    int power_on_pre_time;
    int power_on_time;
    int power_off_time;
    int wait_off_time;
    int reset_time;
} dev_private_data_t;

typedef struct platform_private_data{
    struct class * dev_class;
    int number_device;
    dev_private_data_t devices [];
} platform_private_data_t;

dev_t dev_num ;
int dev_num_major = 0;

void reset_dev(dev_private_data_t *);
void power_on_dev(dev_private_data_t *);
void power_off_dev(dev_private_data_t *);
void update_firmware(dev_private_data_t *);
void delay_time_micro (int);

static inline int sizeof_platform_data(int number_device)
{
	return sizeof(platform_private_data_t) +
		(sizeof(dev_private_data_t) * number_device);
}

void reset_dev(dev_private_data_t *data)
{
    mutex_lock(&data->lock);
    gpio_set_value(data->usb.gpio, 0 ^ data->usb.active_low);
    gpio_set_value(data->reset.gpio, 1 ^ data->reset.active_low);
    gpio_set_value(data->power.gpio, 0 ^ data->power.active_low);

    delay_time_micro(data->reset_time*CONVERT_MICROSECONDS);

    gpio_set_value(data->reset.gpio, 0 ^ data->reset.active_low);

    mutex_unlock(&data->lock);
}

void power_on_dev(dev_private_data_t *data)
{
    mutex_lock(&data->lock);

    delay_time_micro(data->power_on_pre_time*CONVERT_MICROSECONDS);

    gpio_set_value(data->usb.gpio, 0 ^ data->usb.active_low);
    gpio_set_value(data->reset.gpio, 0 ^ data->reset.active_low);
    gpio_set_value(data->power.gpio, 1 ^ data->power.active_low);

    delay_time_micro(data->power_on_time*CONVERT_MICROSECONDS);

    gpio_set_value(data->power.gpio, 0 ^ data->power.active_low);

    mutex_unlock(&data->lock);
}

void power_off_dev(dev_private_data_t *data)
{
    mutex_lock(&data->lock);

    gpio_set_value(data->usb.gpio, 0 ^ data->usb.active_low);
    gpio_set_value(data->reset.gpio, 0 ^ data->reset.active_low);
    gpio_set_value(data->power.gpio, 1 ^ data->power.active_low);

    delay_time_micro(data->power_off_time*CONVERT_MICROSECONDS);

    gpio_set_value(data->power.gpio, 0 ^ data->power.active_low);

    delay_time_micro(data->wait_off_time*CONVERT_MICROSECONDS);

    PINFO("Module Sim POWER OFF.");
    
    mutex_unlock(&data->lock);
}

void boot_update_firmware(dev_private_data_t *data)
{
    mutex_lock(&data->lock);
    // OFF DEVICE
    gpio_set_value(data->usb.gpio, 0 ^ data->usb.active_low);
    gpio_set_value(data->reset.gpio, 0 ^ data->reset.active_low);
    gpio_set_value(data->power.gpio, 1 ^ data->power.active_low);

    delay_time_micro(data->power_off_time*CONVERT_MICROSECONDS);

    gpio_set_value(data->power.gpio, 0 ^ data->power.active_low);

    delay_time_micro(data->wait_off_time*CONVERT_MICROSECONDS);

    // SET MODE UPDATE
    gpio_set_value(data->usb.gpio, 1 ^ data->usb.active_low); 

    // ON DEVICE
    delay_time_micro(data->power_on_pre_time*CONVERT_MICROSECONDS);

    gpio_set_value(data->power.gpio, 1 ^ data->power.active_low);

    delay_time_micro(data->power_on_time*CONVERT_MICROSECONDS);

    gpio_set_value(data->power.gpio, 0 ^ data->power.active_low);

    PINFO("BOOT PROGRAM - UPDATING FIRMWARE VIA USB ...");

    mutex_unlock(&data->lock);
}

void delay_time_micro (int time)
{
    if (time <= 10)
        udelay(time);
    else if (time <= 15000)
        usleep_range(time, time + 10);
    else 
        msleep(time/1000);
}

/***********************************/
/***** define device attribute *****/
/***********************************/

static ssize_t mode_store(struct device *dev, struct device_attribute *attr, const char *buff, size_t len)
{
    dev_private_data_t *data = dev_get_drvdata(dev);
    PINFO ("inside mode_store function, input is \"%s\",size %d byte\n", buff, len);

    if (strcmp(buff,"on") == 0)
    {
        power_on_dev(data);
    } 
    else if (strcmp(buff,"off") == 0)
    {
        power_off_dev(data);
    }
    else if (strcmp(buff,"reset") == 0)
    {
        reset_dev(data);
    }
    else if (strcmp(buff,"program") == 0)
    {
        boot_update_firmware(data);
    }
    else PINFO ("Mode input note valid, please enter \"on\", \"off\", \"reset\", \"program\" (without quote)\n");
    
    return len;
} 

static struct device_attribute dev_class_attr[] = {
    __ATTR(mode,0222,NULL,mode_store),
    __ATTR_NULL,
};

/***************************/
/*****module init + exit****/
/***************************/
static int driver_probe (struct platform_device *pdev)
{
    int number_device, res;
    struct device_node *np = pdev->dev.of_node;
    struct device_node *child ;
    platform_private_data_t *data;

    PINFO ("Driver module init!\n");
    PINFO ("Node name %s\n",pdev->dev.of_node->name );

    number_device = of_get_child_count(np);
    if (!number_device)
		return -ENODEV;

    // create private data
    data = (platform_private_data_t*)kcalloc(1, sizeof_platform_data(number_device), GFP_KERNEL);
    data->number_device = 0;

    // create class 
    data->dev_class = class_create(THIS_MODULE, DRIVER_NAME);
    if (IS_ERR(data->dev_class))
    {
        PERR("Class create fail, error code: %d\n", (int)data->dev_class);

        goto error_class;
    }
    data->dev_class->dev_attrs = dev_class_attr;
    
    // number_device = 0;
    for_each_child_of_node(np, child) {
        dev_private_data_t *device = &data->devices[data->number_device++];
        u32 temp;
        mutex_init(&device->lock);
        
		device->name = of_get_property(child, "label", NULL) ? : child->name;

        // get gpio properties and create device
        device->reset.active_low = of_property_read_bool(child,"reset-active-low");
        device->power.active_low = of_property_read_bool(child,"power-active-low");
        device->usb.active_low = of_property_read_bool(child,"usb-active-low");

        res = of_property_read_u32(child, "power-on-pre-time", &temp);
        if (!res)
            device->power_on_pre_time = temp;
        else
            device->power_on_pre_time = DEFAULT_POWER_ON_PREPARE_TIME;

        res = of_property_read_u32(child, "power-on-time", &temp);
        if (!res)
            device->power_on_time = temp;
        else
            device->power_on_time = DEFAULT_POWER_ON_TIME;

        res = of_property_read_u32(child, "power-off-time", &temp);
        if (!res)
            device->power_off_time = temp;
        else
            device->power_off_time = DEFAULT_POWER_OFF_TIME;

        res = of_property_read_u32(child, "wait-off-time", &temp);
        if (!res)
            device->wait_off_time = temp;
        else
            device->wait_off_time = DEFAULT_WAIT_OFF_TIME;

        res = of_property_read_u32(child, "reset-time", &temp);
        if (!res)
            device->reset_time = temp;
        else
            device->reset_time = DEFAULT_RESET_TIME;

        device->dev = device_create(data->dev_class, &pdev->dev, 0, device, "%s", device->name);

        if (IS_ERR(device->dev))
        {
            PERR("Device for %s create fall, error code: %d\n", device->name, (int)device->dev);

            goto error_device;
        }

        // get gpio number from device tree
        device->reset.gpio = of_get_named_gpio(child, "reset", 0);
        if (!device->reset.gpio)
        {
            PERR ("Can't get reset gpio from %s, error code: %d\n", device->name, device->reset.gpio);

            goto error_reset_gpio;
        } 
        device->power.gpio = of_get_named_gpio(child, "power", 0);
        if (!device->power.gpio)
        {
            PERR ("Can't get power gpio from %s, error code: %d\n", device->name, device->power.gpio);

            goto error_power_gpio;
        } 
        device->usb.gpio = of_get_named_gpio(child, "usb", 0);
        if (!device->usb.gpio)
        {
            PERR ("Can't get usb gpio from %s, error code: %d\n", device->name, device->usb.gpio);

            goto error_usb_gpio;
        } 

        // request gpio and init
        if (!gpio_is_valid(device->reset.gpio))
        {
            PINFO ("Reset gpio number is invalid \n");

            goto error_gpio_init;
        }
        else devm_gpio_request_one(device->dev, device->reset.gpio, GPIOF_OUT_INIT_LOW, "reset");

        if (!gpio_is_valid(device->power.gpio))
        {
            PINFO ("Power gpio number is invalid \n");

            goto error_gpio_init;
        }
        else devm_gpio_request_one(device->dev, device->power.gpio, GPIOF_OUT_INIT_LOW, "power");

        if (!gpio_is_valid(device->usb.gpio))
        {
            PINFO ("Usb gpio number is invalid \n");

            goto error_gpio_init;
        }
        else devm_gpio_request_one(device->dev, device->usb.gpio, GPIOF_OUT_INIT_LOW, "usb");

	    PINFO("Device %s configuration : \n", device->name);
        PINFO("\tpower_on_pre_time: %d\n", device->power_on_pre_time);
        PINFO("\tpower_on_time: %d\n", device->power_on_time);
        PINFO("\tpower_off_time: %d\n", device->power_off_time);
        PINFO("\twait_off_time: %d\n", device->wait_off_time);
        PINFO("\treset_time: %d\n", device->reset_time);
        PINFO("\trset_active_low: %s\n", device->reset.active_low ? "true" : "false");
        PINFO("\tpower_active_low: %s\n", device->power.active_low ? "true" : "false");
        PINFO("\tusb_active_low: %s\n", device->usb.active_low ? "true" : "false");
        PINFO("\treset_gpio_number: %d\n", device->reset.gpio);
        PINFO("\tpower_gpio_number: %d\n", device->power.gpio);
        PINFO("\tusb_gpio_number: %d\n", device->usb.gpio);
        // ++number_device;

        continue;

        error_gpio_init:
            gpio_free(device->usb.gpio);
        error_usb_gpio:
            gpio_free(device->power.gpio);
        error_power_gpio:
            gpio_free(device->reset.gpio);
        error_reset_gpio:
            device_destroy(data->dev_class, dev_num);
        error_device:
            continue;
    }

    platform_set_drvdata(pdev, data);

    return 0;

    //error handle
error_class:
    return -1;

}

static int driver_remove(struct platform_device *pdev)
{
    platform_private_data_t *data = platform_get_drvdata(pdev);
    PINFO("Driver module remove from kernel\n");
    class_destroy(data->dev_class);
    kfree(data);
    platform_set_drvdata(pdev, NULL);

    return 0;
}

static const struct of_device_id gpio_sim_dst[]={
    { .compatible = "gpio-sim-3g", },
    {}
};

MODULE_DEVICE_TABLE(of, gpio_sim_dst);	

static struct platform_driver gpio_platform = {
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,   
        .of_match_table = of_match_ptr (gpio_sim_dst),
    },
    .probe = driver_probe,
    .remove = driver_remove,
};

module_platform_driver(gpio_platform);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Le Phuong Nam <le.phuong.nam@styl.solutions>");

/*
*======================= DEFINE GPIOS ==================
*
* SC20_GPIO_8 => EC25_USB_BOOT => gpio_boot_usb
* SC20_GPIO_9 => EC25_PWRKEY => gpio_power
* SC20_GPIO_10 => EC25_RESET => gpio_reset
*
*======================= DEFINE TIMES ==================
*
* power_on_pre_time = 30ms
* power_on_time = 110ms
* power_off_time = 660ms
* wait_off_time = 30s
* reset_time = 400ms
*
*===================== DESCRIBE ACTION =================
* MODE: ON
*   -> check VBAT on -> wait power_on_pre_time (30ms)
*   gpio_power + gpio_reset -> set High
*   gpio_power: current status = High
*   -> set Low -> wait power_on_time (110ms) -> set High
*
* MODE: OFF
*   gpio_power: current status = High
*   -> set Low -> wait power_off_time (660ms) -> set High
*   -> wait wait_off_time (30s) -> print module off success
*
* MODE: RESET
*   gpio_reset: current status = High
*   -> set Low -> wait reset_time (400ms) -> set High
*
* MODE: UPDATE FIRMWARE MODULE VIA USB
*   gpio_boot_usb: current status = LOW
*   -> set High ... Update firmware -> set LoW
*
* NOTE: when startup
*   VBAT: always on
*   Reset: on
*   power: on
* then start [MODE: ON] to run the module
*/

// DEVICE TREE
/*

gpio-sim-3g{
    compatible = "gpio-sim-3g";
    status = "okay";

    pinctrl-0 = <&reset_pin_ec25>;
    ec25{
            reset = <&msm_gpio 10 1>;
            power = <&msm_gpio 9 1>;
            usb = <&msm_gpio 8 0>;
            power-on-pre-time = <30>;
            power-on-time = <110>;
            power-off-time = <660>;
            wait-off-time = <30000>;
            reset-time = <440>;
            power-active-low;
            reset-active-low;

    };
};

*/

// PIN CONTROL
/*

ec25-pin {
    qcom,pins = <&gp 8>, <&gp 9>, <&gp 10>;
    qcom,num-grp-pins = <3>;
    label = "pinctl-ec25-sim-3g";
    reset_pin_ec25: pinctl-ec25-sim-3g {
        drive-strength = <2>;
        bias-disable;
        output-low;
    };
};

*/

// Makefile => obj-$(CONFIG_GPIO_RESET_EC25)   += gpio-sim-3g.o
// Kconfig
/*

config GPIO_RESET_EC25
	bool "gpio_reset_ec25"
	help
	  gpio to control module sim 3g (EC25) connect to it

*/

// defconfig => 
/*

#6. gpio-reset-ec25
CONFIG_GPIO_RESET_EC25=y

*/

/* DIP-SWITCH-STATUS

echo 937 > /sys/class/gpio/export
echo "in" > /sys/class/gpio/gpio937/direction
echo 939 > /sys/class/gpio/export
echo "in" > /sys/class/gpio/gpio939/direction
echo 946 > /sys/class/gpio/export
echo "in" > /sys/class/gpio/gpio946/direction
echo 945 > /sys/class/gpio/export
echo "in" > /sys/class/gpio/gpio945/direction

*/