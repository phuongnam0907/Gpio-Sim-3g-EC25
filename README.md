# Gpio-Sim-3g-EC25

[Datasheet of EC25 - Quectel](https://www.quectel.com/UploadImage/Downlad/Quectel_EC25_Hardware_Design_V1.3.pdf)

## DEFINATION

### DEFINE GPIOS

```
SC20_GPIO_8 => EC25_USB_BOOT => gpio_boot_usb
SC20_GPIO_9 => EC25_PWRKEY => gpio_power
SC20_GPIO_10 => EC25_RESET => gpio_reset
```

### DEFINE TIMES

```
power_on_pre_time = 30ms
power_on_time = 110ms
power_off_time = 660ms
wait_off_time = 30s
reset_time = 400ms
```

## DESCRIBE ACTION
<b> 1. MODE: ON </b>
```
-> check VBAT on -> wait power_on_pre_time (30ms)
gpio_power + gpio_reset -> set High
gpio_power: current status = High
-> set Low -> wait power_on_time (110ms) -> set High
```

<b> 2. MODE: OFF </b>
```
gpio_power: current status = High
-> set Low -> wait power_off_time (660ms) -> set High
-> wait wait_off_time (30s) -> print module off success
```

<b> 3. MODE: RESET </b>
```
gpio_reset: current status = High
-> set Low -> wait reset_time (400ms) -> set High
```

<b> 4. MODE: UPDATE FIRMWARE MODULE VIA USB </b>
```
gpio_boot_usb: current status = LOW
-> set High ... Update firmware -> set LoW
```

<b>NOTE</b><br>
When board startup...<br>
* VBAT: always on
* Reset: on
* Power: on
<br>Then start [MODE: ON] to run the module


## Add these lines

### DEVICE TREE

```
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
```

### PIN CONTROL

```
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
```

### Makefile

```
obj-$(CONFIG_GPIO_RESET_EC25)   += gpio-sim-3g.o
```

### Kconfig

```
config GPIO_RESET_EC25
	bool "gpio_reset_ec25"
	help
	  gpio to control module sim 3g (EC25) connect to it
```


### defconfig

```
#6. gpio-reset-ec25
CONFIG_GPIO_RESET_EC25=y
```

## Add "DIP-SWITCH-STATUS"

```
echo 937 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio937/direction
echo 939 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio939/direction
echo 946 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio946/direction
echo 945 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio945/direction
chown system system /sys/class/gpio/gpio937/value
chmod 0666 /sys/class/gpio/gpio937/value
chown system system /sys/class/gpio/gpio939/value
chmod 0666 /sys/class/gpio/gpio939/value
chown system system /sys/class/gpio/gpio946/value
chmod 0666 /sys/class/gpio/gpio946/value
chown system system /sys/class/gpio/gpio945/value
chmod 0666 /sys/class/gpio/gpio945/value
```
