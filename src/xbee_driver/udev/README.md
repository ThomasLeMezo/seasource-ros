
# Udev rules for XBee devices
https://wiki.archlinux.org/title/udev


```bash
udevadm info --attribute-walk --path=$(udevadm info --query=path --name=/dev/ttyUSB0)
```


```bash
udevadm monitor --property --udev
```


```bash
udevadm info --attribute-walk --name=/dev/ttyUSB0
```

