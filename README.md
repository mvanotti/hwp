A simple program to try to understand Intel Hardware P-States management on Linux.

Requirements:

To read msrs and cpuid, you will need:

```shell
# modprobe msr
# modprobe cpuid
```

To write to msrs, if you are using a recent kernel version, you will need to disable the kernel lockdown. See [`man kernel_lockdown`](https://man7.org/linux/man-pages/man7/kernel_lockdown.7.html)

Disclaimer: I don't know rust and I have no idea what I am doing.
