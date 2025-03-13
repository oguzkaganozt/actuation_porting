# Zephyr native_sim

Zephyr supports a native_sim board that allows you to run Zephyr applications on a POSIX system. With console backend and zeth device enabled.

In order to run the native_sim board, you need to create a zeth device by using:

```bash
test-tools/net-setup.sh
```

This will create a zeth tun-tap device.

## More Information
https://docs.zephyrproject.org/3.6.0/boards/posix/native_sim/doc/index.html#native-sim