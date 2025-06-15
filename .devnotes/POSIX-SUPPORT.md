# POSIX Support in Zephyr

Zephyr provides a subset of POSIX APIs through the `posix` module with `CONFIG_POSIX_API=y`.

## Supported POSIX APIs

<https://docs.zephyrproject.org/3.6.0/services/portability/posix/option_groups/index.html>

## Important Notes

* Since Zephyr does not support multi-processing, POSIX threads are implemented as kernel threads.

## More info at <https://docs.zephyrproject.org/3.6.0/services/portability/posix/overview/index.html>
