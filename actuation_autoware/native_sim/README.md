# Actuation Autoware on Native Sim

This is a native simulation environment for the actuation autoware.

First we need to create zeth interface, then configure it to connect to the internet via an internet interface, and keep it running.

```bash
sudo ./net-setup.sh --dst-iface <host_interface>
```

Then we can execute the native binary inside the devcontainer, which can be run found in the root of the repository.

```bash
./launch_dev.sh
```
