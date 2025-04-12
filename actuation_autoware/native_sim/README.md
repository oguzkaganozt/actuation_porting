# Actuation Autoware on Native Sim

In order to run the actuation autoware on a native simulation environment, we need to create a zeth interface, then configure it to connect to the internet via a host interface, and keep it running.

```bash
sudo ./net-setup.sh --dst-iface <host_interface>
```

Then we can execute the native binary inside the devcontainer, which can be run found in the root of the repository.

```bash
./launch_dev.sh
```
