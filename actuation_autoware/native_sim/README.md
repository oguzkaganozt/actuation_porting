# Actuation Autoware on Native Sim

This is a native simulation environment for the actuation autoware.

First we need to create zeth interface, keep it running.

```bash
sudo ./net-setup.sh
```

Then we need to configure zeth interface to connect to the internet.

```bash
sudo ./configure_zeth.sh <your_default_interface>
```

Then we can execute the native binary inside the devcontainer, which can be run at the root of the repository.

```bash
./launch_dev.sh
```
