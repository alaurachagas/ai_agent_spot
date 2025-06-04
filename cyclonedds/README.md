# Cyclone DDS tuning

## Issue: 
Cyclone DDS is not delivering large messages reliably, despite using reliable settings and transferring over a wired network.
## Solution: 
Increase the maximum Linux kernel receive buffer size and the minimum socket receive buffer size that Cyclone uses.

Adjustments to solve for a 9MB message:

Set the maximum receive buffer size, rmem_max, by running:

    sudo sysctl -w net.core.rmem_max=2147483647

Or permanently set it by editing the /etc/sysctl.d/10-cyclone-max.conf file to contain:
This config file needs to be added to all hosts, e.g. server and robot

    net.core.rmem_max=2147483647

Next, to set the minimum socket receive buffer size that Cyclone requests, write out a configuration file for Cyclone to use while starting, like so:

```
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config
https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <Internal>
            <MinimumSocketReceiveBufferSize>10MB</MinimumSocketReceiveBufferSize>
        </Internal>
    </Domain>
</CycloneDDS>
```

Then, whenever you are going to run a node, set the following environment variable:

CYCLONEDDS_URI=file:///absolute/path/to/config_file.xml
