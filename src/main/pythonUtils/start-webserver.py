#!/usr/bin/env python3
import subprocess
subprocess.run([
    "ssh", f"admin@10.4.47.2",
    "/usr/local/natinst/etc/init.d/systemWebServer", "start"
])