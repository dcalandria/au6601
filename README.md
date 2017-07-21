# au6601
Linux kernel module for Alcor Micro AU6601 Secure Digital Host Controller

...

This driver is based on documentation which was based on my RE-work and
comparision with other MMC drivers.
It works in legacy mode and can provide 20MB/s R/W spead for most modern
SD cards, even if at least 40MB/s should be possible on this hardware.
It was not possible provide description for all register. But some of them
are important to make this hardware work in some unknown way.

Biggest part of RE-work was done by emulating AU6601 in QEMU.

Signed-off-by: Oleksij Rempel
               linux AT rempel-privat.de

https://launchpadlibrarian.net/175159219/[PATCH] mmc%3A add new au6601 driver.eml             
