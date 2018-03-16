CONTIKI_PROJECT = ip64-router
all: $(CONTIKI_PROJECT)
CONTIKI=../..

# Currently only supported on Orion and cc26xx cc13xx devices with 802.15.4 and Ethernet driver integration.
PLATFORMS_ONLY = zoul srf06-cc26xx
BOARDS_ONLY = orion launchpad/cc1310 launchpad/cc1350 
WITH_IP64 = 1

MODULES += examples/rpl-border-router/webserver

include $(CONTIKI)/Makefile.include
