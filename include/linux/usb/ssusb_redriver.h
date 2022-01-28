#ifndef SSUSB_REDRIVER_H
#define SSUSB_REDRIVER_H
#include <linux/of.h>
enum ssusb_redriver_function {
	USBC_ORIENTATION_CC1=1,
	USBC_ORIENTATION_CC2,
	USBC_DISPLAYPORT_DISCONNECTED,
	EVENT_MAX,
};

int ssusb_redriver_event(struct device_node *node,
                         enum ssusb_redriver_function event);

#endif
