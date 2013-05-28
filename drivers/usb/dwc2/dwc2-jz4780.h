#ifndef __DRIVERS_USB_DWC2_JZ4780_H
#define __DRIVERS_USB_DWC2_JZ4780_H

void jz4780_set_charger_current(struct dwc2 *dwc);
void jz4780_set_vbus(struct dwc2 *dwc, int is_on);

#endif	/* __DRIVERS_USB_DWC2_JZ4780_H */
