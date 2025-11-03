
#ifndef WIFI_H
#define WIFI_H

#include <stdbool.h>

void wifi_init(const char* ssid, const char* password);
bool wifi_is_connected(void);

#endif // WIFI_H


