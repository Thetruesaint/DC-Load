#ifndef APP_OTA_H
#define APP_OTA_H

void app_ota_begin();
void app_ota_handle();
void app_ota_stop();
void app_ota_restart();
bool app_ota_is_active();
bool app_ota_is_uploading();
bool app_ota_has_error();
const char *app_ota_status_text();
const char *app_ota_detail_text();
const char *app_ota_hint_text();

#endif
