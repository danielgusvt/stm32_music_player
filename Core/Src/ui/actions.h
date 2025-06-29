#ifndef EEZ_LVGL_UI_EVENTS_H
#define EEZ_LVGL_UI_EVENTS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void action_play_pause_audio(lv_event_t * e);
extern void action_next_song(lv_event_t * e);
extern void action_update_volume(lv_event_t * e);
extern void action_goto_main_page(lv_event_t * e);
extern void action_goto_settings_page(lv_event_t * e);
extern void action_update_treble(lv_event_t * e);
extern void action_update_mid(lv_event_t * e);
extern void action_update_bass(lv_event_t * e);
extern void action_reset_eq(lv_event_t * e);


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_EVENTS_H*/