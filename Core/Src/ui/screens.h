#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _objects_t {
    lv_obj_t *main;
    lv_obj_t *settings_page;
    lv_obj_t *button_play_pause;
    lv_obj_t *button_next_song;
    lv_obj_t *label_now_p;
    lv_obj_t *label_song;
    lv_obj_t *button_go_settings;
    lv_obj_t *songs_table;
    lv_obj_t *slider_volume;
    lv_obj_t *slider_bass;
    lv_obj_t *slider_treble;
    lv_obj_t *button_go_main;
    lv_obj_t *button_reset_eq;
    lv_obj_t *slider_mid;
    lv_obj_t *label_volume;
    lv_obj_t *label_gain_bass;
    lv_obj_t *label_gain_mid;
    lv_obj_t *label_gain_treble;
} objects_t;

extern objects_t objects;

enum ScreensEnum {
    SCREEN_ID_MAIN = 1,
    SCREEN_ID_SETTINGS_PAGE = 2,
};

void create_screen_main();
void tick_screen_main();

void create_screen_settings_page();
void tick_screen_settings_page();

void tick_screen_by_id(enum ScreensEnum screenId);
void tick_screen(int screen_index);

void create_screens();


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/