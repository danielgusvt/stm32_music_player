#include <string.h>

#include "screens.h"
#include "images.h"
#include "fonts.h"
#include "actions.h"
#include "vars.h"
#include "styles.h"
#include "ui.h"

#include <string.h>

objects_t objects;
lv_obj_t *tick_value_change_obj;
uint32_t active_theme_index = 0;

void create_screen_main() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.main = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 320);
    {
        lv_obj_t *parent_obj = obj;
        {
            // buttonPlayPause
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.button_play_pause = obj;
            lv_obj_set_pos(obj, 368, 83);
            lv_obj_set_size(obj, 100, 50);
            lv_obj_add_event_cb(obj, action_play_pause_audio, LV_EVENT_PRESSED, (void *)0);
            {
                lv_obj_t *parent_obj = obj;
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 0, 0);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Play/Pause");
                }
            }
        }
        {
            // buttonNextSong
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.button_next_song = obj;
            lv_obj_set_pos(obj, 369, 148);
            lv_obj_set_size(obj, 100, 50);
            lv_obj_add_event_cb(obj, action_next_song, LV_EVENT_PRESSED, (void *)0);
            {
                lv_obj_t *parent_obj = obj;
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 0, 0);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Next Song");
                }
            }
        }
        {
            // labelNowP
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.label_now_p = obj;
            lv_obj_set_pos(obj, 26, 26);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "Now Playing:");
        }
        {
            // labelSong
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.label_song = obj;
            lv_obj_set_pos(obj, 168, 26);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "Idle");
        }
        {
            // buttonGoSettings
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.button_go_settings = obj;
            lv_obj_set_pos(obj, 368, 214);
            lv_obj_set_size(obj, 100, 50);
            lv_obj_add_event_cb(obj, action_goto_settings_page, LV_EVENT_PRESSED, (void *)0);
            {
                lv_obj_t *parent_obj = obj;
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 0, 0);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Settings");
                }
            }
        }
        {
            // songsTable
            lv_obj_t *obj = lv_table_create(parent_obj);
            objects.songs_table = obj;
            lv_obj_set_pos(obj, 26, 61);
            lv_obj_set_size(obj, 334, 236);
        }
    }
    
    tick_screen_main();
}

void tick_screen_main() {
}

void create_screen_settings_page() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.settings_page = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 320);
    {
        lv_obj_t *parent_obj = obj;
        {
            // sliderVolume
            lv_obj_t *obj = lv_slider_create(parent_obj);
            objects.slider_volume = obj;
            lv_obj_set_pos(obj, 171, 63);
            lv_obj_set_size(obj, 10, 195);
            lv_slider_set_range(obj, 1, 100);
            lv_slider_set_value(obj, 100, LV_ANIM_OFF);
            lv_obj_add_event_cb(obj, action_update_volume, LV_EVENT_PRESSING, (void *)0);
        }
        {
            // sliderBass
            lv_obj_t *obj = lv_slider_create(parent_obj);
            objects.slider_bass = obj;
            lv_obj_set_pos(obj, 255, 63);
            lv_obj_set_size(obj, 10, 195);
            lv_slider_set_value(obj, 50, LV_ANIM_OFF);
            lv_obj_add_event_cb(obj, action_update_bass, LV_EVENT_PRESSING, (void *)0);
        }
        {
            // sliderTreble
            lv_obj_t *obj = lv_slider_create(parent_obj);
            objects.slider_treble = obj;
            lv_obj_set_pos(obj, 399, 63);
            lv_obj_set_size(obj, 10, 195);
            lv_slider_set_value(obj, 50, LV_ANIM_OFF);
            lv_obj_add_event_cb(obj, action_update_treble, LV_EVENT_PRESSING, (void *)0);
        }
        {
            // buttonGoMain
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.button_go_main = obj;
            lv_obj_set_pos(obj, 19, 157);
            lv_obj_set_size(obj, 100, 50);
            lv_obj_add_event_cb(obj, action_goto_main_page, LV_EVENT_PRESSED, (void *)0);
            {
                lv_obj_t *parent_obj = obj;
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 0, 0);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Controls");
                }
            }
        }
        {
            lv_obj_t *obj = lv_label_create(parent_obj);
            lv_obj_set_pos(obj, 148, 30);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "Volume");
        }
        {
            // buttonResetEQ
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.button_reset_eq = obj;
            lv_obj_set_pos(obj, 19, 77);
            lv_obj_set_size(obj, 100, 50);
            lv_obj_add_event_cb(obj, action_reset_eq, LV_EVENT_PRESSED, (void *)0);
            {
                lv_obj_t *parent_obj = obj;
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 0, 0);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Reset");
                }
            }
        }
        {
            lv_obj_t *obj = lv_label_create(parent_obj);
            lv_obj_set_pos(obj, 381, 30);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "Treble");
        }
        {
            lv_obj_t *obj = lv_label_create(parent_obj);
            lv_obj_set_pos(obj, 324, 30);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "Mid");
        }
        {
            // sliderMid
            lv_obj_t *obj = lv_slider_create(parent_obj);
            objects.slider_mid = obj;
            lv_obj_set_pos(obj, 333, 63);
            lv_obj_set_size(obj, 10, 195);
            lv_slider_set_value(obj, 50, LV_ANIM_OFF);
            lv_obj_add_event_cb(obj, action_update_mid, LV_EVENT_PRESSING, (void *)0);
        }
        {
            lv_obj_t *obj = lv_label_create(parent_obj);
            lv_obj_set_pos(obj, 244, 30);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "Bass");
        }
        {
            // labelVolume
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.label_volume = obj;
            lv_obj_set_pos(obj, 165, 268);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "100");
        }
        {
            // labelGainBass
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.label_gain_bass = obj;
            lv_obj_set_pos(obj, 243, 268);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "0 dB");
        }
        {
            // labelGainMid
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.label_gain_mid = obj;
            lv_obj_set_pos(obj, 321, 268);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "0 dB");
        }
        {
            // labelGainTreble
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.label_gain_treble = obj;
            lv_obj_set_pos(obj, 387, 268);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "0 dB");
        }
    }
    
    tick_screen_settings_page();
}

void tick_screen_settings_page() {
}



typedef void (*tick_screen_func_t)();
tick_screen_func_t tick_screen_funcs[] = {
    tick_screen_main,
    tick_screen_settings_page,
};
void tick_screen(int screen_index) {
    tick_screen_funcs[screen_index]();
}
void tick_screen_by_id(enum ScreensEnum screenId) {
    tick_screen_funcs[screenId - 1]();
}

void create_screens() {
    lv_disp_t *dispp = lv_disp_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), false, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    
    create_screen_main();
    create_screen_settings_page();
}
