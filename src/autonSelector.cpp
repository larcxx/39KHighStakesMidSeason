#include "main.h"
#include "liblvgl/lvgl.h"

lv_obj_t *redLefts;
lv_obj_t *redRights;
lv_obj_t *blueLefts;
lv_obj_t *blueRights;
lv_obj_t *skillss;
const char *a = "hello";
lv_obj_t *rl;
lv_obj_t *rr;
lv_obj_t *bl;
lv_obj_t *br;
lv_obj_t *sk;

lv_style_t myButtonStyleREL; // Released style
lv_style_t myButtonStylePR;  // Pressed style

lv_obj_t *sdCard;
lv_obj_t *autonCurrent;
int autonFunc = 6; // Number for auton selected

void start() {
    autonCurrent = lv_label_create(lv_scr_act());
    lv_obj_align(autonCurrent, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(autonCurrent, "Current Auton: None");
}

void btn_click_action(lv_event_t *e) {
    lv_obj_t *btn = lv_event_get_target(e);
    uint8_t id = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(lv_obj_get_user_data(btn))); // ID is useful for multiple buttons

    lv_label_set_text(autonCurrent, "Current Auton: None");
    if (id == 0) {
        autonFunc = 0;
        lv_label_set_text(autonCurrent, "Current Auton: Red Left");
    } else if (id == 1) {
        autonFunc = 1;
        lv_label_set_text(autonCurrent, "Current Auton: Red Right");
    } else if (id == 2) {
        autonFunc = 2;
        lv_label_set_text(autonCurrent, "Current Auton: Blue Left");
    } else if (id == 3) {
        autonFunc = 3;
        lv_label_set_text(autonCurrent, "Current Auton: Blue Right");
    } else if (id == 4) {
        autonFunc = 4;
        lv_label_set_text(autonCurrent, "Current Auton: Skills");
    }
}

void selection() {
    lv_style_init(&myButtonStyleREL);
    lv_style_set_bg_color(&myButtonStyleREL, lv_color_make(255, 81, 47));
    lv_style_set_bg_grad_color(&myButtonStyleREL, lv_color_make(221, 36, 118));
    lv_style_set_radius(&myButtonStyleREL, 0);
    lv_style_set_text_color(&myButtonStyleREL, lv_color_white());

    lv_style_init(&myButtonStylePR);
    lv_style_set_bg_color(&myButtonStylePR, lv_color_make(255, 0, 0));
    lv_style_set_bg_grad_color(&myButtonStylePR, lv_color_make(0, 0, 255));
    lv_style_set_radius(&myButtonStylePR, 0);
    lv_style_set_text_color(&myButtonStylePR, lv_color_white());

    redLefts = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(redLefts, 0);
    lv_obj_add_event_cb(redLefts, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(redLefts, &myButtonStyleREL, 0);
    lv_obj_add_style(redLefts, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(redLefts, 200, 50);
    lv_obj_align(redLefts, LV_ALIGN_TOP_LEFT, 10, 25);
    rl = lv_label_create(redLefts);
    lv_label_set_text(rl, "Red Left");

    redRights = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(redRights, reinterpret_cast<lv_obj_user_data_t>(1));
    lv_obj_add_event_cb(redRights, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(redRights, &myButtonStyleREL, 0);
    lv_obj_add_style(redRights, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(redRights, 200, 50);
    lv_obj_align(redRights, LV_ALIGN_TOP_RIGHT, -10, 25);
    rr = lv_label_create(redRights);
    lv_label_set_text(rr, "Red Right");

    blueLefts = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(blueLefts, reinterpret_cast<lv_obj_user_data_t>(2));
    lv_obj_add_event_cb(blueLefts, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(blueLefts, &myButtonStyleREL, 0);
    lv_obj_add_style(blueLefts, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(blueLefts, 150, 50);
    lv_obj_align(blueLefts, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    bl = lv_label_create(blueLefts);
    lv_label_set_text(bl, "Blue Left");

    blueRights = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(blueRights, reinterpret_cast<lv_obj_user_data_t>(3));
    lv_obj_add_event_cb(blueRights, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(blueRights, &myButtonStyleREL, 0);
    lv_obj_add_style(blueRights, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(blueRights, 150, 50);
    lv_obj_align(blueRights, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    br = lv_label_create(blueRights);
    lv_label_set_text(br, "Blue Right");

    skillss = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(skillss, reinterpret_cast<lv_obj_user_data_t>(4));
    lv_obj_add_event_cb(skillss, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(skillss, &myButtonStyleREL, 0);
    lv_obj_add_style(skillss, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(skillss, 150, 50);
    lv_obj_align(skillss, LV_ALIGN_BOTTOM_MID, 0, -10);
    sk = lv_label_create(skillss);
    lv_label_set_text(sk, "Skills");
}

void updateSdStatus() {
    sdCard = lv_label_create(lv_scr_act());
    if (pros::usd::is_installed()) {
        lv_label_set_text(sdCard, "SD Card In");
        lv_obj_align(sdCard, LV_ALIGN_LEFT_MID, 14, -107);
    } else {
        lv_label_set_text(sdCard, "SD Card Out");
        lv_obj_align(sdCard, LV_ALIGN_LEFT_MID, 14, -107);
    }
}