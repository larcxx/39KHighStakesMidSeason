#include "main.h"
#include "liblvgl/lvgl.h"

lv_obj_t *redLeftButton;
lv_obj_t *redLeftElimsButton;
lv_obj_t *redRightButton;
lv_obj_t *redRightElimsButton;
lv_obj_t *blueLeftButton;
lv_obj_t *blueLeftElimsButton;
lv_obj_t *blueRightButton;
lv_obj_t *blueRightElimsButton;
lv_obj_t *AWPButton;
lv_obj_t *skillsButton;
const char *a = "hello";
lv_obj_t *rl;
lv_obj_t *rel;
lv_obj_t *rr;
lv_obj_t *rer;
lv_obj_t *bl;
lv_obj_t *bel;
lv_obj_t *br;
lv_obj_t *ber;
lv_obj_t *awp;
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
    } else if (id == 5) {
        autonFunc = 5;
        lv_label_set_text(autonCurrent, "Current Auton: AWP");
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

    redLeftButton = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(redLeftButton, 0);
    lv_obj_add_event_cb(redLeftButton, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(redLeftButton, &myButtonStyleREL, 0);
    lv_obj_add_style(redLeftButton, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(redLeftButton, 150, 50);
    lv_obj_align(redLeftButton, LV_ALIGN_TOP_LEFT, 10, 25);
    rl = lv_label_create(redLeftButton);
    lv_label_set_text(rl, "Red Left");

    redRightButton = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(redRightButton, reinterpret_cast<lv_obj_user_data_t>(1));
    lv_obj_add_event_cb(redRightButton, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(redRightButton, &myButtonStyleREL, 0);
    lv_obj_add_style(redRightButton, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(redRightButton, 150, 50);
    lv_obj_align(redRightButton, LV_ALIGN_TOP_RIGHT, -10, 25);
    rr = lv_label_create(redRightButton);
    lv_label_set_text(rr, "Red Right");

    blueLeftButton = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(blueLeftButton, reinterpret_cast<lv_obj_user_data_t>(2));
    lv_obj_add_event_cb(blueLeftButton, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(blueLeftButton, &myButtonStyleREL, 0);
    lv_obj_add_style(blueLeftButton, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(blueLeftButton, 150, 50);
    lv_obj_align(blueLeftButton, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    bl = lv_label_create(blueLeftButton);
    lv_label_set_text(bl, "Blue Left");

    blueRightButton = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(blueRightButton, reinterpret_cast<lv_obj_user_data_t>(3));
    lv_obj_add_event_cb(blueRightButton, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(blueRightButton, &myButtonStyleREL, 0);
    lv_obj_add_style(blueRightButton, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(blueRightButton, 150, 50);
    lv_obj_align(blueRightButton, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    br = lv_label_create(blueRightButton);
    lv_label_set_text(br, "Blue Right");

    skillsButton = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(skillsButton, reinterpret_cast<lv_obj_user_data_t>(4));
    lv_obj_add_event_cb(skillsButton, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(skillsButton, &myButtonStyleREL, 0);
    lv_obj_add_style(skillsButton, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(skillsButton, 150, 50);
    lv_obj_align(skillsButton, LV_ALIGN_BOTTOM_MID, 0, -10);
    sk = lv_label_create(skillsButton);
    lv_label_set_text(sk, "Skills");

    AWPButton = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(AWPButton, reinterpret_cast<lv_obj_user_data_t>(5));
    lv_obj_add_event_cb(AWPButton, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(AWPButton, &myButtonStyleREL, 0);
    lv_obj_add_style(AWPButton, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(AWPButton, 150, 50);
    lv_obj_align(AWPButton, LV_ALIGN_TOP_MID, 0, 25);
    awp = lv_label_create(AWPButton);
    lv_label_set_text(awp, "AWP");
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