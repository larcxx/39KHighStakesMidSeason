#include "main.h"
#include "liblvgl/lvgl.h"

lv_obj_t *redLeftButton;
lv_obj_t *redLeftElimsButton;

lv_obj_t *redRightSafeButton;
lv_obj_t *redRightButton;
lv_obj_t *redRightElimsButton;

lv_obj_t *blueLeftSafeButton;
lv_obj_t *blueLeftButton;
lv_obj_t *blueLeftElimsButton;

lv_obj_t *blueRightButton;
lv_obj_t *blueRightElimsButton;

lv_obj_t *RedAWPButton;
lv_obj_t *BlueAWPButton;
lv_obj_t *skillsButton;

const char *a = "hello";
lv_obj_t *rl;
lv_obj_t *rel;

lv_obj_t *rsr;
lv_obj_t *rr;
lv_obj_t *rer;

lv_obj_t *bsl;
lv_obj_t *bl;
lv_obj_t *bel;

lv_obj_t *br;
lv_obj_t *ber;

lv_obj_t *rawp;
lv_obj_t *bawp;
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
        lv_label_set_text(autonCurrent, "Current Auton: Red Quals Left");
    } else if (id == 1) {
        autonFunc = 1;
        lv_label_set_text(autonCurrent, "Current Auton: Red Elims Left");   
    } else if (id == 2) {
        autonFunc = 2;
        lv_label_set_text(autonCurrent, "Current Auton: Red Safe Right");
    } else if (id == 3) {
        autonFunc = 3;
        lv_label_set_text(autonCurrent, "Current Auton: Red Quals Right");
    } else if (id == 4) {
        autonFunc = 4;
        lv_label_set_text(autonCurrent, "Current Auton: Red Elims Right");
    } else if (id == 5) {
        autonFunc = 5;
        lv_label_set_text(autonCurrent, "Current Auton: Blue Safe Left");
    } else if (id == 6) {
        autonFunc = 6;
        lv_label_set_text(autonCurrent, "Current Auton: Blue Quals Left");
    } else if (id == 7) {
        autonFunc = 7;
        lv_label_set_text(autonCurrent, "Current Auton: Blue Elims Left");
    } else if (id == 8) {
        autonFunc = 8;
        lv_label_set_text(autonCurrent, "Current Auton: Blue Quals Right");
    } else if (id == 9) {
        autonFunc = 9;
        lv_label_set_text(autonCurrent, "Current Auton: Blue Elims Right");
    } else if (id == 10) {
        autonFunc = 10;
        lv_label_set_text(autonCurrent, "Current Auton: Skills");
    } else if (id == 11) {
        autonFunc = 11;
        lv_label_set_text(autonCurrent, "Current Auton: Red AWP");
    } else if (id == 12) {
        autonFunc = 12;
        lv_label_set_text(autonCurrent, "Current Auton: Blue AWP");
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


    //---------------------RED LEFT RING SIDE--------------------//


    redLeftButton = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(redLeftButton, 0);
    lv_obj_add_event_cb(redLeftButton, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(redLeftButton, &myButtonStyleREL, 0);
    lv_obj_add_style(redLeftButton, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(redLeftButton, 80, 50);
    lv_obj_align(redLeftButton, LV_ALIGN_TOP_LEFT, 0, 0);
    rl = lv_label_create(redLeftButton);
    lv_label_set_text(rl, "Qual RL");

    redLeftElimsButton = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(redLeftElimsButton, reinterpret_cast<lv_obj_user_data_t>(1));
    lv_obj_add_event_cb(redLeftElimsButton, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(redLeftElimsButton, &myButtonStyleREL, 0);
    lv_obj_add_style(redLeftElimsButton, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(redLeftElimsButton, 80, 50);
    lv_obj_align(redLeftElimsButton, LV_ALIGN_TOP_LEFT, 90, 0);
    rel = lv_label_create(redLeftElimsButton);
    lv_label_set_text(rel, "Elims RL");


    //--------------------RED RIGHT GOAL SIDE----------------------//

    redRightSafeButton = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(redRightSafeButton, reinterpret_cast<lv_obj_user_data_t>(2));
    lv_obj_add_event_cb(redRightSafeButton, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(redRightSafeButton, &myButtonStyleREL, 0);
    lv_obj_add_style(redRightSafeButton, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(redRightSafeButton, 80, 50);
    lv_obj_align(redRightSafeButton, LV_ALIGN_TOP_LEFT, 180, 0);
    rsr = lv_label_create(redRightSafeButton);
    lv_label_set_text(rsr, "Safe RR");

    redRightButton = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(redRightButton, reinterpret_cast<lv_obj_user_data_t>(3));
    lv_obj_add_event_cb(redRightButton, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(redRightButton, &myButtonStyleREL, 0);
    lv_obj_add_style(redRightButton, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(redRightButton, 80, 50);
    lv_obj_align(redRightButton, LV_ALIGN_TOP_LEFT, 270, 0);
    rr = lv_label_create(redRightButton);
    lv_label_set_text(rr, "Qual RR");

    redRightElimsButton = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(redRightElimsButton, reinterpret_cast<lv_obj_user_data_t>(4));
    lv_obj_add_event_cb(redRightElimsButton, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(redRightElimsButton, &myButtonStyleREL, 0);
    lv_obj_add_style(redRightElimsButton, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(redRightElimsButton, 80, 50);
    lv_obj_align(redRightElimsButton, LV_ALIGN_TOP_LEFT, 360, 0);
    rer = lv_label_create(redRightElimsButton);
    lv_label_set_text(rer, "Elims RR");

    
    //--------------------------BLUE LEFT GOAL SIDE-----------------------//

    blueLeftSafeButton = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(blueLeftSafeButton, reinterpret_cast<lv_obj_user_data_t>(5));
    lv_obj_add_event_cb(blueLeftSafeButton, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(blueLeftSafeButton, &myButtonStyleREL, 0);
    lv_obj_add_style(blueLeftSafeButton, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(blueLeftSafeButton, 80, 50);
    lv_obj_align(blueLeftSafeButton, LV_ALIGN_BOTTOM_LEFT, 180, 0);
    bsl = lv_label_create(blueLeftSafeButton);
    lv_label_set_text(bsl, "Safe BL");

    blueLeftButton = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(blueLeftButton, reinterpret_cast<lv_obj_user_data_t>(6));
    lv_obj_add_event_cb(blueLeftButton, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(blueLeftButton, &myButtonStyleREL, 0);
    lv_obj_add_style(blueLeftButton, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(blueLeftButton, 80, 50);
    lv_obj_align(blueLeftButton, LV_ALIGN_BOTTOM_LEFT, 270, 0);
    bl = lv_label_create(blueLeftButton);
    lv_label_set_text(bl, "Qual BL");

    blueLeftElimsButton = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(blueLeftElimsButton, reinterpret_cast<lv_obj_user_data_t>(7));
    lv_obj_add_event_cb(blueLeftElimsButton, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(blueLeftElimsButton, &myButtonStyleREL, 0);
    lv_obj_add_style(blueLeftElimsButton, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(blueLeftElimsButton, 80, 50);
    lv_obj_align(blueLeftElimsButton, LV_ALIGN_BOTTOM_LEFT, 360, 0);
    bel = lv_label_create(blueLeftElimsButton);
    lv_label_set_text(bel, "Elims BL");

    //-------------------------BLUE RIGHT RING SIDE---------------------------//

    blueRightButton = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(blueRightButton, reinterpret_cast<lv_obj_user_data_t>(8));
    lv_obj_add_event_cb(blueRightButton, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(blueRightButton, &myButtonStyleREL, 0);
    lv_obj_add_style(blueRightButton, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(blueRightButton, 80, 50);
    lv_obj_align(blueRightButton, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    br = lv_label_create(blueRightButton);
    lv_label_set_text(br, "Qual BR");

    blueRightElimsButton = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(blueRightElimsButton, reinterpret_cast<lv_obj_user_data_t>(9));
    lv_obj_add_event_cb(blueRightElimsButton, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(blueRightElimsButton, &myButtonStyleREL, 0);
    lv_obj_add_style(blueRightElimsButton, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(blueRightElimsButton, 80, 50);
    lv_obj_align(blueRightElimsButton, LV_ALIGN_BOTTOM_LEFT, 90, 0);
    ber = lv_label_create(blueRightElimsButton);
    lv_label_set_text(ber, "Elims BR");

    //------------------------SKILLS AND AWPs--------------------------------//

    skillsButton = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(skillsButton, reinterpret_cast<lv_obj_user_data_t>(10));
    lv_obj_add_event_cb(skillsButton, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(skillsButton, &myButtonStyleREL, 0);
    lv_obj_add_style(skillsButton, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(skillsButton, 80, 40);
    lv_obj_align(skillsButton, LV_ALIGN_CENTER, 160, 0);
    sk = lv_label_create(skillsButton);
    lv_label_set_text(sk, "Skills");

    RedAWPButton = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(RedAWPButton, reinterpret_cast<lv_obj_user_data_t>(11));
    lv_obj_add_event_cb(RedAWPButton, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(RedAWPButton, &myButtonStyleREL, 0);
    lv_obj_add_style(RedAWPButton, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(RedAWPButton, 80, 40);
    lv_obj_align(RedAWPButton, LV_ALIGN_CENTER, 160, 45);
    rawp = lv_label_create(RedAWPButton);
    lv_label_set_text(rawp, "RAWP");

    BlueAWPButton = lv_btn_create(lv_scr_act());
    lv_obj_set_user_data(BlueAWPButton, reinterpret_cast<lv_obj_user_data_t>(12));
    lv_obj_add_event_cb(BlueAWPButton, btn_click_action, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(BlueAWPButton, &myButtonStyleREL, 0);
    lv_obj_add_style(BlueAWPButton, &myButtonStylePR, LV_STATE_PRESSED);
    lv_obj_set_size(BlueAWPButton, 80, 40);
    lv_obj_align(BlueAWPButton, LV_ALIGN_CENTER, 160, -45);
    bawp = lv_label_create(BlueAWPButton);
    lv_label_set_text(bawp, "BAWP");








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