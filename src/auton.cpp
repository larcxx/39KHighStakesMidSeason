#include "main.h"
#include <vector>


void skills(){
    chassis.setPose(62.5,0,270);
    setIntakeState(1);
    pros::delay(700);

    chassis.moveToPose(48,0,270,1000,{.minSpeed = 30});
    base.tankTo(90, true);

    setIntakeState(2);
    chassis.moveToPose(48,24,180,2000,{.forwards=false, .lead = 0, .maxSpeed = 50},true);
    chassis.waitUntil(15);
    setMechState(0);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    setIntakeState(1);
    base.tankTo(0);

    chassis.moveToPose(24,25.5,270,1500,{.minSpeed = 60});
    base.tankTo(-55);

    chassis.moveToPose(-52,60,290,3000,{.horizontalDrift = 7, .lead = 0.65, .maxSpeed = 90},true);
    chassis.waitUntil(80);
    setMechState(1);
    chassis.waitUntilDone();

    chassis.moveToPose(-4,39,315,2000,{.forwards=false, .maxSpeed = 70});
    base.tankTo(-90);
    chassis.moveToPose(-3,65,0,1500,{.lead = 0, .minSpeed = 70},false);
    pros::delay(150);
    setIntakeState(4);
    setMechState(2);

    pros::delay(400);


    chassis.moveToPose(-4,50,0,2000,{.forwards=false, .lead = 0},false);
    setMechState(1);
    setIntakeState(1);

    chassis.moveToPose(-4,65,0,1500,{.lead = 0, .minSpeed = 70},false);
    pros::delay(850);
    setIntakeState(4);
    setMechState(2);
    pros::delay(600);


    chassis.moveToPose(-4,48,0,1500,{.forwards=false, .lead = 0, .minSpeed = 20},false);

    setMechState(0);

    setIntakeState(1);
    base.tankTo(180);

    chassis.moveToPose(61,52,90,3000,{.maxSpeed = 60, .minSpeed=30, .earlyExitRange=3});
    

    chassis.moveToPose(57,52,90,1000,{.forwards=false, .minSpeed=10, .earlyExitRange=1},false);
    setIntakeState(0);
    base.tankTo(-45);
    setIntakeState(1);
    chassis.moveToPose(48,61,315,1000,{});
    base.tankTo(20);
    goalClamp.set_value(0);
    chassis.moveToPose(63,65,260,1500,{.forwards=false, .minSpeed=20});

    

    chassis.moveToPose(48,0,180,2500,{},true);
    base.tankTo(-90);
    chassis.moveToPose(48, -24, 0, 1200, {.forwards = false, .maxSpeed = 50}, true);
    chassis.waitUntil(24);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    

    base.tankTo(0);
    chassis.moveToPose(24,-24,270,1500,{.minSpeed = 1});
    base.tankTo(45);


    chassis.moveToPose(-52,-61,250,3000,{.horizontalDrift = 7, .lead = 0.4, .maxSpeed = 70},true);
    chassis.waitUntil(80);
    setMechState(1);
    chassis.waitUntilDone();

    
    chassis.moveToPose(-3,-39,235,2000,{.forwards=false, .maxSpeed = 80});
    base.tankTo(90);
    chassis.moveToPose(-4,-64,180,1500,{.lead = 0, .minSpeed = 70},false);
    //base.tankTo(90);

    pros::delay(500);
    setIntakeState(4);
    setMechState(2);
    pros::delay(600);

    chassis.moveToPose(-4,-50,180,2000,{.forwards=false, .lead = 0},false);
    setMechState(1);
    setIntakeState(1);
    chassis.moveToPose(-4,-64,180,1500,{.lead = 0, .minSpeed = 70},false);


    //base.tankTo(90);
    pros::delay(500);
    setIntakeState(4);
    setMechState(2);
    pros::delay(600);

    

    setIntakeState(1);
    chassis.moveToPose(-4,-48,180,1500,{.forwards=false, .lead = 0},false);

    setMechState(0);
    base.tankTo(180);
    chassis.moveToPose(61,-52,90,3000, {.maxSpeed=70, .minSpeed = 30});
    base.tankTo(45);
    chassis.moveToPose(48,-64,225,1500,{.earlyExitRange = 1.5});
    base.tankTo(-15);
    pros::delay(200);
    goalClamp.set_value(0);
    chassis.moveToPose(60,-65, 240,1500,{.forwards=false, .minSpeed = 30, .earlyExitRange = 3});

    setMechState(1);

    chassis.moveToPose(-24,-20,315,3000,{.maxSpeed = 80, .minSpeed = 55});
    chassis.moveToPose(-70,-75,190,2000,{.lead = 0.7, .minSpeed = 30});

    chassis.moveToPose(-48,3,200,2000,{.forwards = false, .maxSpeed = 60, .minSpeed = 20}, true);
    chassis.waitUntil(51);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    base.tankTo(0);
    chassis.moveToPose(-70,0,270,1500,{.minSpeed = 70}, false);
    ls.move_voltage(0);
    rs.move_voltage(0);
    pros::delay(75);

    ls.move_voltage(-2500);
    rs.move_voltage(-2500);
    pros::delay(500);
    ls.move_voltage(0);
    rs.move_voltage(0);
    

    setIntakeState(4);
    setMechState(3);

    pros::delay(500);
    
    ls.move_voltage(-6000);
    rs.move_voltage(-6000);
    pros::delay(300);
    ls.move_voltage(0);
    rs.move_voltage(0);


    setIntakeState(1);
    setMechState(0);

    base.tankTo(90);
    chassis.moveToPose(-48,-50,180,1500, {.minSpeed = 50});
    chassis.moveToPose(0,-2,45,3000,{.maxSpeed = 70, .minSpeed = 30}, true);
    chassis.waitUntil(45);
    setIntakeState(3);
    chassis.waitUntilDone();
    
    base.tankTo(315);
    chassis.moveToPose(-46,46,315,2000,{.minSpeed = 50}, true);
    chassis.waitUntil(35);
    setIntakeState(1);

    chassis.moveToPose(-50,48,270,1500,{.maxSpeed = 70, .minSpeed = 20},false);
    

    base.tankTo(110);


    /*
    
    chassis.moveToPose(-70,-66,160,500, {.forwards = false}, true);
    chassis.waitUntil(10);
    goalClamp.set_value(0);
    chassis.waitUntilDone();


    chassis.moveToPose(-17,17,315,3000,{.forwards = false},true);
    chassis.waitUntil(15);
    setMechState(3);
    chassis.waitUntilDone();
    */

    
    /*
    base.tankTo(-70,true);
    doinker.set_value(1);
    chassis.moveToPose(-68,60,0,1000,{.minSpeed=30},false);
    ls.move_voltage(12000);
    rs.move_voltage(-12000);
    pros::delay(440);
    base.tankTo(120,true);
    goalClamp.set_value(0);
    chassis.moveToPose(-70,-70,160,500,{.forwards=false, .minSpeed = 30});




    

    












    /*
    chassis.setPose(0,-61.5,0);
    setIntakeState(1);
    pros::delay(800);
    ls.move_voltage(8000);
    rs.move_voltage(8000);
    pros::delay(80);
    ls.move_voltage(0);
    rs.move_voltage(0);    
    base.tankTo(250);
    goalClamp.set_value(0);
    chassis.moveToPose(-23,-45,0,3000,{.forwards = false, .lead = 0, .maxSpeed = 80, .minSpeed = 30}, true);
    chassis.waitUntil(22);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    chassis.moveToPose(-24,-28,315,1000,{.lead = 0, .minSpeed = 70, .earlyExitRange = 7});
    //chassis.moveToPose(-50,9,320,2000,{.lead = 0.8, .minSpeed = 100, .earlyExitRange = 5}, false);
    //base.tankTo(0);
    chassis.moveToPose(-48,43,0,5000,{.horizontalDrift = 2},true);
    chassis.waitUntil(73);
    setMechState(1);
    chassis.waitUntilDone();

    chassis.moveToPose(-40, -5, 345,3000,{.forwards = false, .minSpeed = 40}, false);
    base.setTankConstants(0.72, 0.008, 7.2, 7, 12, 0.5, 1300, 300);
    base.tankTo(90);
    setMechState(3);
    setIntakeState(2);
    pros::delay(200);
    setIntakeState(1);
    chassis.moveToPose(-63,-5,270,2000,{.lead = 0,.maxSpeed = 50},true);
    base.setTankConstants(0.72, 0.008, 7.2, 7, 12, 3, 1300, 0);
    chassis.waitUntil(16);
    setIntakeState(0);
    chassis.waitUntilDone();
    setMechState(4);
    pros::delay(500);
    setMechState(1);
    pros::delay(700);
    setIntakeState(1);
    pros::delay(1500);
    setMechState(4);
    setIntakeState(2);

    pros::delay(1000);
    setMechState(0);
    setIntakeState(1);
    pros::delay(1000);


    
    ls.move_voltage(-11000);
    rs.move_voltage(-7000);
    pros::delay(200);
    ls.move_voltage(0);
    rs.move_voltage(0);

    base.tankTo(180);




    
    //chassis.moveToPose(-40,-3,270,1000,{.forwards = false, .minSpeed = 30, .earlyExitRange = 10},true);
    //base.tankTo(180);
    chassis.moveToPose(-40,-60,180,2000,{.minSpeed = 70});
    base.tankTo(18);
    chassis.moveToPose(-60,-50,0,2000,{.minSpeed = 40,.earlyExitRange = 5});
    pros::delay(150);
    chassis.moveToPose(-63,-63,5,3000,{.forwards = false, .minSpeed = 35, .earlyExitRange = 5}, true);

    
    chassis.waitUntil(6);
    goalClamp.set_value(0);
    chassis.waitUntilDone();
    base.tankTo(307);
    setIntakeState(3);
    chassis.moveToPose(0,-2,45,3000,{},false);
    base.tankTo(225);
    chassis.moveToPose(24,-28,225,1500,{.minSpeed = 50, .earlyExitRange = 5}, false);
    setIntakeState(0);
    base.tankTo(0);
    chassis.moveToPose(22,-48,0,2000,{.forwards = false, .minSpeed = 30}, true);
    chassis.waitUntil(24);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);
    base.tankTo(70);
    chassis.moveToPose(65,-65,280,2000,{.forwards = false, .minSpeed = 70}, true);
    chassis.waitUntil(30);
    goalClamp.set_value(0);
    chassis.waitUntilDone();
    base.tankTo(45);
    
    chassis.moveToPose(24,-24,315,3000,{.minSpeed = 45}, false);
    setIntakeState(0);
    base.tankTo(225);
    setMechState(4);
    pros::delay(1000);
    chassis.moveToPose(10,-10,45,3000,{.forwards = false, .minSpeed = 90}, false);
    */
}


void blueLeft(){

    chassis.setPose(49,-36,270);
    //setIntakeState(1);
    chassis.moveToPose(15,-38.5,260,2000,{.lead=0.23,.minSpeed=200},true);
    chassis.waitUntil(90);
    doinker.set_value(1);
    chassis.waitUntilDone();


    //pros::delay(0);
    //setIntakeState(0);
    //base.tankTo(305,false);
    //base.tankTo(340,true);
    chassis.moveToPose(24,-32,280,2000,{.forwards = false,.lead=0.3, .minSpeed = 80},true);
    chassis.waitUntil(40);
    chassis.waitUntilDone();

    doinker.set_value(0);
    pros::delay(250);

    base.tankTo(-170);

    chassis.moveToPose(9,-43,85,2000,{.forwards = false, .lead = 0.2, .maxSpeed = 50},true);
    chassis.waitUntil(13);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);
    pros::delay(200);


    chassis.moveToPose(46,-24,0,2000,{.lead = 0},true);
    chassis.waitUntil(3);
    goalClamp.set_value(0);
    chassis.waitUntilDone();
    base.tankTo(180);

    chassis.moveToPose(10,-22,90,2000,{.forwards = false, .maxSpeed = 50}, true);
    chassis.waitUntil(30);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    base.tankTo(60);

    chassis.moveToPose(0,-65,210,1500,{},false);

    chassis.moveToPose(13,-24,210,1000,{.forwards = false});

    base.tankTo(-45);
    setMechState(5);
    
    /*
    chassis.moveToPose(55,-51,172,3000,{.horizontalDrift=7});
    chassis.waitUntil(60);
    doinker.set_value(1);
    chassis.waitUntilDone();
    rs.move_voltage(-12000);
    ls.move_voltage(12000);
    pros::delay(500);
    base.tankTo(30);
    doinker.set_value(0);
    chassis.moveToPose(12,-12,280,1500,{});

    


    /*
    chassis.moveToPose(22,-70,180,1500,{});
    base.tankTo(-10);
    pros::delay(100);
    goalClamp.set_value(0);

    chassis.moveToPose(18.5,-32,180,2000,{.forwards = false, .maxSpeed = 70});
    chassis.waitUntil(35);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    base.tankTo(180);

    

    chassis.moveToPose(61,-75,180,3000,{.lead = 0.5, .minSpeed = 90});
    chassis.waitUntil(80);
    setMechState(3);
    chassis.waitUntilDone();

    chassis.moveToPose(57,-69,170,1000,{.forwards = false});
    chassis.waitUntil(5);
    setMechState(2);
    doinker.set_value(1);
    chassis.waitUntilDone();
    base.tankTo(-60,true);
    doinker.set_value(0);
    goalClamp.set_value(0);
    setMechState(2);
    ls.move_voltage(-10000);
    rs.move_voltage(-10000);
    pros::delay(200);
    chassis.moveToPose(15,-28,315,2000,{});
    chassis.waitUntil(50);
    setMechState(3);
    chassis.waitUntilDone();
    */






}

void blueLeftElims(){

    chassis.setPose(54.75,-60+4.5+5.75,270);
    //setIntakeState(1);
    chassis.moveToPose(21.5,-52,263,2000,{.lead=0.1,.minSpeed=127},true);
    chassis.waitUntil(90);
    doinker.set_value(1);
    chassis.waitUntilDone();


    //pros::delay(0);
    //setIntakeState(0);
    //base.tankTo(305,false);
    //base.tankTo(340,true);
    chassis.moveToPose(40,-45,270,2000,{.forwards = false,.lead=0.3, .minSpeed = 80},true);
    chassis.waitUntil(40);
    chassis.waitUntilDone();

    doinker.set_value(0);
    pros::delay(250);

    base.tankTo(-170);

    chassis.moveToPose(12,-57,53,2000,{.forwards = false, .lead = 0.4, .minSpeed = 50},true);
    chassis.waitUntil(26);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    pros::delay(200);

    //base.tankTo(90);
    chassis.moveToPose(22,-70,180,1500,{});
    base.tankTo(-10);
    pros::delay(100);
    goalClamp.set_value(0);

    chassis.moveToPose(18.5,-32,180,2000,{.forwards = false, .maxSpeed = 70});
    chassis.waitUntil(35);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    base.tankTo(180);

    chassis.moveToPose(61,-75,180,3000,{.lead = 0.5, .minSpeed = 90});
    chassis.waitUntilDone();
    doinker.set_value(1);
    pros::delay(700);
    setIntakeState(0);
    base.tankTo(-60,true);
    doinker.set_value(0);
    chassis.moveToPose(12,-50,270,2000,{});







}

void blueRight(){
    chassis.setPose(-58.33,-13.23,323);
    
    setMechState(3);
    pros::delay(500);
    ls.move_voltage(-6000);
    rs.move_voltage(-6000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    chassis.moveToPose(-19,-24,280,1500,{.forwards=false, .maxSpeed = 80},true);
    chassis.waitUntil(22);
    setMechState(0);
    chassis.waitUntil(41);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    base.tankTo(-172);
    chassis.moveToPose(-6,-56,180,2000,{.maxSpeed = 60, .earlyExitRange = 3},false);
    chassis.moveToPose(-24,-24,135,1500,{.forwards=false, .lead = 0.6, .earlyExitRange = 3},true);
    base.tankTo(143);
    //chassis.moveToPose(-24,50,0,1500,{});
    chassis.moveToPose(-24,-50,180,2000,{.minSpeed = 50,.earlyExitRange=3}, false);
    chassis.moveToPose(-24,-20,180,2000,{.forwards = false, .minSpeed = 50, .earlyExitRange=3}, false);
    setMechState(4);
    //base.tankTo(-53);
    chassis.moveToPose(-68,-72,225,2000,{.lead = 0.6, .minSpeed = 100},false);
    pros::delay(200);

    setMechState(0);
    chassis.moveToPose(-48,-24,180,2000,{.forwards=false, .minSpeed = 70 });
    base.tankTo(-37,true);
    intakeLift.set_value(1);
    chassis.moveToPose(-48,-3,0,1500,{.minSpeed = 30, .earlyExitRange = 2});
    chassis.moveToPose(-48,-15,0,1500,{.forwards = false, .minSpeed = 30, .earlyExitRange = 2});


    chassis.moveToPose(-20,-5,70,1000,{}, false);
    intakeLift.set_value(0);

    setMechState(5);

    /*
    chassis.setPose(57,12,90);
    setMechState(1);
    setIntakeState(1);
    pros::delay(300);
    base.tankTo(-45);

    pros::delay(100);
    setIntakeState(2);
    setMechState(3);
    pros::delay(1300);
    ls.move_voltage(-3000);
    rs.move_voltage(-3000);
    pros::delay(250);
    ls.move_voltage(0);
    rs.move_voltage(0);
    chassis.moveToPose(20,24,105,2000,{.forwards=false, .maxSpeed = 80},true);
    chassis.waitUntil(20);
    setMechState(0);
    chassis.waitUntil(36);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);
    base.tankTo(135);

    chassis.moveToPose(7,65,0,2000,{.maxSpeed = 50},false);
    chassis.moveToPose(24,33,270,2000,{.forwards=false, .lead = 0.3, .maxSpeed = 80},true);
    base.tankTo(90);
    chassis.moveToPose(24,60,0,1500,{});
    chassis.moveToPose(24,33,0,1500,{.forwards = false,.minSpeed=60});
    base.tankTo(225);
    chassis.moveToPose(20,20,225,2000,{.forwards=true});
    setMechState(5);

    /*
    chassis.setPose(53.25,29.75,90);
    chassis.moveToPose(20,22,70,1500,{.forwards = false,.lead = 0.25, .maxSpeed = 80},true);
    chassis.waitUntil(29);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    base.tankTo(120, true);
    setIntakeState(1);
    chassis.moveToPose(5.5,57,0,2500,{.forwards = true,.lead=0.60,.maxSpeed =50,.minSpeed = 30},true);
    pros::delay(150);
    chassis.moveToPose(24,25,290,2000,{.forwards=false,.minSpeed=60},true);
    base.tankTo(110, true);
    chassis.moveToPose(36,43,90,2000,{.forwards = true,.lead=0.80,.maxSpeed =80,.minSpeed = 40},true);
    base.tankTo(-50,true);
    chassis.moveToPose(65,65,50,2000,{.forwards = true,.lead=0.80,.maxSpeed =127,.minSpeed = 65},true);
    chassis.moveToPose(52,52,50,750,{.forwards = false, .minSpeed = 80});
    base.tankTo(270, true);
    
    intakeLift.set_value(1);
    setMechState(1);
    
    chassis.moveToPose(52,10,180,3000,{.forwards = true,.lead=0.0, .maxSpeed = 90});
    chassis.waitUntil(20);
    goalClamp.set_value(0);
    setAdjustment(true);
    setAutonState(false);
    
    chassis.waitUntilDone();
    intakeLift.set_value(0);


    chassis.moveToPose(52,0,180,1600,{});
    chassis.waitUntil(10);
    setIntakeState(2);
    setMechState(2);


    base.tankTo(0);
    
    chassis.moveToPose(58,-2,90,700,{.lead = 0},false);
    setMechState(3);
    pros::delay(500);

    ls.move_voltage(-9000);
    rs.move_voltage(-9000);
    pros::delay(700);


    //chassis.moveToPose(40,-2,90,1000,{.forwards = false, .lead = 0, .minSpeed=60},true);

    /*
    setMechState(2);
    pros::delay(50);
    ls.move_voltage(0);
    rs.move_voltage(0);

    base.tankTo(180,true);
    setMechState(3);*/




    //lift up the intake while going forward and move lb up
    //base.tankTo(0);
    //forward 5 inches
    //scoring the ring with lb
    //back up a few inches
    //base.tankTo(180);

    //ls.move_voltage(11000);
    //rs.move_voltage(11000);
    //pros::delay(500);
    //ls.move_voltage(0);
    //rs.move_voltage(0);

}

void redLeft(){
    chassis.setPose(-58.33,13.23,217);
    
    setMechState(3);
    pros::delay(500);
    ls.move_voltage(-3000);
    rs.move_voltage(-3000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    chassis.moveToPose(-20,24,255,1500,{.forwards=false, .maxSpeed = 80},true);
    chassis.waitUntil(20);
    setMechState(0);
    chassis.waitUntil(36);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    base.tankTo(-188);
    chassis.moveToPose(-6,56,0,2000,{.maxSpeed = 70},false);

    chassis.moveToPose(-24,24,45,1500,{.forwards=false, .lead = 0.6, .minSpeed = 40},false);
    setIntakeState(0);
    base.tankTo(-143);
    setIntakeState(1);
    //chassis.moveToPose(-24,50,0,1500,{});
    chassis.moveToPose(-24,50,0,2000,{.minSpeed = 70,.earlyExitRange=3}, false);
    setMechState(4);
    chassis.moveToPose(-75,68,270,1500,{.lead = 0.2, .minSpeed = 100},false);

    setMechState(0);
    chassis.moveToPose(-48,24,0,2000,{.forwards=false, .minSpeed = 45, .earlyExitRange = 3}); 
    base.tankTo(37,true);

    intakeLift.set_value(1);
    chassis.moveToPose(-49,0,180,1200,{.earlyExitRange = 3});
    chassis.moveToPose(-49,15,180,1200,{.forwards = false, .earlyExitRange = 3});


    chassis.moveToPose(-20,5,110,1000,{},false);
    pros::delay(10);
    setMechState(5);
    
    




    /*
    chassis.moveToPose(-60,68,270,2000,{.forwards = false, .lead = 0, .minSpeed = 70.});
    chassis.moveToPose(-70,68,270,2000,{.minSpeed = 70});
    chassis.moveToPose(-60,68,270,2000,{.forwards = false, .lead = 0, .minSpeed = 70});
    chassis.moveToPose(-70,68,270,2000,{.minSpeed = 70});

    /*
    chassis.moveToPose(-24,33,0,1500,{.forwards = false,.minSpeed=60});
    //base.tankTo(130);
    chassis.moveToPose(-20,20,135,2000,{.forwards=false});
    setMechState(5);

    


    /*
    chassis.setPose(-53.25,29.75,270);
    chassis.moveToPose(-20,22,290,1500,{.forwards = false,.lead = 0.25, .maxSpeed = 80},true);
    chassis.waitUntil(29);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    base.tankTo(240,true);

    setIntakeState(1);
    chassis.moveToPose(-9,58,0,2500,{.forwards = true,.lead=0.60,.maxSpeed = 50,.minSpeed = 50},true);

    pros::delay(150);

    chassis.moveToPose(-30,25,70,2000,{.forwards=false, .lead = 0.3, .minSpeed=60},true);
    base.tankTo(250, true);

    chassis.moveToPose(-36,43,270,2000,{.forwards = true,.lead=0.80,.maxSpeed =80,.minSpeed = 40},true);
    //base.tankTo(50,true);

    chassis.moveToPose(-70,70,-50,2000,{.forwards = true,.lead=0.80,.maxSpeed =127,.minSpeed = 80},true);
    pros::delay(300); 
    chassis.moveToPose(-52,52,-50,750,{.forwards = false, .minSpeed = 80});

    base.tankTo(90, true);

    intakeLift.set_value(1);
    setMechState(1);
    

    chassis.moveToPose(-48,10,180,3000,{.forwards = true,.lead=0.0, .maxSpeed = 90});
    chassis.waitUntil(20);
    setAutonState(false);
    setAdjustment(true);

    
    
    chassis.waitUntilDone();
    intakeLift.set_value(0);

    chassis.moveToPose(-48,4,180,1600,{});
    chassis.waitUntil(10);
    //setIntakeState(2);
    //setMechState(2);
    chassis.waitUntil(20);
    goalClamp.set_value(0);
    chassis.waitUntilDone();

    base.tankTo(0);
    
    chassis.moveToPose(-58,0,270,700,{.lead = 0},false);
    setIntakeState(2);
    setMechState(3);
    pros::delay(500);


    ls.move_voltage(-6000);
    rs.move_voltage(-6000);
    pros::delay(700);
    */


    
}

void redRight(){
    /* kalahari redright auton
    chassis.setPose(49,60,270);
    chassis.moveToPose(15,57.5,260,2000,{.lead=0.23,.minSpeed=200},true);
    chassis.waitUntil(90);
    doinker.set_value(1);
    chassis.waitUntilDone();


    chassis.moveToPose(24,64,270,2000,{.forwards = false,.lead=0.3, .minSpeed = 80},true);
    chassis.waitUntil(40);
    chassis.waitUntilDone();

    doinker.set_value(0);
    pros::delay(250);

    base.tankTo(-170);
    chassis.moveToPose(9,53,84,2000,{.forwards = false, .lead = 0.2, .maxSpeed = 50},true);
    chassis.waitUntil(15);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    setIntakeState(1);
    pros::delay(200);

    chassis.moveToPose(50,24,180,2000,{.horizontalDrift = 10, .lead = 0.4 , .minSpeed = 50},true);
    chassis.waitUntil(20);
    goalClamp.set_value(0);
    chassis.waitUntilDone();
    goalClamp.set_value(0);
    pros::delay(100);
    base.tankTo(180);
    chassis.moveToPose(13,20.5,90,2000,{.forwards=false, .maxSpeed = 70});
    chassis.waitUntil(33);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    base.tankTo(-60);
    chassis.moveToPose(0,60,330,2000,{});
    chassis.moveToPose(9,22,330,1500,{.forwards = false});
    base.tankTo(45);

    setMechState(5);
    */
    chassis.setPose(50,37.8,270);
    setIntakeState(3);
    chassis.moveToPose(21,47,300,2000,{.lead = 0.05, .minSpeed=200},true);
    chassis.waitUntil(40);
    doinker.set_value(1);
    chassis.waitUntilDone();


    chassis.moveToPose(48,30,300,2000,{.forwards=false, .minSpeed = 60},true);
    chassis.waitUntil(20);
    doinker.set_value(0);
    chassis.waitUntilDone();

    setIntakeState(0);

    base.tankTo(190 ,true);
    chassis.moveToPose(22,24,80,2000,{.forwards =false}, true);
    chassis.waitUntil(20);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    setIntakeState(1);

    chassis.moveToPose(50,54,45,1500,{.maxSpeed = 80, .minSpeed = 30});

    chassis.moveToPose(70,74,45,1500,{.minSpeed = 127});


    chassis.moveToPose(24,24,45,1300,{.forwards = false},true);
    chassis.waitUntil(15);
    intakeLift.set_value(0);
    chassis.waitUntilDone();

    setIntakeState(0);


    


    /*
    base.tankTo(160);
    chassis.moveToPose(54,70,85,1500,{},true);
    chassis.waitUntil(20);
    doinker.set_value(1);
    chassis.waitUntilDone();
    ls.move_voltage(12000);
    rs.move_voltage(-12000);
    pros::delay(500);
    base.tankTo(-30, cw);
    doinker.set_value(0);
    chassis.moveToPose(12,24,300,2000,{});


    /*

    */


    /*
    chassis.moveToPose(20,43,180,1000,{});
    chassis.moveToPose(20,55,180,1000,{.forwards = false, .lead = 0});
    
    


    //chassis.moveToPose(16,45,180,1500,{.minSpeed = 30});
    //pros::delay(1500);



    
    base.tankTo(20,true);

    goalClamp.set_value(0);

    
    base.tankTo(300);
    

    chassis.moveToPose(16,16,9,1500,{.forwards = false, .lead = 0.6, .maxSpeed = 80, .minSpeed = 30},true);
    chassis.waitUntil(30);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    base.tankTo(250,true);

    setMechState(2);
    chassis.moveToPose(60,76,0,3000,{.lead = 0.65, .minSpeed = 100});
    chassis.waitUntil(80);
    setMechState(3);
    chassis.waitUntilDone();

    //chassis.moveToPose(58,76,0,2000,{.lead = 0.6, .maxSpeed = 80, .minSpeed = 79});
    pros::delay(100);
    setMechState(2);

    chassis.moveToPose(23,23,45,2000,{.forwards = false, .minSpeed = 70});
    
    base.tankTo(45,true);
    setMechState(3);
    


    




    /*
    chassis.setPose(-24,53.25,0);
    chassis.moveToPose(-24,24,0,2000,{.forwards = false,.minSpeed = 60},true);
    chassis.waitUntil(23);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);
    base.tankTo(90);
    chassis.moveToPose(-51,23,270,4800,{.minSpeed = 30},false);
    pros::delay(800);
    chassis.moveToPose(-24,24,270,1500,{.forwards = false, .minSpeed = 50},false);
    setIntakeState(2);
    goalClamp.set_value(0);
    base.tankTo(270);
    setIntakeState(2);
    chassis.moveToPose(-48, 24, 100,1300,{.forwards = false, .lead = 0, .minSpeed = 30}, false);
    base.tankTo(0);
    chassis.moveToPose(-45.5,2,0,2000,{.forwards = false, .maxSpeed = 50, .minSpeed = 40}, true);
    chassis.waitUntil(20);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    ls.move_voltage(12000);
    rs.move_voltage(12000);
    pros::delay(65);
    base.tankTo(-30);
    chassis.moveToPose(-24,47,30,3000,{.minSpeed = 70},false);
    base.tankTo(-90);
    setIntakeState(1);
    chassis.moveToPose(0,46,90,1000,{.minSpeed = 40},false);
    ls.move_voltage(-12000);
    rs.move_voltage(-12000);
    pros::delay(100);
    ls.move_voltage(0);
    rs.move_voltage(0);
    base.tankTo(315);
    setIntakeState(2);
    pros::delay(500);
    base.tankTo(-90);
    setIntakeState(1);
    setMechState(4);
    chassis.moveToPose(20,47,90,1200,{.minSpeed = 40},false);
    
    base.tankTo(150);
    

    ls.move_voltage(11000);
    rs.move_voltage(11000);
    
    pros::delay(200);
    pros::delay(300);
    ls.move_voltage(0);
    rs.move_voltage(0);
    */

}

void AWPRed(){
    chassis.setPose(-58.33,13.23,217);
    
    setMechState(3);
    pros::delay(500);
    ls.move_voltage(-4000);
    rs.move_voltage(-4000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    chassis.moveToPose(-20,24,255,1500,{.forwards=false, .maxSpeed = 80},true);
    chassis.waitUntil(20);
    setMechState(0);
    chassis.waitUntil(36);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    base.tankTo(-188);
    chassis.moveToPose(-7,56,0,2000,{.maxSpeed = 50, .earlyExitRange = 3 },false);
    chassis.moveToPose(-24,24,45,1500,{.forwards=false, .lead = 0.6, .minSpeed = 40},true);
    base.tankTo(-143);
    //chassis.moveToPose(-24,50,0,1500,{});
    chassis.moveToPose(-24,50,0,2000,{.minSpeed = 50,.earlyExitRange=3}, false);
    chassis.moveToPose(-48,24,60,2000,{.minSpeed = 50,.earlyExitRange = 5});
    base.tankTo(37);
    goalClamp.set_value(0);

    chassis.moveToPose(-48,-24,180,1000,{.earlyExitRange = 3},true);
    chassis.waitUntil(20);
    setIntakeState(0);
    chassis.waitUntilDone();

    base.tankTo(-53,true);
    chassis.moveToPose(-22,-20,270,2000,{.forwards = false}, true);
    chassis.waitUntil(22);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);
    base.tankTo(37,true);
    chassis.moveToPose(-24,-48,180,1500,{.minSpeed = 50, .earlyExitRange = 3});
    chassis.moveToPose(-27,-23,135, 1200, {.forwards = false, .minSpeed = 50, .earlyExitRange = 3});
    base.tankTo(180);
    setMechState(5);






}

void AWPBlue(){
    chassis.setPose(-58.33,-13.23,323);
    
    setMechState(3);
    pros::delay(500);
    ls.move_voltage(-4000);
    rs.move_voltage(-4000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    chassis.moveToPose(-20,-24,285,1500,{.forwards=false, .maxSpeed = 80},true);
    chassis.waitUntil(20);
    setMechState(0);
    chassis.waitUntil(36);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    base.tankTo(-172);
    chassis.moveToPose(-7,-56,180,2000,{.maxSpeed = 50, .earlyExitRange = 3 },false);
    chassis.moveToPose(-24,-24,135,1500,{.forwards=false, .lead = 0.6, .minSpeed = 40},true);
    base.tankTo(143);
    //chassis.moveToPose(-24,50,0,1500,{});
    chassis.moveToPose(-24,-50,180,2000,{.minSpeed = 50,.earlyExitRange=3}, false);
    chassis.moveToPose(-44,-24,120,2000,{.minSpeed = 50,.earlyExitRange = 5});
    base.tankTo(-37);
    goalClamp.set_value(0);

    chassis.moveToPose(-44,25,0,2000,{.earlyExitRange=2});
    base.tankTo(53,true);
    chassis.moveToPose(-20,27,270,2000,{.forwards=false},true);
    chassis.waitUntil(22);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    base.tankTo(-37,true);
    chassis.moveToPose(-20,50,0,1500,{.minSpeed = 50, .earlyExitRange = 3});
    chassis.moveToPose(-20,25,0,1500,{.forwards=false,.minSpeed=30,.earlyExitRange=2});
    base.tankTo(180,true);
    setMechState(5);


}




void blueLeftElim(){
    chassis.setPose(-24,-53.25,180);
    chassis.moveToPose(-24,-22,180,2000,{.forwards = false,.minSpeed = 60},true);
    chassis.waitUntil(25);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);
    base.tankTo(270);
    chassis.moveToPose(-51,-27,270,5000,{.minSpeed = 30},false);
    pros::delay(800);
    chassis.moveToPose(-24,-24,270,1500,{.forwards = false, .minSpeed = 50},false);
    base.tankTo(-45);
    chassis.moveToPose(-58,-58,225,6000,{},false);
    setDoinkerState();
}

void redRightElim(){
    chassis.setPose(-24,53.25,0);
    chassis.moveToPose(-24,22,0,5000,{.forwards = false,.minSpeed = 60},true);
    chassis.waitUntil(25);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    pros::delay(1000);
    setIntakeState(1);
    base.tankTo(90);
    chassis.moveToPose(-51,23,270,4800,{.minSpeed = 30},false);
    pros::delay(800);
    chassis.moveToPose(-21,24,270,1500,{.forwards = false, .minSpeed = 50},false);
    base.tankTo(45);
    chassis.moveToPose(-58,58,315,6000,{}, false);
    setDoinkerState();
}

void qualRedRight(){
    chassis.setPose(-58.67,-15.204,323);
    
    setMechState(3);
    pros::delay(500);
    ls.move_voltage(-4000);
    rs.move_voltage(-4000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    chassis.moveToPose(-20,-25,270,1500,{.forwards=false, .maxSpeed = 80},true);
    chassis.waitUntil(20);
    setMechState(0);
    chassis.waitUntil(36);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    base.tankTo(-80);

    chassis.moveToPose(-4,-15,60,1500,{.minSpeed=5},false);
    doinker.set_value(1);
    pros::delay(100);

    
    
    chassis.moveToPose(-24,-24, 60, 1500,{.forwards = false, .earlyExitRange=1.5},false);
    doinker.set_value(0);
    setIntakeState(1);

    chassis.moveToPose(-15,-19,60,1500,{.minSpeed = 80, .earlyExitRange = 1},false);

    /*
    chassis.moveToPose(-10,-10,75,1500,{.lead = 0.3},true);
    chassis.waitUntil(24);
    setIntakeState(0);
    chassis.waitUntilDone();
    doinker.set_value(1);
    pros::delay(300);

    chassis.moveToPose(-24,-28,30,1500,{.forwards = false,.earlyExitRange = 1},false);
    doinker.set_value(0);
    setIntakeState(1);
    chassis.moveToPose(-24,-12,0,1500,{.minSpeed = 30, .earlyExitRange = 1});
    */

    chassis.moveToPose(-22,-28,30,1500,{.forwards = false,.minSpeed=80,.earlyExitRange = 1},false);
    base.tankTo(143,true);

    chassis.moveToPose(-22, -50, 180, 1200, {.minSpeed = 50, .earlyExitRange = 2});

    chassis.moveToPose(-50,-70,250,1500,{.minSpeed=80, .earlyExitRange = 2});
    chassis.moveToPose(-70,-70,260,1500, {.maxSpeed=50, .minSpeed = 40});
    chassis.moveToPose(-60,-64,250,1000,{.forwards=false});
    chassis.moveToPose(-70,-70,255,1000,{.maxSpeed=50, .minSpeed = 40});
    chassis.moveToPose(-22,-22,225,1500,{.forwards=false});





}