package org.firstinspires.ftc.teamcode.opmodes.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RedAuto")
public class RedAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        int ringCount = 0;

        //determining which program to run
        if (ringCount == 0) {
            //a-tile first
            RedAutoA auton = new RedAutoA();
            auton.runOpMode();
        } else if (ringCount == 1){
            //b-tile first
            RedAutoB auton = new RedAutoB();
            auton.runOpMode();
        } else if (ringCount == 4) {
            //c-tile first
            RedAutoCNew auton = new RedAutoCNew();
            auton.runOpMode();
        }
    }
}