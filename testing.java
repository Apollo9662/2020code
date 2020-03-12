package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.functions;

public class testing extends functions {
    public void runOpMode() {
        Hardware robot = new Hardware();
        robot.init(hardwareMap,true);
        double v;
        double Elb;
        double Elf;
        double Erb;
        double Erf;
        double afl;
        double afr;
        double abl;
        double abr;
        double bfl;
        double bfr;
        double bbl;
        double bbr;

        double x;
        double y;

        double mainA;
        double mainB;
        waitForStart();
        while (opModeIsActive()){
            v = gamepad1.right_stick_y;
            Elb = robot.driveLeftBack.getCurrentPosition();
            Elf = robot.driveLeftFront.getCurrentPosition();
            Erb = robot.driveRightBack.getCurrentPosition();
            Erf = robot.driveRightFront.getCurrentPosition();
            robot.setDriveMotorsPower(v, Hardware.DRIVE_MOTOR_TYPES.ALL);
            afl = v * Elf;
            abl = v * Elb;
            afr = v * Erf;
            abr = v * Erb;
            bbl = abl;
            bbr = abr;
            bfl = afl;
            bfr = afr;
            mainB = bbl + bbr + bfl + bfr;
            mainA = abl + abr + afl + afr;

            x = mainB;
            y = mainA;
            telemetry.addData("x", x);
            telemetry.addData("y",y);
            telemetry.update();


        }

    }
}
