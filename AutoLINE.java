package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Hardware.backClosePos;
import static org.firstinspires.ftc.teamcode.Hardware.frontOpenPos;

@Autonomous(name = "Auto line", group = "A ollo")
public class AutoLINE extends GyroOperator {
    public void runOpMode(){
        robot.init(hardwareMap, true);

        telemetry.addData("state ==> ", "Start");

        telemetry.update();
        waitForStart();
        pidDriveLowSpeed(20,0,0,DriveMode.normal,1);
        if (opModeIsActive()){
            telemetry.addData("ANGLE - ",robot.GetGyroAngle());
            telemetry.addData("CURRENT - ",robot.leftCollector.getCurrentPosition() / robot.CuiToIn);
            telemetry.update();
            sleep(10000);
            pidDriveLowSpeed(-9,0,0,DriveMode.normal,1);


        }
    }
}
