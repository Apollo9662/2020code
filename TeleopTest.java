package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareTester;
import org.firstinspires.ftc.teamcode.functions;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.Hardware.backClosePos;
import static org.firstinspires.ftc.teamcode.Hardware.backOpenPos;
import static org.firstinspires.ftc.teamcode.Hardware.frontClosePos;
import static org.firstinspires.ftc.teamcode.Hardware.frontOpenPos;
import static org.firstinspires.ftc.teamcode.MathFunctions.inRange;

@TeleOp(name="testerBot", group="APOLLO")
public class TeleopTest extends functions {
    HardwareTester robot = new HardwareTester();
    boolean a = false;
    public void runOpMode() {
        robot.init(hardwareMap, false);
        telemetry.addData("STATE - ", "ready for test");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            if(a){
                robot.motor0.setPower(0.25);
                robot.motor1.setPower(0.25);
                robot.motor2.setPower(0.25);
                robot.motor3.setPower(0.25);

                robot.s1.setPosition(0);
            }else{
                robot.motor0.setPower(1);
                robot.motor1.setPower(1);
                robot.motor2.setPower(1);
                robot.motor3.setPower(1);

                robot.s1.setPosition(1);
            }
            if(gamepad1.a){
                a = !a;
                while (gamepad1.a && opModeIsActive()){
                }
            }
            telemetry.addData("m0 encoder -",robot.motor0.getCurrentPosition());
            telemetry.addData("m1 encoder -",robot.motor1.getCurrentPosition());
            telemetry.update();
        }

    }

}





