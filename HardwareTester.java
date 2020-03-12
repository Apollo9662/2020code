package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Disabled
public class HardwareTester {
    DcMotor motor0 = null;
    DcMotor motor1 = null;
    DcMotor motor2 = null;
    DcMotor motor3 = null;

    Servo s1 = null;
    public HardwareTester(){

    }
    void init(HardwareMap ahwMap,boolean teleop) {
        motor0 = ahwMap.get(DcMotor.class, "m0");
        motor1 = ahwMap.get(DcMotor.class, "m1");
        motor2 = ahwMap.get(DcMotor.class, "m2");
        motor3 = ahwMap.get(DcMotor.class, "m3");
        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        s1 = ahwMap.get(Servo.class,"s1");

    }
 }


