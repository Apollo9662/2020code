/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.MineralVision;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;

import android.util.Log;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="OpenCv Test", group="Misgav")
@Disabled
public class OpenCvTest_Linear extends LinearOpMode {

    private MineralVision vision;
    private final String TAG = "OpenCv Test";
    AppUtil appUtil  = AppUtil.getInstance();
    boolean first = true;

    private BaseLoaderCallback mLoaderCallBack = new BaseLoaderCallback(appUtil.getActivity()) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case BaseLoaderCallback.SUCCESS:
                    Log.d(TAG, "callback: OpenCv test success");
                    break;
                default:
                    Log.d(TAG, "callback: fail to load opencv");
                    break;
            }
        }
    };


    //@Override
    public void runOpMode() {

        Log.d(TAG, "Initializing OpenCV");
        if (!OpenCVLoader.initDebug()) {
            boolean success = OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION, appUtil.getActivity(), mLoaderCallBack);

            if (success) {
                Log.d(TAG, "Init success");
            }
            else {
                Log.d(TAG, "Init fail");
            }
        }
        else {
            Log.d(TAG, "openCv library found");
            mLoaderCallBack.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        vision = new MineralVision();
        // Start to display image of camera.
        //vision.init(hardwareMap.appContext, 0);
        vision.createObjects();
        //vision.enable();


        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //if (vision.goldMineralFound())
            {
            //    telemetry.addLine("Find gold mineral");
            }
            //else
            {
            //    telemetry.addLine("No gold mineral");
            }
            //telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(30);
        }
    }
}
