/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.skyStoneVision;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.firstinspires.ftc.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.easyopencv.OpenCvInternalCamera;
import org.firstinspires.ftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.Hardware.backClosePos;
import static org.firstinspires.ftc.teamcode.Hardware.backOpenPos;
import static org.firstinspires.ftc.teamcode.Hardware.frontClosePos;
import static org.firstinspires.ftc.teamcode.Hardware.frontOpenPos;
import static org.firstinspires.ftc.teamcode.skyStoneVision.stonePosition.center;
import static org.firstinspires.ftc.teamcode.skyStoneVision.stonePosition.left;
import static org.firstinspires.ftc.teamcode.skyStoneVision.stonePosition.right;

@Autonomous(name="EsayOpenCv Test", group="Misgav")
public class InternalCameraExample extends LinearOpMode
{
    OpenCvCamera phoneCam;
    Hardware robot = new Hardware();
    public skyStoneVision vision;
    Mat processImage;
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
    public void proces(int red){
        if(red > -1){
            vision.skyStone.croper = 130;
        }
        vision.red = red;
        sleep(500);
    }
    skyStoneVision.stonePosition  getPosition(){
        skyStoneVision.stonePosition prev = vision.position;
        int counter = 0;
        for(int i = 0; i < 10 && counter < 10; i++){
            updatePos();
            if(prev != vision.position){
                i = 0;
                counter++;
            }
            prev = vision.position;
            sleep(40);
        }
        return prev;
    }
    void updatePos(){
        if ((!(vision.skyStone.blobs.isEmpty()))) {
            vision.stone.y = vision.skyStone.findBlobsOutput().get(0).y;
            vision.stone.x = vision.skyStone.blobs.get(0).getCenter().x;
            if (vision.stone.x < (250 + vision.red)) {
                vision.position = right;
            } else {
                vision.position = center;
            }
        }
    }
    private class loadOpenCvLib extends Thread {
        int position = 0;

        loadOpenCvLib() { setName("loadOpenCvLib"); }

        @Override
        public void run() {
            try {

                Log.d(TAG, "Initializing OpenCV");
                if (!OpenCVLoader.initDebug()) {
                    boolean success = OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION, appUtil.getActivity(), mLoaderCallBack);
                    Log.d(TAG, "init ???");
                    if (success) {
                        Log.d(TAG, "Init success");
                    } else {
                        Log.d(TAG, "Init fail");
                    }
                } else {
                    Log.d(TAG, "openCv library found");
                    mLoaderCallBack.onManagerConnected(LoaderCallbackInterface.SUCCESS);
                }
            } finally {

            }
            //catch (InterruptedException ignored) {
//
            //}
        }
    }

    public void initOpenCV(HardwareMap hardwareMap) {

        //Thread loadOpenCvLibThread = new loadOpenCvLib();
        //loadOpenCvLibThread.start();
        //sleep(500);

        Log.d(TAG, "Initializing OpenCV");
        if (!OpenCVLoader.initDebug()) {
            boolean success = OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION, appUtil.getActivity(), mLoaderCallBack);
            Log.d(TAG, "init ???");
            if (success) {
                Log.d(TAG, "Init success");
            } else {
                Log.d(TAG, "Init fail");
            }
        } else {
            Log.d(TAG, "openCv library found");
            mLoaderCallBack.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
        vision = new skyStoneVision();
        vision.createObjects();

        processImage = new Mat();


        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        phoneCam.setPipeline(new SamplePipeline());

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
    }




    @Override
    public void runOpMode()
    {

//        robot.init(hardwareMap,true);
        initOpenCV(hardwareMap);
        waitForStart();

        if (opModeIsActive())
        {
            telemetry.addData("position",getPosition());
            telemetry.addData("size",vision.skyStone.blobs.size());
            telemetry.addData("X position",vision.stone.x);
            telemetry.addData("Y position",vision.stone.y);
            if (vision.goldMineralFound())
            {
                 telemetry.addLine("Found stone");
            }
            else
            {
                telemetry.addLine("no stone was found ");
            }
//            switch (vision.position){
//                case right:
//                    robot.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
//                    robot.led.displayPattern();
//                    break;
//                case center:
//                    robot.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
//                    robot.led.displayPattern();
//                    break;
//                case left:
//                    robot.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
//                    robot.led.displayPattern();
//                    break;
//
//            }
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("stone center", vision.stone.x);
             telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
             telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
             telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
             telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
             telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
             telemetry.update();
             proces(60);
        }
        while (opModeIsActive()){
        }
    }


    class SamplePipeline extends OpenCvPipeline
    {

        @Override
        public Mat processFrame(Mat input)
        {


            processImage = (vision.processFrame(input));



            return processImage;
        }
    }
}
