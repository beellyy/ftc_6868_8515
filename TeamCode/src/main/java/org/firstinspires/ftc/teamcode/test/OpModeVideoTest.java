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

package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LocationDectator;
import org.firstinspires.ftc.teamcode.MineralDetector;
import org.firstinspires.ftc.teamcode.MoveAction;
import org.firstinspires.ftc.teamcode.RobotRoverRuckus;

import java.util.List;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forwards, and causes the encoders to count UP.
 * <p>
 * The desired path in this example is:
 * - Drive forward for 48 inches
 * - Spin right for 12 Inches
 * - Drive Backwards for 24 inches
 * - Stop and close the claw.
 * <p>
 * The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 * that performs the actual movement.
 * This methods assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "ZTest Videos", group = "Test")
@Disabled
public class OpModeVideoTest extends LinearOpMode {

    /* Declare OpMode members. */

    private RobotRoverRuckus robot = null;

    private LocationDectator locationDectator = null;

    private String goldMinePosition = "No";

    private int initQuadrant = 0;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtimeTemp = new ElapsedTime();
    private ElapsedTime runtimeLocate = new ElapsedTime();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        robot = new RobotRoverRuckus(hardwareMap, telemetry);

        initHardware();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", robot);    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        goldMineralDectect();
        locateStart();
        while (opModeIsActive()) {

            if(runtimeTemp.seconds()>2){
                locate();
                runtimeTemp.reset();
            }

            // Display it for the driver.
            telemetry.addData("Robot Status:", robot);
            telemetry.update();
        }
    }

    /**
     * 初始化硬件
     */
    private void initHardware() {


    }

    /**
     * 检测金矿
     * 左右旋转伺服来寻找金矿
     * todo：考虑摄像头宽度不够的解决方法
     */
    private void goldMineralDectect() {
        MineralDetector detection = new MineralDetector();
        detection.init(robot);
        detection.start();
        runtimeTemp.reset();
        // 移动云台扫描
        while (opModeIsActive() && runtimeTemp.seconds() < 5) {

            // Display the current value
            String goldMinePosition = detection.searchGoldMine();
            if (!"No".equals(goldMinePosition)) {
                break;
            }

            // Set the servo to the new camPosition and pause;
            //TODO:此处需要定位程序的时间间隔
            sleep(RobotRoverRuckus.CAM_CYCLE_MS);
            idle();
        }
        detection.stop();

    }

    /**
     * 开始定位
     */
    private void locateStart() {
        locationDectator = new LocationDectator();
        locationDectator.init(robot);

        locate();

        if (robot.location.found) {
            initQuadrant = robot.location.quadrant();
        }

    }

    /**
     * 来回旋转摄像头伺服，通过视频进行定位
     */
    private void locate() {
        runtimeLocate.reset();
        // 转动摄像头云台扫描
        while (opModeIsActive() && runtimeLocate.seconds() < 3) {
            // 获取当前定位
            robot.location = locationDectator.searchLocation();
            if (robot.location.found) {
                break;
            }

            //camScanNextStep();

            // Set the servo to the new camPosition and pause;
            //TODO:此处需要定位程序的时间间隔
            sleep(RobotRoverRuckus.CAM_CYCLE_MS);
            idle();
        }
    }


}
