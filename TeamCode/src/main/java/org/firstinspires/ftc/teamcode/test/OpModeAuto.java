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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FieldCoordinate;
import org.firstinspires.ftc.teamcode.FieldData;
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

@Autonomous(name = "Z Auto", group = "test")
@Disabled
public class OpModeAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFront = null;
    private DcMotor leftRare = null;

    private DcMotor rightFront = null;
    private DcMotor rightRare = null;

    private DcMotor lift = null;
    private DigitalChannel liftLimit = null;

    private Servo camServoHor = null;
    private Servo camServoVer = null;

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

        //着陆
        landing();

        //检测金矿
        goldMineralDectect();

        //定位
        locateStart();

        //自动驾驶到
        gotoGoldMineral();

        //自动驾驶到仓库
        goToStore();

        //释放队标
        releaseTeammark();

        //自动驾驶到陨石坑
        gotoCrater();

        telemetry.addData("Status", robot);
        telemetry.update();
    }

    /**
     * 初始化硬件
     */
    private void initHardware() {

        leftFront = initMotor("lf");
        leftRare = initMotor("lr");
        rightFront = initMotor("rf");
        rightRare = initMotor("rr");

        //一般规定电机逆时针方向转为正向FORWARD
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRare.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRare.setDirection(DcMotor.Direction.REVERSE);

        //
        lift = initMotor("lift");
        liftLimit = hardwareMap.digitalChannel.get("lift_limit");
        liftLimit.setMode(DigitalChannel.Mode.INPUT);

        camServoHor = initServo("ch");
        camServoVer = initServo("cv");
        robot.camRobotHeadingPos = camServoHor.getPosition();

    }

    private Servo initServo(String name) {
        if (!hardwareMap.servo.contains(name))
            return null;
        //TODO:不存在的异常处理；
        return hardwareMap.servo.get(name);
    }

    private DcMotor initMotor(String name) {
        if (!hardwareMap.dcMotor.contains(name))
            return null;
        //TODO:不存在的异常处理；
        return hardwareMap.dcMotor.get(name);
    }

    /**
     * 机器人着陆
     */
    private void landing() {
        // 下降
        lift.setPower(1);
        runtimeTemp.reset();
        boolean limitNotPressed = true;
        while (opModeIsActive() &&
                (runtimeTemp.seconds() < 10) &&
                limitNotPressed
                ) {
            // 检测是否有限位开关，如果有，根据限位开关停止下降
            if (null != liftLimit) {
                limitNotPressed = liftLimit.getState();
            }
            // Display it for the driver.
            idle();
        }
        lift.setPower(0);

        // 脱钩，扭一下
        goDistance(RobotRoverRuckus.DRIVE_SPEED, 2, -2, 1);
        //向前，脱离钩子
        goDistance(RobotRoverRuckus.DRIVE_SPEED, 2, 2, 1);
        // 恢复角度
        goDistance(RobotRoverRuckus.DRIVE_SPEED, -2, 2, 1);
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

            // slew the servo, according to the rampUp (direction) variable.
            camScanNextStep();

            // Display the current value
            goldMinePosition = detection.searchGoldMine();
            if (!"No".equals(goldMinePosition)) {
                break;
            }

            // Set the servo to the new camPosition and pause;
            //TODO:此处需要定位程序的时间间隔
            // sleep(RobotRoverRuckus.CAM_CYCLE_MS);
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

            camScanNextStep();

            // Set the servo to the new camPosition and pause;
            //TODO:此处需要定位程序的时间间隔
            //sleep(RobotRoverRuckus.CAM_CYCLE_MS);
            idle();
        }
    }

    /**
     * 转动摄像头云台
     */
    private void camScanNextStep() {
        // slew the servo, according to the rampUp (direction) variable.
        if (robot.camScan) {
            // Keep stepping up until we hit the max value.
            robot.setCamPosition(robot.getCamPosition() + RobotRoverRuckus.CAM_INCREMENT);
            if (robot.getCamPosition() >= RobotRoverRuckus.CAM_RIGHT_MAX_POS) {
                robot.setCamPosition(RobotRoverRuckus.CAM_RIGHT_MAX_POS);
                robot.camScan = !robot.camScan;   // Switch ramp direction
            }
        } else {
            // Keep stepping down until we hit the min value.
            robot.setCamPosition(robot.getCamPosition() - RobotRoverRuckus.CAM_INCREMENT);
            if (robot.getCamPosition() <= RobotRoverRuckus.CAM_LEFT_MAX_POS) {
                robot.setCamPosition(RobotRoverRuckus.CAM_LEFT_MAX_POS);
                robot.camScan = !robot.camScan;  // Switch ramp direction
            }
        }
        camServoHor.setPosition(robot.getCamPosition());
    }

    /**
     *
     * */
    private void gotoGoldMineral() {
        if (!robot.location.found) {
            //
            //TODO: 没找到定位，需要明确处理方法

        }

        //根据检测到的金矿位置和机器人所在象限，获取金矿的坐标
        FieldCoordinate goldPosition = findFieldCoor("Mineral_" + goldMinePosition, initQuadrant);

        //根据角度计算偏移角度
        double angle = angleOfVector(robot.location.coordinate(), goldPosition, robot.location.heading);
        MoveAction action = MoveAction.BuildTurnAction(angle);
        executeAction(action);

        //行驶到金矿位置
        action = MoveAction.BuildForwardAction(goldPosition);
        executeAction(action);

    }

    /**
     * 自动驾驶向仓库运行
     */
    private void goToStore() {
        String actionName = "To_Storage_" + initQuadrant;
        List<MoveAction> actions = FieldData.defaultInstance().findAction(actionName);
        for (MoveAction action : actions) {
            executeAction(action);
        }
    }

    /**
     * 释放团队标志物
     */
    private void releaseTeammark() {

    }

    /**
     * 自动驾驶向陨石坑
     */
    private void gotoCrater() {

        String actionName = "To_Storage_" + initQuadrant;
        List<MoveAction> actions = FieldData.defaultInstance().findAction(actionName);
        for (MoveAction action : actions) {
            executeAction(action);
        }

    }

    /**
     * 执行指定动作
     *
     * @param action
     */
    private void executeAction(MoveAction action) {
        double distance = 0.0;
        locate();
        switch (action.actionType) {
            case Turn:
                //根据角度计算移动距离
                double angle = action.data.x - robot.location.heading;
                distance = RobotRoverRuckus.ROBOT_MOTION_CIRCUMFERENCE * angle / 360;
                goDistance(RobotRoverRuckus.TURN_SPEED, -distance, distance, 2);
                break;
            case Forward:
                distance = robot.location.coordinate().distance(action.data);
                goDistance(RobotRoverRuckus.DRIVE_SPEED, distance, distance, 3);
                break;
            case Backward:
                distance = robot.location.coordinate().distance(action.data);
                goDistance(RobotRoverRuckus.DRIVE_SPEED, -distance, -distance, 3);
                break;
            case ForwardByWall:
                distance = robot.location.coordinate().distance(action.data);
                //TODO:增加根据传感器判断到墙距离的功能
                goDistance(RobotRoverRuckus.DRIVE_SPEED, distance, distance, 3);
                break;
            default:
                break;
        }
    }

    /**
     * 根据机器人旋转角度反向调整摄像头云台角度
     * 如果将移动超过云台限位，则反向转动90度（可以指向另一个图像进行定位）
     *
     * @param angle 左右旋转的角度
     */
    private void camHeadingTune(double angle) {
        //检测旋转角度是否超过云台限制

    }

    /**
     * 等待指定的时间，期间软件系统不做其他事情
     *
     * @param second 秒数，可以是小数
     */
    private void wait(double second) {
        runtimeTemp.reset();
        while (opModeIsActive() && runtimeTemp.seconds() < second) {
            idle();
        }
    }


    /**
     * 获取场地上某个点的坐标
     *
     * @param type      点类别
     * @param quardrant 所在象限，坐标系为场地坐标，着陆器正中地面位置为坐标原点，指向蓝方控制区方向为y轴正方向
     * @return
     */
    private FieldCoordinate findFieldCoor(String type, int quardrant) {
        String pointName = type + "_" + quardrant;
        return FieldData.defaultInstance().findPoint(pointName);
    }

    private double angleOfVector(FieldCoordinate source, FieldCoordinate target, double robotHeading) {
        //计算2个方向的夹角
        double p2x = source.x + 10 * Math.cos(robotHeading);
        double p2y = source.y + 10 * Math.sin(robotHeading);

        //向量的点乘
        double vector = (target.x - source.x) * (p2x - source.x) + (target.y - source.y) * (p2y - source.y);
        //向量的模乘
        double sqrt = Math.sqrt(
                (Math.abs((target.x - source.x) * (target.x - source.x)) + Math.abs((target.y - source.y) * (target.y - source.y)))
                        * (Math.abs((p2x - source.x) * (p2x - source.x)) + Math.abs((p2y - source.y) * (p2y - source.y)))
        );
        //反余弦计算弧度
        double radian = Math.acos(vector / sqrt);
        //弧度转角度制
        return (int) (180 * radian / Math.PI);

    }


    /**
     * 设置机器人使用RunToPosition模式
     */
    public void runModeRunToPosition() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRare.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRare.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRare.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRare.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runModeUsingEncoder() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRare.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRare.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRare.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRare.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void power(double speed) {
        powerLeft(speed);
        powerRight(speed);
    }

    private void powerLeft(double speed) {
        leftFront.setPower(speed);
        leftRare.setPower(speed);
    }

    private void powerRight(double speed) {
        rightFront.setPower(speed);
        rightRare.setPower(speed);
    }


    public void turn(double speed) {
        runModeUsingEncoder();
        powerLeft(speed);
        powerRight(-speed);
    }


    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current camPosition.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired camPosition
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void goDistance(double speed,
                           double leftInches, double rightInches,
                           double timeoutS) {
        int newLeftFrontTarget;
        int newLeftRareTarget;
        int newRightFrontTarget;
        int newRightRareTarget;

        timeoutS = Math.max(Math.abs(leftInches), Math.abs(rightInches)) / 15 + 0.5;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            runModeRunToPosition();

            // Determine new target camPosition, and pass to motor controller
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int) (leftInches * RobotRoverRuckus.COUNTS_PER_INCH);
            newLeftRareTarget = leftFront.getCurrentPosition() + (int) (leftInches * RobotRoverRuckus.COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() + (int) (rightInches * RobotRoverRuckus.COUNTS_PER_INCH);
            newRightRareTarget = rightFront.getCurrentPosition() + (int) (rightInches * RobotRoverRuckus.COUNTS_PER_INCH);

            leftFront.setTargetPosition(newLeftFrontTarget);
            leftRare.setTargetPosition(newLeftRareTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            rightRare.setTargetPosition(newRightRareTarget);

            // reset the timeout time and start motion.
            runtime.reset();
            power(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target camPosition, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && leftRare.isBusy() && rightRare.isBusy() && rightRare.isBusy())
                    ) {

                // Display it for the driver.
                idle();
            }

            // Stop all motion;
            power(0);
        }
    }

}
