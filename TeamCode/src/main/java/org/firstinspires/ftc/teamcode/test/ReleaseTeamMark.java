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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.teamcode.ImuReader;
import org.firstinspires.ftc.teamcode.LocationDectator;
import org.firstinspires.ftc.teamcode.MineralDetector;
import org.firstinspires.ftc.teamcode.RobotRoverRuckus;

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

@Autonomous(name = "release", group = "6868")
@Disabled
public class ReleaseTeamMark extends LinearOpMode {

    /* Declare OpMode members. */
    protected DcMotor leftFront = null;
    protected DcMotor rightFront = null;
    protected DcMotor leftBack = null;
    protected DcMotor rightBack = null;
    // 1挂钩上升 -1挂钩下降
    protected DcMotor elevator = null;
    protected DcMotor topLeft = null;
    protected DcMotor topRight = null;
    protected DcMotor topMid = null;
    protected Servo servoLeft = null;
    protected Servo servoRight = null;
    // test
    protected Servo camHori = null;

    protected Servo lock = null;
    private BNO055IMU imu = null;

    static final double     LOCK_OPEN = 0;
    static final double     LOCK_CLOSE = 1;
    static final double     CLAW_OPEN = 0;
    static final double     CLAW_CLOSE = 1;

    protected RobotRoverRuckus robot = null;

    protected LocationDectator locationDectator = null;

    protected String goldMinePosition = "No";

    static final boolean IMU_ACTIVE = true;    // 如果不是用imu，修改成false

    static final int START_FROM_CRATER = 1;
    static final int START_FROM_STORAGE = 2;

    protected int defaultStartPoint = START_FROM_CRATER;
    protected int startPoint = 0;


    protected ElapsedTime runtime = new ElapsedTime();
    protected ElapsedTime runtimeTemp = new ElapsedTime();
    protected ElapsedTime runtimeLocate = new ElapsedTime();

    protected ImuReader imuReader = new ImuReader();


    static final double FORWARD_SPEED = 0.5;
    static final double FORWARD_SLOW_SPEED = 0.3;
    static final double TURN_SPEED = 0.25;

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 9 / 2.54;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double ROBOT_LENGTH_WHEEL = 29 / 2.54;    // 前后轮的轴距
    static final double ROBOT_WIDTH_WHEEL = 37 / 2.54;    // 左右轮的轴距
    static final double ROBOT_MOTION_CIRCUM = Math.PI * Math.sqrt(ROBOT_LENGTH_WHEEL * ROBOT_LENGTH_WHEEL + ROBOT_WIDTH_WHEEL * ROBOT_WIDTH_WHEEL);

    MineralDetector detection = null;
    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        robot = new RobotRoverRuckus(hardwareMap, telemetry);
        //设置摄像头位置
        robot.CAMERA_FORWARD_DISPLACEMENT = -50;
        robot.CAMERA_VERTICAL_DISPLACEMENT = 400;
        robot.CAMERA_LEFT_DISPLACEMENT = -220;

        initHardware();

        //detection = new MineralDetector();
        //detection.init(robot);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", robot);    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //着陆
        //landing();

        //检测金矿
        //goldMineralDectect();

        //定位
        //locateStart();

        // 设置默认值，如果找不到金矿则默认中间点

        releaseTeamMark();

        telemetry.addData("Status", robot);
        telemetry.update();
    }

    /**
     * 初始化硬件
     */
    private void initHardware() {

        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        topLeft = hardwareMap.get(DcMotor.class, "top_left");
        topRight = hardwareMap.get(DcMotor.class, "top_right");
        topMid = hardwareMap.get(DcMotor.class, "top_middle");
        servoLeft = hardwareMap.get(Servo.class, "servo_left");
        servoRight = hardwareMap.get(Servo.class, "servo_right");

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // test
        camHori = hardwareMap.get(Servo.class, "camera_horizontal");
        lock = hardwareMap.get(Servo.class, "lock");

        // 设置imu硬件
        imu = hardwareMap.get(BNO055IMU.class, "imu");


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 调整挂车锁伺服的运行区间。目前调整为0.6-0.9。
        // 此时，如setPosition(0)，代表实际设置值为0.6；如setPosition(0.5)，代表实际设置值为0.75；
        lock.scaleRange(0.6,0.9);
        lock.setPosition(LOCK_CLOSE);

        //servoLeft.scaleRange(0.2,0.8);
        //servoLeft.setPosition(CLAW_CLOSE);
        //servoRight.setDirection(Servo.Direction.REVERSE);
        //servoRight.scaleRange(0.2,0.8);
        //servoRight.setPosition(CLAW_CLOSE);

        // 调整摄像头云台水平伺服的运行区间。目前调整为0.6-1。
        // 此时，如setPosition(0)，代表实际设置值为0.6；如setPosition(0.5)，代表实际设置值为0.8；
        camHori.scaleRange(0.6,1);
        camHori.setPosition(1);
        // 初始化imu角度读取对象
        imuReader.initImu(imu);

        if (Vuforia.isInitialized()) {
            Vuforia.deinit();
        }
    }

    /**
     * 机器人出发后，走到距仓库40英寸左右距离时，面向仓库执行本方法中的动作
     * 1、释放机械臂，降落到地面附近
     * 2、打开机械爪，松开标志物
     * 3、收回机械臂
     */
    private void releaseTeamMark() {
        topMid.setPower(1.0);
        waitForCompete(1.0);
        topMid.setPower(0);


        //servoLeft.setPosition(CLAW_OPEN);
        //servoRight.setPosition(CLAW_OPEN);
        //waitForCompete(0.5);

        topMid.setPower(-0.9);
        waitForCompete(0.8);
        topMid.setPower(0);
    }

    /**
     * 从面对仓库的点出发，走到仓库，释放团队标志物之后
     * 执行驶向陨石坑的动作
     */
    private void storeGotoCrater() {

        //右转并倒退，到机器人尾部靠近墙
        goDistance(FORWARD_SPEED, 5, 5, 1);
        turnAngle(-20);
        goDistance(FORWARD_SPEED, -20, -20, 2);
        //左转5度，继续倒退，以稍微离开墙一点距离
        turnAngle(5);
        goDistance(FORWARD_SPEED, -15, -15, 2);
        //右转5度，拉直，继续退，直到越过墙中间位置的立柱
        turnAngle(-5);
        goDistance(FORWARD_SPEED, -45, -45, 2);
        //右转10度，向靠近墙的方向减速倒退，靠近陨石坑，尽量避免触碰地对联盟银矿的可能性
        turnAngle(-10);
        goDistance(FORWARD_SPEED * 0.5, -10, -10, 4);


    }


    /**
     * 设置机器人使用RunToPosition模式
     */
    public void runModeRunToPosition() {

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void runModeUsingEncoder() {

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void power(double speed) {
        powerLeft(speed);
        powerRight(speed);
    }

    private void powerLeft(double speed) {
        leftFront.setPower(speed);
        leftBack.setPower(speed);
    }

    private void powerRight(double speed) {
        rightFront.setPower(speed);
        rightBack.setPower(speed);
    }

    /**
     * @param angle angle为正左转，angle为负右转
     */
    public void turnAngle(double angle) {
        //原有方法，使用编码器进行转向
//        double distance  =0.6 * angle/360 * ROBOT_MOTION_CIRCUM;
//        goDistance(TURN_SPEED,-distance,distance,3);

        // 新方法，使用imu传感器的信号进行转向
        turnByIMU(angle, TURN_SPEED);
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

        // timeoutS = Math.max(Math.abs(leftInches), Math.abs(rightInches)) / 15 + 0.5;
        // 用一个简单的公式来尝试根据距离来计算行进时间
        // 逻辑是按距离除以速度。速度按照最大速度为15英寸测算；即如果speed为0.5，则按15×0.5=7.5英寸/秒计算时间。
        // 这个15，在编码器正常的情况下可以不管。
        // 如果编码器不正常，需要按时间控制，可以根据具体观测值进行调整
        // 方法：写个测试程序，让机器人跑3秒，测量起止点距离，算出此值
        double maxDistance = Math.max(Math.abs(leftInches),Math.abs(rightInches));
        double time = (maxDistance>0)?maxDistance/(15*speed):0;
        // 为保证即便上述数值不准确也不会真的提前停止，额外追加0.5秒时间。
        time+=0.5;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            //设置四个马达使用runToPosition模式运行
            runModeRunToPosition();

            // Determine new target camPosition, and pass to motor controller
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newLeftRareTarget = leftBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newRightRareTarget = rightBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            leftFront.setTargetPosition(newLeftFrontTarget);
            leftBack.setTargetPosition(newLeftRareTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            rightBack.setTargetPosition(newRightRareTarget);

            // reset the timeout time and start motion.
            runtime.reset();
            power(Math.abs(speed));

            //四个马达中只要有还没停止的，就等待。
            //如果马达行进中，isBusy返回true；行进到指定的targetPosition之后，isBusy返回false
            while (opModeIsActive() &&
                    (leftFront.isBusy() || leftBack.isBusy() || rightFront.isBusy() || rightBack.isBusy()) &&
                    (runtime.seconds() < time)) {
                idle();
            }

            // Stop all motion;
            power(0);
        }

    }

    /**
     * 等待指定的时间，期间软件系统不做其他事情
     *
     * @param second 秒数，可以是小数
     */
    private void waitForCompete(double second) {
        runtimeTemp.reset();
        while (opModeIsActive() && runtimeTemp.seconds() < second) {
            idle();
        }
    }


    /**
     * 此方法用于计算调整运行方向的power值
     * 方法是根据imu角度读数与当前角度的偏差，返回一个调整值
     * 角度偏差越大调整值越大
     * 在定向直线巡航时可以使用
     * 暂未使用
     *
     * @return 调整值, +代表向左调整 -代表向右调整.
     */
    private double checkDirection() {

        double correction;
        double angle;
        // 调整比例，简单处理直接用小数；复杂点可以考虑用三角函数计算；
        double gain = .10;

        angle = imuReader.getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * 使用IMU进行转向控制
     *
     * @param degrees 转向角度, 正数表示向左转，负数表示向右转，0不做处理
     */
    private void turnByIMU(double degrees, double power) {
        if(degrees == 0)
            return;
        // 不使用RunToPosition模式，而是使用RunUsingEncoder模式。
        // 此时给的power值，实际上代表的是速度。额定最高速度为1，如power=0.5则恒定半速巡航
        runModeUsingEncoder();

        // 根据IMU读数，进行旋转
        // 如果旋转过程中读取到的当前角度与目标角度的偏差大于1度，则继续旋转；小于等于1度则认为已经结束。
        // 此数值可以根据实践观察调整。
        //
        // 可以自动根据角度差确定向左还是向右调整
        // 如果调整过程超过3秒则停止
        // 根据实践经验，旋转速度需要控制不要太高，如太高可能有旋转太快导致角度超出目标角，此时会出现来回调整的情况。
        // 另，如果想要更精确，4个驱动马达应该设置setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // 这样当旋转角度达到目标角之后，马达power设置0之后，会自动锁死，防止受惯性影响多转的情况
        runtimeTemp.reset();
        // 重置IMU中的角度，会把当前方向作为0度
        imuReader.resetAngle();
        double currentDegrees = imuReader.getAngle();
        while (opModeIsActive() &&
                Math.abs(degrees - currentDegrees) > 1 &&
                runtimeTemp.seconds() < 3) {
            // 根据imu角度读数和目标角数值，计算下次调整方向
            // 若degrees为正，则目标角度为左侧的某个角度。此时currentDegrees从0开始。
            //    减法结果为正，大于0，返回1，则左侧power为负，右侧power为正，机器人左转
            //    若机器人旋转超出degrees，减法结果为负，返回-1，则左侧power为正，右侧power为负，机器人右转
            // 若degrees为负，则目标角度为右侧的某个角度。此时currentDegrees从0开始。
            //    减法结果为负，小于0，返回-1，则左侧power为正，右侧power为负，机器人右转
            //    若机器人旋转超出degrees，减法结果为正，返回1，则左侧power为负，右侧power为正，机器人左转
            int direction = (degrees - currentDegrees) > 0 ? 1 : -1;
            // clockwise为正则向左调整，为负则向右调整
            powerLeft(-power * direction);
            powerRight(power * direction);
            currentDegrees = imuReader.getAngle();
            telemetry.addData("target angle:",degrees);
            telemetry.addData("current angle:", currentDegrees);
            telemetry.update();
            idle();
        }

        // 结束旋转，power设置为0
        power(0);

    }
}
