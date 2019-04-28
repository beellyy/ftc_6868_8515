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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Vuforia;

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

@Autonomous(name = "Test on Movement", group = "autonomous")

public class PureMovement extends LinearOpMode {

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

    protected RobotRoverRuckus robot = null;

    protected LocationDetector locationDetector = null;

    protected String goldMinePosition = "No";

    static final int START_FROM_CRATER = 1;
    static final int START_FROM_STORAGE = 2;

    protected int defaultStartPoint = START_FROM_CRATER;
    protected int startPoint = 0;


    protected ElapsedTime runtime = new ElapsedTime();
    protected ElapsedTime runtimeTemp = new ElapsedTime();
    protected ElapsedTime runtimeLocate = new ElapsedTime();

    protected ImuReader imuReader = new ImuReader();


    static final double FORWARD_SPEED = 0.5;
    static final double TURN_SPEED = 0.3;

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 9 / 2.54;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double ROBOT_LENGTH_WHEEL = 29 / 2.54;    // 前后轮的轴距
    static final double ROBOT_WIDTH_WHEEL = 37 / 2.54;    // 左右轮的轴距
    static final double ROBOT_MOTION_CIRCUM = Math.PI * Math.sqrt(ROBOT_LENGTH_WHEEL * ROBOT_LENGTH_WHEEL + ROBOT_WIDTH_WHEEL * ROBOT_WIDTH_WHEEL);


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

        // 设置默认值，如果找不到金矿则默认中间点
        /*
        if (goldMinePosition.equals("No")) {
            goldMinePosition = "Center";
        }
        // 设置默认值，如果没有图像识别定位，则使用默认点
        if (startPoint == 0) {
            startPoint = defaultStartPoint;
        }
        */

        // 如果出发点是面向仓库
        /*
        if (startPoint == START_FROM_STORAGE) {

            if (goldMinePosition.equals("Left")) {          // 而且金矿在左侧
                storeLeftPath();
            } else if (goldMinePosition.equals("Right")) {  // 而且金矿在右侧
                storeRightPath();
            } else {                                        // 而且金矿在中键
                storeCenterPath();
            }
        } else {
            // 如果出发点是面向陨石坑
            if (goldMinePosition.equals("Left")) {         // 而且金矿在左侧
                craterLeftPath();
            } else if (goldMinePosition.equals("Right")) { // 而且金矿在右侧
                craterRightPathFull();
            } else {                                        // 而且金矿在中键
                craterCenterPathFull();
            }
        }
        */

        // test


        goDistance(FORWARD_SPEED,15,15);
        turnAngle(35);
        goDistance(FORWARD_SPEED, 25,25);
        goDistance(FORWARD_SPEED,-15,-15);
        turnAngle(35);
        goDistance(FORWARD_SPEED,40,40);
        turnAngle(35);
        goDistance(FORWARD_SPEED,30,30);
        releaseTeamMark();
        goDistance(FORWARD_SPEED, -50,-50);

        telemetry.addData("Status", robot);
        telemetry.update();


        // test on turning
        /*
        for (int i = 0; i < 10; i++){
            turnAngle(90);
            waitForComplete(2);
        }
        */
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

        // 初始化imu角度读取对象
        imuReader.initImu(imu);

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

        // 调整摄像头云台水平伺服的运行区间。目前调整为0.6-1。
        // 此时，如setPosition(0)，代表实际设置值为0.6；如setPosition(0.5)，代表实际设置值为0.8；
        camHori.scaleRange(0.6,1);
        camHori.setPosition(0.5);

        if (Vuforia.isInitialized()) {
            Vuforia.deinit();
        }
    }


    /**
     * 机器人着陆
     */
    private void landing() {
        // 抬起top，防止触地之后被卡住
        topLeft.setPower(-1);
        topRight.setPower(-1);
        waitForComplete(0.5);
        topLeft.setPower(0);
        topRight.setPower(0);

        // 机器人下降到触地
        elevator.setPower(1);
        waitForComplete(3);
        elevator.setPower(0);

        // 开锁扣，由于变更成2段挂车模式，现在开锁之后，机器人有可能会自动掉落到位
        lock.setPosition(LOCK_OPEN);

        // 机器人如果没有滑落，那么再多下降一些保证触地
        elevator.setPower(1);
        waitForComplete(0.5);
        elevator.setPower(0);

        // 现有锁扣的模式，打开之后，直接把抬升装置下降就可以不影响后续动作
        // 所以不需要机器人旋转脱钩了
        elevator.setPower(-1);
        waitForComplete(2);
        elevator.setPower(0);

    }

    /**
     * 检测金矿
     * 左右旋转伺服来寻找金矿
     *
     */
    private void goldMineralDectect() {

        MineralDetector detection = new MineralDetector();
        detection.init(robot);
        detection.start();
        runtimeTemp.reset();
        // 循环移动云台，扫描金矿，不超过5秒
        while (opModeIsActive() && runtimeTemp.seconds() < 5) {

            // 旋转伺服，让摄像头转动一点点
            camScanNextStep();

            // 调用Tensorflow去做金矿识别，
            goldMinePosition = detection.searchGoldMine();
            // 如果返回的不是No，意味着识别到了，就立即退出循环
            if (!"No".equals(goldMinePosition)) {
                break;
            }

            // 让程序等待一会，给伺服转动留时间
            sleep(RobotRoverRuckus.CAM_CYCLE_MS);
            idle();
        }
        detection.stop();
    }

    /**
     * 转动摄像头云台
     */
    private void camScanNextStep() {
        // slew the servo, according to the rampUp (direction) variable.
        if (robot.camScan) {
            // Keep stepping up until we hit the max value.
            robot.setCamPosition(robot.getCamPosition() + 0.02);
            if (robot.getCamPosition() >= 0.95) {
                robot.setCamPosition(1);
                robot.camScan = !robot.camScan;   // Switch ramp direction
            }
        } else {
            // Keep stepping down until we hit the min value.
            robot.setCamPosition(robot.getCamPosition() - 0.02);
            if (robot.getCamPosition() <= 0.05) {
                robot.setCamPosition(0);
                robot.camScan = !robot.camScan;  // Switch ramp direction
            }
        }
        camHori.setPosition(robot.getCamPosition());
    }


    /**
     * 从面对仓库的点出发，走到仓库之后
     * 执行释放团队标志物动作
     */
    private void releaseTeamMark() {
        topMid.setPower(0.5);
        waitForComplete(2);

        servoLeft.setPosition(0.2);
        waitForComplete(0.5);

        topMid.setPower(-1);
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
     * @param speed speed为正左转，speed为负右转
     */
    public void turn(double speed) {
        runModeUsingEncoder();
        powerLeft(-speed);
        powerRight(speed);
    }

    /**
     * @param angle angle为正左转，angle为负右转
     */
    public void turnAngle(double angle) {
        //原有方法，使用编码器进行转向
        //double distance  =0.6 * angle/360 * ROBOT_MOTION_CIRCUM;
        //goDistance(TURN_SPEED,-distance,distance);

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
                           double leftInches, double rightInches) {
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
    private void waitForComplete(double second) {
        runtimeTemp.reset();
        while (opModeIsActive() && runtimeTemp.seconds() < second) {
            idle();
        }
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
        // 重置IMU中的角度，会把当前方向作为0度
        imuReader.resetAngle();

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
        while (opModeIsActive() &&
                Math.abs(degrees - imuReader.getAngle()) > 1 &&
                runtimeTemp.seconds() < 3) {
            // 根据imu角度读数和目标角数值，计算下次调整方向
            int clockwise = (degrees - imuReader.getAngle()) > 0 ? 1 : -1;
            // clockwise为正则向左调整，为负则向右调整
            powerLeft(-power * clockwise);
            powerRight(power * clockwise);

            idle();

            telemetry.addData("target angle:", degrees);
            telemetry.addData("current angle:", imuReader.getAngle());
        }

        // 结束旋转，power设置为0
        power(0);

    }
}
