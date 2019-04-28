package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotBao extends RobotRoverRuckus {

    public static final double     CMM_PER_INCH    = 2.54 ;    // in inch 宽度
    public static final double     ROBOT_WIDTH    = 23 / CMM_PER_INCH ;    // in inch 宽度
    public static final double     ROBOT_LENGTH    = 30 / CMM_PER_INCH ;    // in inch 长度
    public static final double     ROBOT_MOTION_DIM    = Math.sqrt(ROBOT_WIDTH*ROBOT_WIDTH+ROBOT_LENGTH*ROBOT_LENGTH) ;    // 运动直径
    public static final double     ROBOT_MOTION_CIRCUM = Math.PI * ROBOT_MOTION_DIM ;    // 运动周长

    public static final double     COUNTS_PER_MOTOR_REV    = 1680;// tetrix:1440;andymark40:1120;andymark60:1680
    public static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    public static final double     WHEEL_DIAMETER_INCHES   = 9/CMM_PER_INCH ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                             (WHEEL_DIAMETER_INCHES * Math.PI);

    public static final double     INCH_PER_SECOND    = 20;//
    public static final double     FORWARD_SPEED = 0.6;
    public static final double     FORWARD_SLOW_SPEED = 0.4;
    public static final double     TURN_SPEED    = 0.5;

    private double clawOpen = 1;
    private double clawClose = 0;


    public DcMotor driveLeft1 = null;
    public DcMotor driveLeft2 = null;
    public DcMotor driveRight1 = null;
    public DcMotor driveRight2 = null;


    /**
     * 平台伸缩 -后退 +前进
     */
    public DcMotor motorPlatform = null;

    /**
     * hook -打开 +关闭   -下 +上
     */
    public DcMotor motorWristHook = null;

    /**
     * 支持爪子和钩子的机械臂 -上升 +下降
     */
    public DcMotor motorArm = null;

    /**
     * 抬升马达 -上升 +下降
     */
    public DcMotor motorLift = null;

    //车身周边安装的测距传感器，识别距离大约十几厘米
    public DistanceSensor sdFront  = null;
    public DistanceSensor sdBack  = null;
    public DistanceSensor sdLeftFront = null;
    public DistanceSensor sdLeftRear = null;
    public DistanceSensor sdRightFront = null;
    public DistanceSensor sdRightRear = null;

    public Servo claw1 = null;
    public Servo claw2 = null;

    public boolean claw1Opened = false;
    public boolean claw2Opened = false;

    public RobotBao(){}

    public RobotBao(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap,telemetry);
    }

    public void init(HardwareMap hardwareMap){
        driveLeft1  = hardwareMap.get(DcMotor.class, "lf");
        driveLeft2 = hardwareMap.get(DcMotor.class, "lr");
        driveRight1  = hardwareMap.get(DcMotor.class, "rf");
        driveRight2 = hardwareMap.get(DcMotor.class, "rr");
        motorPlatform = hardwareMap.get(DcMotor.class, "mxy");
        motorWristHook = hardwareMap.get(DcMotor.class, "mab");
        motorArm = hardwareMap.get(DcMotor.class, "mdpadud");
        motorLift = hardwareMap.get(DcMotor.class, "mdpadlr");

        sdFront = hardwareMap.get(DistanceSensor.class, "sdf");
        sdBack = hardwareMap.get(DistanceSensor.class, "sdb");
        sdLeftFront = hardwareMap.get(DistanceSensor.class, "sdlf");
        sdLeftRear = hardwareMap.get(DistanceSensor.class, "sdlr");
        sdRightFront = hardwareMap.get(DistanceSensor.class, "sdrf");
        sdRightRear = hardwareMap.get(DistanceSensor.class, "sdrr");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        driveLeft1.setDirection(DcMotor.Direction.FORWARD);
        driveLeft2.setDirection(DcMotor.Direction.FORWARD);
        driveRight1.setDirection(DcMotor.Direction.REVERSE);
        driveRight2.setDirection(DcMotor.Direction.REVERSE);

        driveModeUsingEncoder();

        motorPlatform.setDirection(DcMotor.Direction.FORWARD);
        motorWristHook.setDirection(DcMotor.Direction.FORWARD);
        motorArm.setDirection(DcMotor.Direction.FORWARD);
        motorLift.setDirection(DcMotor.Direction.FORWARD);

        motorWristHook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorPlatform.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorWristHook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        claw1 = hardwareMap.get(Servo.class, "c1");
        claw2 = hardwareMap.get(Servo.class, "c2");
        claw2.setDirection(Servo.Direction.REVERSE);
        claw1.scaleRange(0,1);
        claw2.scaleRange(0,1);
        claw1.setPosition(0.5);
        claw2.setPosition(0.5);
    }

    private void driveMode(DcMotor.RunMode runmode) {
        driveLeft1.setMode(runmode);
        driveLeft2.setMode(runmode);
        driveRight1.setMode(runmode);
        driveRight2.setMode(runmode);
    }

    private void driveModeUsingEncoder() {
        driveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void go(double leftPower, double rightPower) {
        driveLeft1.setPower(leftPower);
        driveLeft2.setPower(leftPower);
        driveRight1.setPower(rightPower);
        driveRight2.setPower(rightPower);
    }

    public void go(double power) {
        go(power,power);
    }

    public void turn(double power){
        go(-power,power);
    }

    public void stop() {
        go(0,0);
    }


    /**
     * 爪子打开
     */
    public void clawOpen() {
        claw1.setPosition(clawOpen);
        claw1Opened = true;
        claw2.setPosition(clawOpen);
        claw2Opened = true;
    }

    /**
     * 爪子1切换状态
     */
    public void claw1Switch() {
        if(claw1Opened){
            claw1.setPosition(clawClose);
        }else {
            claw1.setPosition(clawOpen);
        }
        claw1Opened = !claw1Opened;
    }

    /**
     * 爪子2切换状态
     */
    public void claw2Switch() {
        if(claw2Opened){
            claw2.setPosition(clawClose);
        }else {
            claw2.setPosition(clawOpen);
        }
        claw2Opened = !claw2Opened;
    }

    /**
     * 爪子全部切换状态
     */
    public void clawSwitch() {
        claw1Switch();
        claw2Switch();
    }

    /**
     * 钩子关闭
     */
    public void hookClose(){
        motorWristHook.setPower(1);
    }

    /**
     * 钩子打开
     */
    public void hookOpen(){
        motorWristHook.setPower(-1);

    }

    /**
     * 钩子马达停止
     */
    public void hookStop() {
        motorWristHook.setPower(0);
    }

    /**
     * 抬升马达升起
     */
    public void liftUp(){
        motorLift.setPower(-1);
    }

    /**
     * 抬升马达下降
     */
    public void liftDown(){
        motorLift.setPower(1);
    }

    /**
     * 抬升马达停止工作
     */
    public void liftStop(){
        motorLift.setPower(0);
    }

    /**
     * 机械臂抬升
     */
    public void armUp(){
        motorArm.setPower(-1);
    }

    /**
     * 机械臂放下
     */
    public void armDown(){
        motorArm.setPower(1);
    }

    /**
     * 机械臂马达停止工作
     *
     */
    public void armStop(){
        motorArm.setPower(0);
    }


}
