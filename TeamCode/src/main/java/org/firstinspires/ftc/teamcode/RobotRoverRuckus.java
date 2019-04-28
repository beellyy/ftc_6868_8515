package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 *
 * */
public class RobotRoverRuckus {

    public static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    public static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    public static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double     ROBOT_LENGTH_WHEEL    = 20 ;    // 前后轮的轴距
    public static final double     ROBOT_WIDTH_WHEEL    = 15 ;    // 左右轮的轴距

    public static final double     ROBOT_MOTION_DIAMETER    = Math.sqrt(ROBOT_LENGTH_WHEEL*ROBOT_LENGTH_WHEEL + ROBOT_WIDTH_WHEEL*ROBOT_WIDTH_WHEEL) ;    // 转圈直径
    public static final double     ROBOT_MOTION_CIRCUMFERENCE    = Math.PI * ROBOT_MOTION_DIAMETER ;    // 转圈直径

    public static final double     DRIVE_SPEED             = 0.6;
    public static final double     TURN_SPEED              = 0.5;

    public static final double CAM_INCREMENT = 0.01;     // 每次旋转的偏移量
    public static final int CAM_CYCLE_MS =   50;     // 旋转时间间隔
    public static final double CAM_RIGHT_MAX_POS = 1.0;     // 摄像头右转最大值
    public static final double CAM_LEFT_MAX_POS = 0.0;     // 摄像头左转最大值
    public static final double CAM_SERVO_POS_RANGE = Math.abs(CAM_RIGHT_MAX_POS - CAM_LEFT_MAX_POS);     // 摄像头位置区间
    public static final double CAM_SERVO_ANGLE_MAX = 270;     // 摄像头水平伺服最大角度
    public static final double CAM_SERVO_ANGLE_PER_POS = 270/ CAM_SERVO_POS_RANGE;     // 摄像头水平伺服最大角度


    //摄像头在机器人坐标系上的坐标
    public int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line
    public int CAMERA_FORWARD_DISPLACEMENT  = 100;   // eg: Camera is 110 mm in front of robot center
    public int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground

    //机器人在场地坐标系上的坐标
    public RobotLocation location = new RobotLocation();

    public double camRobotHeadingPos =  0.5;     // 摄像头与机器人对齐时的位置
    //摄像头在场地坐标系的当前角度
    public double camHeading = 0.0;

    //摄像头云台伺服角度与机器人一致时的伺服数值
    public boolean camScan = false;
    private double camPosition = (CAM_RIGHT_MAX_POS - CAM_LEFT_MAX_POS) / 2; // Start at halfway camPosition

    public HardwareMap hardwareMap = null;
    public Telemetry telemetry  = null;

    public RobotRoverRuckus(){}

    public RobotRoverRuckus(HardwareMap map, Telemetry tele){
        this.telemetry = tele;
        this.hardwareMap = map;
    }

    /**
     * 根据定位系统设置camHeading角度
     * 计算计算摄像头伺服角度与机器人角度差
     * 根据角度查修正机器人角度
     * */
    public double robotHeading() {
        //todo: reset robot heading. by
        location.heading = camHeading + CAM_SERVO_ANGLE_MAX*(camPosition - camRobotHeadingPos)/1;
        return location.heading;
    }

    @Override
    public String toString(){
        StringBuilder builder = new StringBuilder();
        if(null!=location){
            builder.append("{x:");
            builder.append(location.x);
            builder.append(",y:");
            builder.append(location.x);
            builder.append(",h:");
            builder.append(location.heading);
            builder.append("t:");
            builder.append(location.targetName);
            builder.append("} ");
        }
        builder.append("{ch:");
        builder.append(camHeading);
        builder.append(",cp:");
        builder.append(getCamPosition());
        builder.append("} ");

        return builder.toString();
    }

    public double getCamPosition() {
        return camPosition;
    }

    public void setCamPosition(double camPosition) {
        this.camPosition = camPosition;
        robotHeading();
    }
}
