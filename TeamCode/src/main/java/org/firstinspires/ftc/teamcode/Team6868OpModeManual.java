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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name = "Manual6868", group = "manual")
@Disabled
public class Team6868OpModeManual extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor elevator = null;
    private DcMotor topLeft = null;
    private DcMotor topRight = null;
    private DcMotor topMid = null;
    private Servo servoLeft = null;
    private Servo servoRight = null;

    // test
    private Servo camHori = null;
    protected Servo lock = null;

    private ElapsedTime clawLeftTimer = new ElapsedTime();
    private ElapsedTime clawRightTimer = new ElapsedTime();
    static final double     LOCK_OPEN = 0;
    static final double     LOCK_CLOSE = 1;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack  = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        topLeft  = hardwareMap.get(DcMotor.class, "top_left");
        topRight = hardwareMap.get(DcMotor.class, "top_right");
        topMid = hardwareMap.get(DcMotor.class, "top_middle");
        servoLeft = hardwareMap.get(Servo.class, "servo_left");
        servoRight = hardwareMap.get(Servo.class, "servo_right");

        // test
        camHori = hardwareMap.get(Servo.class, "camera_horizontal");
        lock = hardwareMap.get(Servo.class, "lock");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        elevator.setDirection(DcMotor.Direction.FORWARD);
        topLeft.setDirection(DcMotor.Direction.REVERSE);
        topRight.setDirection(DcMotor.Direction.FORWARD);
        topMid.setDirection(DcMotor.Direction.FORWARD);
        servoLeft.setDirection(Servo.Direction.FORWARD);
        servoRight.setDirection(Servo.Direction.FORWARD);

        topMid.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lock.scaleRange(0.6,0.9);
        lock.setPosition(LOCK_OPEN);

        camHori.scaleRange(0.6,1);
        camHori.setPosition(0.5);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // test
        double servoPositionLeft = 0.3;
        double servoPositionRight = 0.8;

        int countLeft = 0;
        int countRight = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double elevatePower = 0.0;
            double topPower = 0.0;
            double topMidPower = 0.0;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

            leftPower = 0.7 * Range.clip( (drive + turn), -1.0, 1.0) ;
            rightPower = 0.7 * Range.clip((drive - turn), -1.0, 1.0) ;

            // elevator
            if (gamepad1.left_bumper){
                elevatePower = 1;
            } else if (gamepad1.right_bumper){
                elevatePower = -1;
            }

            if (gamepad2.left_bumper){//上
                topPower = -1;
            } else if (gamepad2.right_bumper){//下
                topPower = 1;
            }

            if (gamepad2.left_trigger > 0.5){
                topMidPower = 0.4;
            } else if (gamepad2.right_trigger > 0.5){
                topMidPower = -0.4;
            }

            if (gamepad2.dpad_right && clawRightTimer.seconds()>0.5) {
                if (countRight % 2 == 0) {
                    servoPositionRight = 0.0;
                } else {
                    servoPositionRight = 0.8;
                }
                countRight++;
                clawRightTimer.reset();
            }
            if (gamepad2.dpad_left && clawLeftTimer.seconds()>0.5){
                if (countLeft % 2 == 0) {
                    servoPositionLeft = 0.9;
                } else {
                    servoPositionLeft = 0.3;
                }
                countLeft++;
                clawLeftTimer.reset();
            }

            // test
            if (gamepad1.dpad_left){
                camHori.setPosition(camHori.getPosition()-0.1);
            } else if (gamepad1.dpad_right){
                camHori.setPosition(camHori.getPosition()+0.1);
            } else if (gamepad1.dpad_down){
                lock.setPosition(lock.getPosition()+0.1);
            } else if (gamepad1.dpad_up){
                lock.setPosition(lock.getPosition()-0.1);
            }

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftFront.setPower(leftPower);
            leftBack.setPower(leftPower);

            rightFront.setPower(rightPower);
            rightBack.setPower(rightPower);

            elevator.setPower(elevatePower);

            topLeft.setPower(topPower);
            topRight.setPower(topPower);

            topMid.setPower(topMidPower);

            servoLeft.setPosition(servoPositionLeft);
            servoRight.setPosition(servoPositionRight);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), elevate (%.2f), "
                    , leftPower, rightPower, elevatePower);
            telemetry.update();
        }
    }
}
