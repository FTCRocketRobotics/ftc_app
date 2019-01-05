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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;


/**
*hi
*/

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="yeet mobile", group="Titans Linear Opmode")
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor latcherFar = null;
    private DcMotor latcherClose = null;
    private DcMotor armyDude = null;
    private DcMotor armyOff = null;
    private Servo mrclawPants = null;
    private Servo mrsclawPants = null;

    private double leftPower; // left drive motor
    private double rightPower; // right drive motor
    private double leftTriggerPower; // latch
    private double rightTriggerPower; // latch
    private double armyDudePower; // arm
    private double armyOffPower; // arm
    private double mrclawPos; // claw
    private double mrsclawPos; // claw

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initRobot();
        waitForStart();
        runtime.reset();

// Setup a variable for each drive wheel to save power level for telemetry

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            /*
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            */
            //Tank mode

            if (gamepad2.a) {
                openClaw();
            }
            //else if (gamepad1.b){
            //    closeClaw();
            //}
            else {
                closeClaw();
            }

            leftPower    = Range.clip(gamepad1.left_stick_y, -1.0, 1.0) ;
            rightPower   = Range.clip(gamepad1.right_stick_y, -1.0, 1.0) ;
            leftTriggerPower  = Range.clip(latcherClose(), -1.0, 1.0);
            rightTriggerPower  = Range.clip(latcherFar(), -1.0, 1.0);
            armyDudePower  = Range.clip(gamepad2.left_stick_x, -0.5, 0.5);
            armyOffPower  = Range.clip(gamepad2.right_stick_x, -0.5, 0.5);

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            latcherFar.setPower(leftTriggerPower);
            latcherClose.setPower(rightTriggerPower);
            armyDude.setPower(armyDudePower);
            armyOff.setPower(armyOffPower);
            teleUpdate();
        }
    }

    private void initRobot(){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //mrsclawPants doesn't exist.
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        latcherFar = hardwareMap.get (DcMotor.class, "latcher_far");
        latcherClose = hardwareMap.get (DcMotor.class, "latcher_close");
        armyDude = hardwareMap.get (DcMotor.class, "army_dude");
        armyOff = hardwareMap.get (DcMotor.class, "army_off");
        mrclawPants = hardwareMap.get (Servo.class, "mrclaw_pants");
        mrsclawPants = hardwareMap.get (Servo.class, "mrsclaw_pants");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        latcherFar.setDirection(DcMotor.Direction.FORWARD);
        latcherClose.setDirection(DcMotor.Direction.REVERSE);
        armyDude.setDirection(DcMotor.Direction.FORWARD);
        armyOff.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        latcherFar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        latcherClose.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armyDude.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armyOff.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);// Wait for the game to start (driver presses PLAY)
    }

    private void openClaw(){
        mrclawPos = 1.0;
        mrsclawPos = 0.0;
        mrclawPants.setPosition(mrclawPos);
        mrsclawPants.setPosition(mrsclawPos);
    }

    private void closeClaw(){
        mrclawPos = 0.0;
        mrsclawPos = 1.0;
        mrclawPants.setPosition(mrclawPos);
        mrsclawPants.setPosition(mrsclawPos);
    }

    double latcherClose() {
        double ltPower;
        // latch close
        if (gamepad2.dpad_up) {
            ltPower = 1.0;
        }
        else if (gamepad2.dpad_down){
            ltPower = -1.0;
        }
        else {
            ltPower = 0.0;
        }
        return ltPower;
    }

    double latcherFar() {
        double rtPower;
        if (gamepad2.dpad_left) {
            rtPower = 1.0;
        }
        else if (gamepad2.dpad_right) {
            rtPower = -1.0;
        }
        else {
            rtPower = 0.0;
        }
        return rtPower;
    }

    private void teleUpdate(){
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Motors", "latch far (%.2f), latch close (%.2f)", leftTriggerPower, rightTriggerPower);
        telemetry.addData("Motors","arm far (%.2f), arm close (%.2f)", armyDudePower, armyOffPower);
        telemetry.addData("Motors", "left claw (%.2f), right claw (%.2f)", mrclawPos, mrsclawPos);
        // TODO: need to add flag servo data
        telemetry.addData("voltage", "%.1f volts", new Func<Double>() {
            @Override public Double value() {
                return getBatteryVoltage();
            }
        });
        telemetry.update();
    }

    // Computes the current battery voltage
    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

}
