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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Basic: Linear OpMode11", group = "Linear Opmode")

public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor0;
    private DcMotor motor1;
    private DcMotor motor3;
    private DcMotor motor2;
    private double curr_power = 0.7;


    public void setupRobot() {


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //MOTOR 3 AND 0 => DIRECTION IS MESSED UP
        // USE FORWARD FOR BACKWARD AND VICE VERSA
        //motor2 is upper left
        //motor3 is upper right
        //motor1 is lower left
        //motor0 is lower right
        motor0 = hardwareMap.get(DcMotor.class, "upperright");
        motor1 = hardwareMap.get(DcMotor.class, "upperleft");
        motor3 = hardwareMap.get(DcMotor.class, "lowerright");
        motor2 = hardwareMap.get(DcMotor.class, "lowerleft");

        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        /*
         Most robots need the motor on one side to be reversed to drive forward
         Reverse the motor that runs backwards when connected directly to the battery
        */


        waitForStart();
        runtime.reset();

//        telemetry.addData("Status", "Success!!");
//        telemetry.update();


    }


    public void setPower_diagonal_left(double power) {
        motor1.setPower(power);
        motor3.setPower(power);
    }

    public void setPower_diagonal_right(double power) {
        motor0.setPower(power);
        motor2.setPower(power);
    }


    public void setPower(double power) {
        motor1.setPower(power);
        motor0.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);
    }

    public void runOpMode() {


        setupRobot();
        waitForStart();

        // Wait for the game to start (driver presses PLAY)


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
//            double leftPower;
//            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
//            leftPower = -gamepad1.left_stick_y;
//            rightPower = gamepad1.right_stick_x;
            //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            if (gamepad1.left_stick_x < -0.5) {
                //left
                motor0.setDirection(DcMotorSimple.Direction.FORWARD);
                motor1.setDirection(DcMotorSimple.Direction.FORWARD);
                motor3.setDirection(DcMotorSimple.Direction.REVERSE);
                motor2.setDirection(DcMotorSimple.Direction.REVERSE);
                setPower(curr_power);
            } else if (gamepad1.left_stick_x > 0.5) {//&& gamepad1.left_stick_y > -0.5 && gamepad1.left_stick_y <0.5)  {
                //x = right
                motor0.setDirection(DcMotorSimple.Direction.REVERSE);
                motor1.setDirection(DcMotorSimple.Direction.REVERSE);
                motor3.setDirection(DcMotorSimple.Direction.FORWARD);
                motor2.setDirection(DcMotorSimple.Direction.FORWARD);
                setPower(curr_power);
            } else if (gamepad1.left_stick_y > 0.5) { /*
                && gamepad1.left_stick_x > -0.5 && gamepad1.left_stick_x < 0.5) {
                x = up if x
                */
                motor0.setDirection(DcMotorSimple.Direction.REVERSE);
                motor1.setDirection(DcMotorSimple.Direction.FORWARD);
                motor3.setDirection(DcMotorSimple.Direction.REVERSE);
                motor2.setDirection(DcMotorSimple.Direction.FORWARD);
                setPower(curr_power);
            } else if (gamepad1.left_stick_y < -0.5) { // && gamepad1.left_stick_x > -0.5 && gamepad1.left_stick_x >0.5)  {
                //x = down if x
                motor1.setDirection(DcMotorSimple.Direction.REVERSE);
                motor0.setDirection(DcMotorSimple.Direction.FORWARD);
                motor3.setDirection(DcMotorSimple.Direction.FORWARD);
                motor2.setDirection(DcMotorSimple.Direction.REVERSE);
                setPower(curr_power);

            } else if (gamepad1.right_stick_x < -0.3 && gamepad1.right_stick_y < -0.3) {
                //lower left
                motor0.setDirection(DcMotorSimple.Direction.FORWARD);
                motor2.setDirection(DcMotorSimple.Direction.REVERSE);
                setPower_diagonal_right(curr_power);
            } else if (gamepad1.right_stick_x < -0.3 && gamepad1.right_stick_y > 0.3) {
                //upper left
                motor1.setDirection(DcMotorSimple.Direction.FORWARD);
                motor3.setDirection(DcMotorSimple.Direction.REVERSE);
                setPower_diagonal_left(curr_power);
            } else if (gamepad1.right_stick_x > 0.3 && gamepad1.right_stick_y > 0.3) {
                // upper right.
                motor0.setDirection(DcMotorSimple.Direction.REVERSE);
                motor2.setDirection(DcMotorSimple.Direction.FORWARD);
                setPower_diagonal_right(curr_power);

            } else if (gamepad1.right_stick_x > 0.3 && gamepad1.right_stick_y < -0.3) {
                //lower right.
                motor1.setDirection(DcMotorSimple.Direction.REVERSE);
                motor3.setDirection(DcMotorSimple.Direction.FORWARD);
                setPower_diagonal_left(curr_power);
            }else if (gamepad1.x) {
                //left turn
                motor0.setDirection(DcMotorSimple.Direction.REVERSE);
                motor1.setDirection(DcMotorSimple.Direction.REVERSE);
                motor2.setDirection(DcMotorSimple.Direction.REVERSE);
                motor3.setDirection(DcMotorSimple.Direction.REVERSE);
                setPower_diagonal_left(curr_power);
                setPower_diagonal_right(curr_power);
            }else if (gamepad1.b){
                //right turn
                motor0.setDirection(DcMotorSimple.Direction.FORWARD);
                motor1.setDirection(DcMotorSimple.Direction.FORWARD);
                motor2.setDirection(DcMotorSimple.Direction.FORWARD);
                motor3.setDirection(DcMotorSimple.Direction.FORWARD);
                setPower_diagonal_right(curr_power);
                setPower_diagonal_left(curr_power);

            }else if (gamepad1.right_bumper){
                //normal mode
             curr_power=0.7;

            }else if(gamepad1.left_bumper){
                //slow mode
                curr_power=0.2;

            }





          /*
          else if(gamepad1.b == true && gamepad1.x = false){


            }
            }

           */
            else {

                if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 ) {
                    setPower(0);

                    motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    sleep(10);


                }

            }


            // Show the elapsed game time and wheel power.
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
//            telemetry.update();
        }
    }
}


