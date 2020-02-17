package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static android.content.Context.SENSOR_SERVICE;

@TeleOp(name = "Test Robot", group = "Linear Opmode")
public class Test_Robot extends LinearOpMode {


    DcMotor R0, R2, L1, L3, CE1, CE2, SCM;
    Servo FNM, CM;
    BNO055IMU imu;
    Orientation angle;


    private ElapsedTime timer = new ElapsedTime();
    float power = (float) 0.8;
    private double counter = 0;
    private double increment = 0.09;


    public void setup() {

        //set gyro setting
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode=BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!imu.isGyroCalibrated()) {
            telemetry.addData("calibrating imu","...");
            telemetry.update();
            sleep(50);
            idle();
        }


        R0 = hardwareMap.get(DcMotor.class, "R0");
        R2 = hardwareMap.get(DcMotor.class, "R2");
        L1 = hardwareMap.get(DcMotor.class, "L1");
        L3 = hardwareMap.get(DcMotor.class, "L3");
        CE1 = hardwareMap.get(DcMotor.class, "CE1");//cube eater 1 (right)
        CE2 = hardwareMap.get(DcMotor.class, "CE2");//cube eater 2 (left)
        SCM = hardwareMap.get(DcMotor.class, "SCM"); //seazer motor (misparaim)
        CM = hardwareMap.get(Servo.class, "CM");


        R0.setDirection(DcMotor.Direction.FORWARD);
        R2.setDirection(DcMotor.Direction.FORWARD);
        L1.setDirection(DcMotor.Direction.FORWARD);
        L3.setDirection(DcMotor.Direction.FORWARD);
        CE1.setDirection(DcMotor.Direction.FORWARD);
        CE2.setDirection(DcMotor.Direction.FORWARD);
        SCM.setDirection(DcMotor.Direction.FORWARD);
        CM.setDirection(Servo.Direction.FORWARD);


        telemetry.addData("Stauts", "success!");
        telemetry.update();
    }

    public void drive(int dir, int cm, double power) {

        int target; // the original ticks of the motor ( see datasheet )
        int target_position; //the final position ( ticks )
        int error = 0; //the original ticks  divide by 3 ( 1993/3 = 665 )
        int error2; // the target position - original one circle 1328 ( 1993 -665 )
//        double power = 0.2; //power
        double acceleration = 0.01; // by how much the motor accelerate!
        target = cm * 45;//by the datasheet ( see goBilda web Motor 84 RPM )
        //45 ticks is 1 cm

        //set the direction of the motor
        // 1 --> FWD
        // 2 --> BWD
        //3 --> RIGHT
        //4 --> LEFT
        //5--> ROTATE LEFT
        //6--> ROTATE RIGHT
        switch (dir) {
            case 1: {
                L3.setDirection(DcMotor.Direction.REVERSE);
                L1.setDirection(DcMotor.Direction.FORWARD);
                R2.setDirection(DcMotor.Direction.REVERSE);
                R0.setDirection(DcMotor.Direction.FORWARD);
                break;
            }
            case 2: {
                L3.setDirection(DcMotor.Direction.FORWARD);
                L1.setDirection(DcMotor.Direction.REVERSE);
                R2.setDirection(DcMotor.Direction.FORWARD);
                R0.setDirection(DcMotor.Direction.REVERSE);
                break;
            }
            case 3: {
                L3.setDirection(DcMotor.Direction.FORWARD);
                L1.setDirection(DcMotor.Direction.REVERSE);
                R2.setDirection(DcMotor.Direction.REVERSE);
                R0.setDirection(DcMotor.Direction.FORWARD);
                break;
            }
            case 4: {
                L3.setDirection(DcMotor.Direction.REVERSE);
                L1.setDirection(DcMotor.Direction.FORWARD);
                R2.setDirection(DcMotor.Direction.FORWARD);
                R0.setDirection(DcMotor.Direction.REVERSE);
                break;

            }

            case 5: {
                R0.setDirection(DcMotor.Direction.REVERSE);
                R2.setDirection(DcMotor.Direction.REVERSE);
                L1.setDirection(DcMotor.Direction.REVERSE);
                L3.setDirection(DcMotor.Direction.REVERSE);
                break;
            }
            case 6: {
                R0.setDirection(DcMotor.Direction.FORWARD);
                R2.setDirection(DcMotor.Direction.FORWARD);
                L1.setDirection(DcMotor.Direction.FORWARD);
                L3.setDirection(DcMotor.Direction.FORWARD);
                break;
            }
        }

        //the mode of the motor
        //After using enncoder - do stop and reset
        L3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //mesure the encoder
        L3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //when power is 0 - do a breakes!
        L3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        R2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        R0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //supply power from the battery
        L3.setPower(power);
        L1.setPower(power);
        R2.setPower(power);
        R0.setPower(power);

        //calc the error - by ticks
        target_position = (L3.getCurrentPosition() + target) - error;
        // be safe with abs value
        //  error2 = Math.abs(target_position - 1328);
        // target_position = Math.abs(target_position - error2);


        //run to position
        while (opModeIsActive() && (L3.getCurrentPosition() < target_position)) {


            //acceleration
            power = power + acceleration;
            if (power >= 1) {
                //dont over 1!
                // if the power is 1 --> keep it 1
                power = 1;
            }

            //keep update the power
            R2.setPower(power);
            L3.setPower(power);
            L1.setPower(power);
            R0.setPower(power);

            telemetry.addData("R2 ticks = ", R0.getCurrentPosition());
            telemetry.addData("R2 ticks = ", R2.getCurrentPosition());
            telemetry.addData("R2 ticks = ", L1.getCurrentPosition());
            telemetry.addData("R2 ticks = ", L3.getCurrentPosition());
            telemetry.update();
        }

        //stop!
        L3.setPower(0);
        L1.setPower(0);
        R2.setPower(0);
        R0.setPower(0);

    }

    void initMotor() {
        //the mode of the motor
        //After using enncoder - do stop and reset
        L3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //mesure the encoder
        L3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //when power is 0 - do a breakes!
        L3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        R2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        R0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void testRobot() {


        double pos_R0, pos_R2, pos_L1, pos_L3;
        double lastPos;

        initMotor();


        //set robot drive fwd

        L3.setDirection(DcMotor.Direction.REVERSE);
        L1.setDirection(DcMotor.Direction.FORWARD);
        R2.setDirection(DcMotor.Direction.REVERSE);
        R0.setDirection(DcMotor.Direction.FORWARD);


        R0.setPower(0.9);
        sleep(1000);
        R0.setPower(0);
        pos_R0 = R0.getCurrentPosition();

        R2.setPower(0.9);
        sleep(1000);
        R2.setPower(0);
        pos_R2 = R2.getCurrentPosition();

        L1.setPower(0.9);
        sleep(1000);
        L1.setPower(0);
        pos_L1 = L1.getCurrentPosition();

        L3.setPower(0.9);
        sleep(1000);
        L3.setPower(0);
        pos_L3 = L3.getCurrentPosition();

        telemetry.addData("postion R0 = ", pos_R0);
        telemetry.addLine("=========");
        telemetry.addData("position R2 = ", pos_R2);
        telemetry.addLine("=========");
        telemetry.addData("position L1 = ", pos_L1);
        telemetry.addLine("=========");
        telemetry.addData("position L3 = ", pos_L3);
        telemetry.addLine("=========");
        telemetry.addLine();

        telemetry.update();
        sleep(15000);


    }

    void goFwdTest(double cm, double power) {
        double target_position;
        double target;
        double pos_R0, pos_R2, pos_L1, pos_L3;
        double error = 0.3;
        initMotor();

        //set drive fwd
        // L3.setDirection(DcMotor.Direction.REVERSE);
        // L1.setDirection(DcMotor.Direction.FORWARD);
        // R2.setDirection(DcMotor.Direction.REVERSE);
        // R0.setDirection(DcMotor.Direction.FORWARD);


        int target_postion = 7000;
        R0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        target_postion = R0.getCurrentPosition() + target_postion;
        target_postion = R2.getCurrentPosition() + target_postion;
        target_postion = L1.getCurrentPosition() + target_postion;
        target_postion = L3.getCurrentPosition() + target_postion;

        R0.setTargetPosition(target_postion);
        R2.setTargetPosition(target_postion);
        L1.setTargetPosition(target_postion);
        L3.setTargetPosition(target_postion);

        R0.setPower(-1);
        R2.setPower(1);
        L1.setPower(-1);
        L3.setPower(1);

        R0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        R2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L3.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (R0.isBusy() && R2.isBusy() && L1.isBusy() && L3.isBusy()) {
            if (Math.abs(R0.getCurrentPosition()) >= target_postion) {
                R0.setPower(0);
                R2.setPower(0);
                L1.setPower(0);
                L3.setPower(0);
            }
            if (Math.abs(R2.getCurrentPosition()) >= target_postion) {
                R0.setPower(0);
                R2.setPower(0);
                L1.setPower(0);
                L3.setPower(0);
            }
            if (Math.abs(L1.getCurrentPosition()) >= target_postion) {
                R0.setPower(0);
                R2.setPower(0);
                L1.setPower(0);
                L3.setPower(0);
            }
            if (Math.abs(L3.getCurrentPosition()) >= target_postion) {
                R0.setPower(0);
                R2.setPower(0);
                L1.setPower(0);
                L3.setPower(0);
            }
        }
        R0.setPower(0);
        R2.setPower(0);
        L1.setPower(0);
        L3.setPower(0);


        //target = cm * 45;
        //target_position = (L3.getCurrentPosition() + target);
        /*
        R0.setPower(power);
        R2.setPower(power);
        L3.setPower(power);
        L1.setPower(power);


        while (opModeIsActive()) {

            pos_R0 = R0.getCurrentPosition();
            pos_R2 = R2.getCurrentPosition();
            pos_L1 = L1.getCurrentPosition();
            pos_L3 = L3.getCurrentPosition();

            if (pos_R0 > pos_R2) {
                R2.setPower(power + error);
                R0.setPower(power - error);
            } else {
                R2.setPower(power - error);
                R0.setPower(power + error);
            }

            if (pos_L1 > pos_L3) {
                L3.setPower(power + error);
                L1.setPower(power - error);
            } else {
                L3.setPower(power - error);
                L1.setPower(power + error);
            }
        }
        */
    }

    @Override
    public void runOpMode() throws InterruptedException {


        double power = 0.6;
        double error = 0;
        double kp = 1.2;
        double ang;
        //init0 mode
        setup();
        waitForStart();
        // testRobot();
        // goFwdTest(1, 0.5);

        R0.setDirection(DcMotor.Direction.FORWARD);
        R2.setDirection(DcMotor.Direction.REVERSE);
        L1.setDirection(DcMotor.Direction.FORWARD);
        L3.setDirection(DcMotor.Direction.REVERSE);

        R0.setPower(0.9);
        R2.setPower(0.9);
        L1.setPower(0.9);
        L3.setPower(0.9);


        while (opModeIsActive()) {

//            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Heading", angle.firstAngle);
//            telemetry.addData("Roll", angle.secondAngle);
//            telemetry.addData("Pitch", angle.thirdAngle);
            telemetry.update();


            error = angle.firstAngle / 10;
            ang = error;
            error = error * kp;

            //right
//            telemetry.addData("angle",angle.firstAngle);
//            telemetry.update();

            double curr_angle=angle.firstAngle;
            if (curr_angle<0) {
                curr_angle=curr_angle*-1;
            }
            double curr_pos_power=power+(curr_angle / 10);
            if (curr_pos_power>=1) {
                curr_pos_power=1;
            }
            double curr_minus_power=power-((curr_angle / 10));
            if (curr_minus_power<=0) {
                curr_minus_power=0.2;
            }
//            if(angle.firstAngle > 0){
//
//                R0.setPower(curr_minus_power);
//                R2.setPower(curr_pos_power);
//                L3.setPower(curr_pos_power);
//                L1.setPower(curr_minus_power);
//            }
//            if (angle.firstAngle < 0){
//                R0.setPower(curr_pos_power);
//                R2.setPower(curr_minus_power);
//                L3.setPower(curr_minus_power);
//                L1.setPower(curr_pos_power);
//            }
        }
        sleep(1000);


    }
}
