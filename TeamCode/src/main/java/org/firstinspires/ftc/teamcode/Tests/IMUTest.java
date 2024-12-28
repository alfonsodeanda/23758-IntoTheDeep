package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous (name = "IMUtest")
public class IMUTest extends LinearOpMode {
    private DcMotor delanteIz, delanteDe, atrasIz, atrasDe;

    public IMU imu;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addLine("Prueba de uso del girscopio");
        telemetry.update();

        delanteIz = hardwareMap.get(DcMotor.class, "delanteIz");
        delanteDe = hardwareMap.get(DcMotor.class, "delanteDe");
        atrasIz  = hardwareMap.get(DcMotor.class, "atrasIz");
        atrasDe = hardwareMap.get(DcMotor.class, "atrasDe");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            new Orientation(
                                    AxesReference.INTRINSIC,
                                    AxesOrder.ZYX,
                                    AngleUnit.DEGREES,
                                    0,
                                    0,
                                    60,
                                    0
                            )
                    )
                )
        );

        delanteIz.setDirection(DcMotor.Direction.REVERSE);
        delanteDe.setDirection(DcMotor.Direction.FORWARD);
        atrasIz.setDirection(DcMotor.Direction.REVERSE);
        atrasDe.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();

        delanteIz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        delanteDe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        atrasIz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        atrasDe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (opModeIsActive()){
            turnToAngle(90, 0.6);
        }
    }

    public void resetEncoders() {
        delanteIz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        delanteDe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        atrasIz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        atrasDe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        delanteIz.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        delanteDe.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        atrasIz.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        atrasDe.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stopMotors() {
        motorsSetPower(0, 0, 0, 0);
    }

    public void motorsSetPower (double powDeIz, double powDeDe, double powAtIz, double powAtDe) {
        delanteIz.setPower(powDeIz);
        delanteDe.setPower(powDeDe);
        atrasIz.setPower(powAtIz);
        atrasDe.setPower(powAtDe);
    }

    public double currentAngle () {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void turnToAngle (double targetAngle, double power) {
        while (opModeIsActive() && Math.abs(targetAngle - currentAngle()) > 1) {
            if (targetAngle > currentAngle()) {
                motorsSetPower(power, -power, power, -power);
            } else {
                motorsSetPower(-power,  power, -power, power);
            }
        }
        stopMotors();
    }

}
