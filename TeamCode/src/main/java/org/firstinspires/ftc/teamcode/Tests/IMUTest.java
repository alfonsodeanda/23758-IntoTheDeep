package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Autonomous (name = "IMUtest")
public class IMUTest extends LinearOpMode {
    private DcMotor DelanteIz, DelanteDe, AtrasIz, AtrasDe;

    public IMU imu;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addLine("Prueba de uso del girscopio");
        telemetry.update();

        DelanteIz = hardwareMap.get(DcMotor.class, "DelanteIz");
        DelanteDe = hardwareMap.get(DcMotor.class, "DelanteDe");
        AtrasIz  = hardwareMap.get(DcMotor.class, "AtrasIz");
        AtrasDe = hardwareMap.get(DcMotor.class, "AtrasDe");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                        )
                )
        );

        DelanteIz.setDirection(DcMotor.Direction.REVERSE);
        DelanteDe.setDirection(DcMotor.Direction.FORWARD);
        AtrasIz.setDirection(DcMotor.Direction.REVERSE);
        AtrasDe.setDirection(DcMotor.Direction.FORWARD);

        DelanteIz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DelanteDe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AtrasIz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AtrasDe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()){
            motorsSetPower(-0.3, 0.3, -0.3, 0.3);

            telemetry.addData("Angle: ", getAngle());
            telemetry.update();
        }
    }

    public void motorsSetPower (double powDeIz, double powDeDe, double powAtIz, double powAtDe) {
        DelanteIz.setPower(powDeIz);
        DelanteDe.setPower(powDeDe);
        AtrasIz.setPower(powAtIz);
        AtrasDe.setPower(powAtDe);
    }

    public double getAngle () {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

}
