package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "PID")
public class PIDForwardTest extends LinearOpMode {
    private DcMotor delanteIz, delanteDe, atrasIz, atrasDe;

    private static final double cmByTick = 0.053855874;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addLine("Prueba de control PID para ir adelante de forma precisa");
        telemetry.update();

        delanteIz = hardwareMap.get(DcMotor.class, "delanteIz");
        delanteDe = hardwareMap.get(DcMotor.class, "delanteDe");
        atrasIz  = hardwareMap.get(DcMotor.class, "atrasIz");
        atrasDe = hardwareMap.get(DcMotor.class, "atrasDe");

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
            PIDForward(50, 0.6);
        }
    }

    public void stopMotors() {
        motorsSetPower(0, 0, 0, 0);
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

    public void motorsSetPower (double powDeIz, double powDeDe, double powAtIz, double powAtDe) {
        delanteIz.setPower(powDeIz);
        delanteDe.setPower(powDeDe);
        atrasIz.setPower(powAtIz);
        atrasDe.setPower(powAtDe);
    }

    public int encoderAverage () {
        return (delanteIz.getCurrentPosition() + delanteDe.getCurrentPosition() +
                atrasIz.getCurrentPosition() + atrasDe.getCurrentPosition()) / 4;
    }

    public void PIDForward (double cm, double maxPower) {
        resetEncoders();
        int targetPos = (int) (cm/cmByTick);

        double integral = 0;
        double lastError = 0;

        double kP = 0.01;
        double kI = 0;
        double kD = 0;

        while (opModeIsActive()){
            int currentPos = encoderAverage();

            double error = targetPos - currentPos;
            integral += error;
            double derivative = error - lastError;

            double power = (kP * error) + (kI * integral) + (kD * derivative);
            power = Math.max(-maxPower, Math.min(power, maxPower));

            motorsSetPower(power, power, power, power);

            lastError = error;

            if (Math.abs(error) < 2) {
                break;
            }

            telemetry.addData("Encoders", "delanteDe: %d , delanteIz: %d , atrasDe: %d , atrasIz: %d",
                    delanteDe.getCurrentPosition(), delanteIz.getCurrentPosition(),
                    atrasDe.getCurrentPosition(), atrasIz.getCurrentPosition());
            telemetry.addData("Average encoder value: ", encoderAverage());
            telemetry.addData("Power: ", power);
            telemetry.update();
        }
        stopMotors();
    }
}
