package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Chasis Test", group = "Linear OpMode")
public class Plesiov1 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor delanteIz, delanteDe, atrasIz, atrasDe;
    private DcMotor Intake = null;
    private DcMotor Elevador = null;
    private DcMotor Outtake = null;
    private CRServo ServoOuttake = null;
    private CRServo BrazoIntake = null;
    private CRServo GarraIntake= null;
    private CRServo Muñeca = null; // Añadido: Definir Muñeca
    private CRServo Brazo = null;  // Añadido: Definir Brazo

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        delanteIz = hardwareMap.get(DcMotor.class, "delanteIz");
        delanteDe = hardwareMap.get(DcMotor.class, "delanteDe");
        atrasIz = hardwareMap.get(DcMotor.class, "atrasIz");
        atrasDe = hardwareMap.get(DcMotor.class, "atrasDe");
        Elevador = hardwareMap.get(DcMotor.class, "elevador");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Outtake = hardwareMap.get(DcMotor.class, "outtake");
        BrazoIntake = hardwareMap.get(CRServo.class, "brazoI");
        GarraIntake = hardwareMap.get(CRServo.class, "garraI");
        ServoOuttake = hardwareMap.get(CRServo.class, "garraO");
        Muñeca = hardwareMap.get(CRServo.class, "muñeca"); // Inicialización de Muñeca
        Brazo = hardwareMap.get(CRServo.class, "brazo"); // Inicialización de Brazo

        float threshold = 0.5f; // Umbral (límite) de los triggers

        // Direcciones
        delanteIz.setDirection(DcMotor.Direction.REVERSE);
        delanteDe.setDirection(DcMotor.Direction.FORWARD);
        atrasIz.setDirection(DcMotor.Direction.REVERSE);
        atrasDe.setDirection(DcMotor.Direction.FORWARD);

        // Comportamiento al soltar
        delanteIz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        delanteDe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        atrasIz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        atrasDe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            chasis(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, 0.9);

            // Botones
            boolean ButtonA = gamepad2.a;
            boolean ButtonY = gamepad2.y;
            boolean ButtonX = gamepad2.x;
            boolean ButtonB = gamepad2.b;
            boolean flechaUp = gamepad2.dpad_up;
            boolean flechaDown = gamepad2.dpad_down;
            boolean flechaLeft = gamepad2.dpad_left;
            boolean flechaRight = gamepad2.dpad_right;
            boolean RightBumper = gamepad2.right_bumper;
            boolean LeftBumper = gamepad2.left_bumper;
            float RightTrigger = gamepad2.right_trigger;  // Trigger derecho como float
            float LeftTrigger = gamepad2.left_trigger;   // Trigger izquierdo como float

            Elevador(flechaUp, flechaDown);
            Brazo(RightBumper, LeftBumper);
            Muñeca(ButtonY, ButtonA);
            GarraOuttake(ButtonX, ButtonB);
            Intake(ButtonX, ButtonB);
            Outtake(RightTrigger, LeftTrigger);

            telemetry.addData("Runtime", runtime.seconds());
            telemetry.update();
        }
    }

    public void chasis(double x, double y, double rx, double maxPow) {
        double powDelanteIzquierda = (y + x + rx);
        double powAtrasIzquierda = (y - x + rx);
        double powDelanteDerecha = (y - x - rx);
        double powAtrasDerecha = (y + x - rx);

        powDelanteIzquierda = Math.max(-maxPow, Math.min(powDelanteIzquierda, maxPow));
        powAtrasIzquierda = Math.max(-maxPow, Math.min(powAtrasIzquierda, maxPow));
        powDelanteDerecha = Math.max(-maxPow, Math.min(powDelanteDerecha, maxPow));
        powAtrasDerecha = Math.max(-maxPow, Math.min(powAtrasDerecha, maxPow));

        delanteDe.setPower(powDelanteDerecha);
        delanteIz.setPower(powDelanteIzquierda);
        atrasIz.setPower(powAtrasIzquierda);
        atrasDe.setPower(powAtrasDerecha);
    }

    public void Elevador(boolean flechaUp, boolean flechaDown) {
        if (flechaUp) {
            Elevador.setPower(-1);
        } else if (flechaDown) {
            Elevador.setPower(1);
        } else {
            Elevador.setPower(0);
        }
    }

    public void GarraOuttake(boolean ButtonX, boolean ButtonB) {
        if (ButtonX) {
            ServoOuttake.setPower(-1);
        } else if (ButtonB) {
            ServoOuttake.setPower(1);
        } else {
            ServoOuttake.setPower(0);
        }
    }

    public void Brazo(boolean RightBumper, boolean LeftBumper) {
        if (RightBumper) {
            Brazo.setPower(-1);
        } else if (LeftBumper) {
            Brazo.setPower(1);
        } else {
            Brazo.setPower(0);
        }
    }

    public void Muñeca(boolean ButtonY, boolean ButtonA) {
        if (ButtonY) {
            Muñeca.setPower(-1);
        } else if (ButtonA) {
            Muñeca.setPower(1);
        } else {
            Muñeca.setPower(0);
        }
    }

    public void Intake(boolean ButtonX, boolean ButtonB) {
        if (ButtonX) {
            Intake.setPower(1);
        } else if (ButtonB) {
            Intake.setPower(-1);
        } else {
            Intake.setPower(0);
        }
    }

    public void Outtake(float RightTrigger, float LeftTrigger) {
        if (RightTrigger > 0.5f) {
            Outtake.setPower(1);
        } else if (LeftTrigger > 0.5f) {
            Outtake.setPower(-1);
        } else {
            Outtake.setPower(0);
        }
    }
}
