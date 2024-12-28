package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Chasis Final", group = "Linear OpMode")
public class BobSponjav1 extends LinearOpMode {

    /*
    // Variables
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor delanteIz = null;
    private DcMotor delanteDe = null;
    private DcMotor atrasIz = null;
    private DcMotor atrasDe = null;
    private DcMotor Intake = null;
    private DcMotor trompo = null;
    private DcMotor elevador = null;
    private CRServo garra = null;

    private boolean garraAbierta = false;
    private boolean ButtonY = false;
    private boolean ButtonA = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Declarar las variables en la driver
        delanteIz = hardwareMap.get(DcMotor.class, "delanteIz");
        delanteDe = hardwareMap.get(DcMotor.class, "delanteDe");
        atrasIz = hardwareMap.get(DcMotor.class, "atrasIz");
        atrasDe = hardwareMap.get(DcMotor.class, "atrasDe");
        trompo = hardwareMap.get(DcMotor.class, "trompo");
        elevador = hardwareMap.get(DcMotor.class, "elevador");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        garra = hardwareMap.get(CRServo.class, "garra");

        // Establecer dirección de motores
        delanteIz.setDirection(DcMotor.Direction.FORWARD);
        delanteDe.setDirection(DcMotor.Direction.REVERSE);
        atrasIz.setDirection(DcMotor.Direction.REVERSE);
        atrasDe.setDirection(DcMotor.Direction.FORWARD);
        trompo.setDirection(DcMotor.Direction.FORWARD);

        // Configurar la garra en estado inicial (cerrada)
        garra.setPower(0);

        // Esperar a que inicie
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Movimiento del chasis
            double x = gamepad1.left_stick_x;  // Movimiento lateral
            double y = -gamepad1.left_stick_y; // Movimiento adelante/atrás
            double rotation = gamepad1.right_stick_x; // Rotación

            // Potencias para el chasis
            delanteIz.setPower((x - y) - rotation);
            delanteDe.setPower((x - y) + rotation);
            atrasIz.setPower((x + y) + rotation);
            atrasDe.setPower((x + y) - rotation);

            // Control de trompo
            MoverTrompo(gamepad2.left_trigger, gamepad2.right_trigger);

            // Control del elevador
            Elevador(gamepad2.dpad_up, gamepad2.dpad_down);

            // Control del Intake
            Intake(gamepad2.x, gamepad2.b);

            // Control de la garra con lógica toggle
            if (gamepad2.y && !ButtonY) {
                garraAbierta = true;
            }
            if (gamepad2.a && !ButtonA) {
                garraAbierta = false;
            }

            // Actualizar estado de la garra con menor potencia
            if (garraAbierta) {
                garra.setPower(0.01);
            } else {
                garra.setPower(-0.7);
            }

            // Actualizar estados previos de los botones
            ButtonY = gamepad2.y;
            ButtonA = gamepad2.a;

            // Mostrar información en la telemetría
            telemetry.addData("Runtime", runtime.seconds());
            telemetry.addData("Garra Abierta", garraAbierta);
            telemetry.addData("x  ", x);
            telemetry.addData("y  ", y);
            telemetry.addData("rotation  ", rotation);
            telemetry.update();
        }
    }

    // Trompo
    public void MoverTrompo(double triggerL, double triggerR) {
        if (triggerL > 0 && triggerR == 0) {
            trompo.setPower(-triggerL);
        } else if (triggerR > 0 && triggerL == 0) {
            trompo.setPower(triggerR);
        } else {
            trompo.setPower(0);
        }
    }

    // Elevador
    public void Elevador(boolean flechaUp, boolean flechaDown) {
        if (flechaUp) {
            elevador.setPower(-1);
        } else if (flechaDown) {
            elevador.setPower(1);
        } else {
            elevador.setPower(0);
        }
    }





    delanteIz.setPower((x - y) - rotation);
    delanteDe.setPower((x - y) + rotation);
    atrasIz.setPower((x + y) + rotation);
    atrasDe.setPower((x + y) - rotation);






    // Intake
    public void Intake(boolean buttonX, boolean buttonB) {
        if (buttonX) {
            Intake.setPower(1);
        } else if (buttonB) {
            Intake.setPower(-1);
        } else {
            Intake.setPower(0);
        }
    }
     */
}

