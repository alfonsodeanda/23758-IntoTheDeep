package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp(name = "Plesiov2 | Teleop Ferrería")
public class Plesiov2punto1 extends LinearOpMode {
    private AtomicBoolean isTransferRunning = new AtomicBoolean(false); // Bandera para controlar el estado de la rutina

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor DelanteIz, DelanteDe, AtrasIz, AtrasDe;
    private DcMotor Intake = null;
    private DcMotor Elevador = null;
    private DcMotor IntakeWrist = null;
    private DcMotor Outtake = null;
    private Servo ServoOuttake = null;
    private Servo Muñeca = null;


    //variables para el Toggle en las garras de intake y outtake
    private boolean outtakeMode = false; //para saber el modo en el que esta el outtake (abrir/cerrar)
    private boolean outtakeButtonState = false; //para saber si el boton ha sido presionado
    private boolean intakeMode = false; //para saber el modo en el que esta el intake(abrir/cerrar)
    private boolean intakeButtonState = false; //para saber si el boton ha sido presionado
    public boolean transferButtonState = false; //para saber si el boton ha sido presionado


    final double outtake_open = 0.5;
    final double outtake_close = 0.2;
    final double intake_open = 0.4;
    final double intake_close = 0.55;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Programa de Teleoperado version 1.2");
        telemetry.addLine("Controles:" +
                "GAMEPAD 1 (CHASIS)" +
                "Joystick izquierdo: Movmiento en x, y. Joystick derecho: Rotacion sobre su propio eje" +
                "GAMEPAD 2 (MECANISMOS)" +
                "Joystick izquierdo: muñeca intake. Joystick derecho: muñeca outtake" +
                "Boton Y: Garra Intake. Boton X: Garra outtake. Boton A: Transfer de piezas" +
                "DPAD Arriba: Elevador hacia arriba. DPAD Abajo: Elevador hacia abajo" +
                "DPAD izquierda: Extender riel. DPAD Derecha: Contraer Riel");
        telemetry.update();

        //Motores chasis
        DelanteIz = hardwareMap.get(DcMotor.class, "DelanteIz");
        DelanteDe = hardwareMap.get(DcMotor.class, "DelanteDe");
        AtrasIz = hardwareMap.get(DcMotor.class, "AtrasIz");
        AtrasDe = hardwareMap.get(DcMotor.class, "AtrasDe");
        //Motores mecanismos
        Elevador = hardwareMap.get(DcMotor.class, "elevador"); //Uno de los dos motores del elevador
        IntakeWrist = hardwareMap.get(DcMotor.class, "intakeWrist");
        Intake = hardwareMap.get(DcMotor.class, "Intake"); //Motor del riel
        Outtake = hardwareMap.get(DcMotor.class, "outtake"); //Motor de la muñeca del outtake
        //Servos mecanismos
        ServoOuttake = hardwareMap.get(Servo.class, "garraO"); //servo de la garra del outtake
        Muñeca = hardwareMap.get(Servo.class, "muñeca"); //servo de la garra del intake

        // Direcciones
        DelanteIz.setDirection(DcMotor.Direction.FORWARD);
        DelanteDe.setDirection(DcMotor.Direction.REVERSE);
        AtrasIz.setDirection(DcMotor.Direction.FORWARD);
        AtrasDe.setDirection(DcMotor.Direction.REVERSE);
        Muñeca.setDirection(Servo.Direction.REVERSE);

        zeroPowerBehaviorBrake();
        resetEncoders();

        waitForStart();
        runtime.reset();

        //Programa principal, se repite por siempre
        while (opModeIsActive()) {
            chassis(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, 0.9);
            intakeWrist(gamepad2.left_stick_y);
            intakeToggle(gamepad2.y); //garra elevador
            outtakeToggle(gamepad2.x); //brazo
            riel(gamepad2.dpad_right, gamepad2.dpad_left);
            outtakeWrist(gamepad2.right_stick_y); //mover brazo
            elevador(gamepad2.dpad_up, gamepad2.dpad_down);

            // Llama al transfer solo si no está en ejecución
            if (gamepad2.a && !isTransferRunning.get()) {
                isTransferRunning.set(true);
                new Thread(() -> { //mandar a llamar el transfer en paralelo
                    transfer();
                    isTransferRunning.set(false); // Marca como completado
                }).start();
            }

            //imprimir valores para debuggear
            telemetry.addData("Runtime", runtime.seconds());
            telemetry.addData("Riel", Intake.getCurrentPosition());
            telemetry.addData("Muñeca outtake:", Outtake.getCurrentPosition());
            telemetry.addData("Elevador:", Elevador.getCurrentPosition());
            telemetry.addData("Muñeca intake:", IntakeWrist.getCurrentPosition());
            telemetry.update();
        }
    }

    //settea todos los motores en brake cuando no se les aplica potencia
    public void zeroPowerBehaviorBrake(){
        DelanteIz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DelanteDe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AtrasIz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AtrasDe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Elevador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeWrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //se configuran los encoders para poder acceder a las lecturas y configurar limites
    public void resetEncoders() {
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevador.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeWrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Elevador.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeWrist.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*-------------AQUI INICIAN FUNCIONES DE MOVIMIENTO EN EL ROBOT-----------------*/

    /*-------------CHASIS-----------------*/
    //funcion de movimiento en chasis.
    //Parametros: joystick del valor en x, en y, y de la rotacion. Valor maximo de potencia que se aplica a motores
    public void chassis(double x, double y, double rx, double maxPow) {
        //formula de movimiento mecanum almacenado en variable
        double powDelanteIzquierda = (y + x + rx);
        double powAtrasIzquierda = (y - x + rx);
        double powDelanteDerecha = (y - x - rx);
        double powAtrasDerecha = (y + x - rx);

        //Se limita el valor maximo que puede dar la variable de la formula al maximo escogido en el parametro
        powDelanteIzquierda = Math.max(-maxPow, Math.min(powDelanteIzquierda, maxPow));
        powAtrasIzquierda = Math.max(-maxPow, Math.min(powAtrasIzquierda, maxPow));
        powDelanteDerecha = Math.max(-maxPow, Math.min(powDelanteDerecha, maxPow));
        powAtrasDerecha = Math.max(-maxPow, Math.min(powAtrasDerecha, maxPow));

        //se aplica la potencia con limites
        DelanteDe.setPower(powDelanteDerecha);
        DelanteIz.setPower(powDelanteIzquierda);
        AtrasIz.setPower(powAtrasIzquierda);
        AtrasDe.setPower(powAtrasDerecha);
    }

    /*-------------MECANISMOS-----------------*/

    //funcion de control del elevador. con cada boton se prenden dos motores
    public void elevador(boolean up, boolean down) {
        if (up) {
            Elevador.setPower(-1);
        } else if (down) {
            Elevador.setPower(1);
        } else {
            Elevador.setPower(0);
        }
    }

    //funcion para abrir/cerrar la garra del intake con un solo boton
    public void outtakeToggle(boolean button) { //parametro del boton que se usara
        if (button && !outtakeButtonState) { //si el boton es presionado una vez
            outtakeMode = !outtakeMode; //cambia el modo (abrir a cerrar, o cerrar a abrir)
            outtakeButtonState = true; //guarda que esta presionado
        }

        if(!button){ //si no es presionado
            outtakeButtonState=false;  //guardar que no esta siendo presionado
        }

        double pos=1;
        if (!outtakeMode) pos = outtake_close; //si el modo es cerrar, aplicar power negativo
        else pos = outtake_open; //si es abrir, aplicar power positivo
        ServoOuttake.setPosition(pos);
    }

    //funcion de la muneca del intake. se controla con un joystick
    public void intakeWrist(double y) {
        if(y < 0.0){
            IntakeWrist.setPower(-0.4);
        }
        else if(y > 0.0){
            IntakeWrist.setPower(0.5);
        }
        else{
            IntakeWrist.setPower(0.0);
        }
    }

    //funcion para abrir/cerrar la garra del intake con un solo boton
    public void intakeToggle(boolean button) { //parametro del boton que se usara
        if (button && !intakeButtonState) { //si el boton es presionado una vez
            intakeMode = !intakeMode; //cambia el modo (abrir a cerrar, o cerrar a abrir)
            intakeButtonState = true; //guarda que esta presionado
        }

        if(!button){ //si no es presionado
            intakeButtonState=false;  //guardar que no esta siendo presionado
        }

        double pos=1;
        if (!intakeMode) pos = intake_close; //si el modo es cerrar, aplicar power negativo
        else pos = intake_open; //si es abrir, aplicar power positivo
        Muñeca.setPosition(pos);
    }

    //funcion de movimiento del intake, con dos botones
    public void riel(boolean forward, boolean backward) {
        if (forward) {
            Intake.setPower(-1);
        } else if (backward) {
            Intake.setPower(1);
        } else {
            Intake.setPower(0);
        }
    }

    //funcion de la muneca del outtake(motor core hex). se controla con el joystick
    public void outtakeWrist(double y) {
        if(y < 0.0){
            Outtake.setPower(0.5);
        }
        else if(y > 0.0){
            Outtake.setPower(-0.5);
        }
        else{
            Outtake.setPower(0.0);
        }
    }

    public void transfer() {
        Outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorsSetPower(0, 0, 0, 0);

        Muñeca.setPosition(0.55); //cerrar intake (apretao)
        sleep(50);

        autoRiel(-200, 0.5); //acomodar riel

        autoIntakeWrist(0, 0.85); //bajar muneca

        ServoOuttake.setPosition(outtake_open);
        autoOuttakeWristPos(-145, 0.8); //bajar muneca
        ServoOuttake.setPosition(outtake_close);

        Muñeca.setPosition(intake_open);
        sleep(50);

        autoRiel(-550, 0.95); //acomodar riel

        autoOuttakeWristPos(-10, 0.8);

        Muñeca.setPosition(intake_close);
        sleep(50);

        intakeMode = false; //cambiar de estado las garras para que se queden cerradas
        outtakeMode = false;
    }

    public void autoRiel(int pos, double pow){
        Intake.setTargetPosition(pos);
        Intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Intake.setPower(pow);

        while (opModeIsActive() && Intake.isBusy()) {
            telemetry.addData("Encoder Riel:", Intake.getCurrentPosition());
            telemetry.update();
        }

        Intake.setPower(0);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void autoIntakeWrist(int pos, double pow){
        IntakeWrist.setTargetPosition(pos);
        IntakeWrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IntakeWrist.setPower(pow);

        while (opModeIsActive() && IntakeWrist.isBusy()) {
            telemetry.addData("Encoder muñeca:", IntakeWrist.getCurrentPosition());
            telemetry.update();
        }

        IntakeWrist.setPower(0);
        IntakeWrist.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void autoOuttakeWristPos(int pos, double pow){
        Outtake.setTargetPosition(pos);
        Outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Outtake.setPower(pow);

        while (opModeIsActive() && Outtake.isBusy()) {
            telemetry.addData("Encoder muñeca:", Outtake.getCurrentPosition());
            telemetry.update();
        }

        Outtake.setPower(0);
        Outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void motorsSetPower (double powDeIz, double powDeDe, double powAtIz, double powAtDe) {
        DelanteIz.setPower(powDeIz);
        DelanteDe.setPower(powDeDe);
        AtrasIz.setPower(powAtIz);
        AtrasDe.setPower(powAtDe);
    }
}


