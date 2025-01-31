package org.firstinspires.ftc.teamcode.opmode.v2_opModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasSubsistemas.Inferior.UnderGrounSubystemStates;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasSubsistemas.Superior.UpperSubsystemStates;
import org.firstinspires.ftc.teamcode.subsystems.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Garra.GarraInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal.LinearHorizontalInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarra.BracoGarraSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Garra.GarraSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Horizontal.LinearHorizontalSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.LinearVertical.LinearVertical;

import java.util.ArrayList;

@TeleOp(name="Teleoperado V5")
public class TeleoperadoV5 extends OpMode {


    private V5 robot;
    GamepadEx gamepadEx1,gamepadEx2;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();

    ArrayList<Servo> servos = new ArrayList<>();

    public boolean needToOutake = false;

    @Override
    public  void init() {

        robot = this.createRobot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.TRASNFER;
        robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL;
    }
    @Override
    public void loop() {

        this.gerenciarModo(robot,gamepadEx1);
        this.robotCentricDrive(robot,gamepadEx1);
        this.binds(robot,gamepadEx2,robot.carteiro,gamepadEx1);
        this.linearHorizontalInferior(robot.intakeInferior.horizontalInferior,gamepadEx1);
        this.linearHorizontalSuperior(robot.outtakeIntakeSuperior.horizontalSuperior,gamepadEx1,robot.carteiro);
        this.linearVertical(robot.outtakeIntakeSuperior.linearVertical,gamepadEx1, robot.carteiro);
        this.bracoGarra(robot.outtakeIntakeSuperior.braco,gamepadEx1,robot.carteiro);
        this.garraInferior(robot.intakeInferior.garraInferior,robot.carteiro,gamepadEx1);
        this.garraSuperior(robot.outtakeIntakeSuperior.garraSuperior,robot.carteiro,gamepadEx1);
        this.runActions(robot.carteiro);
        this.testar1servo(robot,gamepadEx1,robot.carteiro);
        this.SetPositionServos();

        //telemetry.addData("Horizontal superior", robot.outtakeIntakeSuperior.horizontalSuperior.servoLinearHorizontal.getPosition());
        //telemetry.addData("braco garra superior", robot.outtakeIntakeSuperior.braco.bracoGarraSuperiorServo.getPosition());
        //telemetry.addData("garra angulação inferior", robot.intakeInferior.garraInferior.angulacaoGarraServo.getPosition());
        //telemetry.addData("Garra port", robot.intakeInferior.garraInferior.servoAberturaDaGarra.getPortNumber());
        //telemetry.addData("Garra pwm status", robot.intakeInferior.garraInferior.servoAberturaDaGarra.getController().getPwmStatus());
        //telemetry.addData("Posição garra", robot.outtakeIntakeSuperior.horizontalSuperior.servoLinearHorizontal.getPosition());
        telemetry.addData("PWM Horizontal superior", robot.outtakeIntakeSuperior.horizontalSuperior.servoLinearHorizontal.getController().getPwmStatus());
        telemetry.addData("Posição Horizintal Superior",robot.outtakeIntakeSuperior.horizontalSuperior.servoLinearHorizontal.getPosition());
        telemetry.addData("Variavel Horizonta",robot.outtakeIntakeSuperior.horizontalSuperior.servoLinearHorizontalPosition);

        telemetry.addData("PWM BRACO GARRA ",robot.outtakeIntakeSuperior.braco.bracoGarraSuperiorServo.getController().getPwmStatus());
        telemetry.addData("PWM BRACO GARRA ",robot.outtakeIntakeSuperior.braco.bracoGarraSuperiorServo.getPosition());
        telemetry.addData("PWM BRACO GARRA ",robot.outtakeIntakeSuperior.braco.ServoBracoSuperiorPosition);
        telemetry.update();


    }

    @NonNull
    private V5 createRobot(HardwareMap hardwareMap) {

        return new V5(hardwareMap, telemetry);

    }
    private void robotCentricDrive(V5 robot,GamepadEx gamepad) {
        double drive = Range.clip(gamepad.getLeftY(), -1, 1);

        //if( Math.abs(drive) < 0.8 && Math.abs(drive) > 0.02  ) drive = (drive / 1.5);

        double strafe = Range.clip(-gamepad.getLeftX(), -1, 1);
        if (gamepad1.right_stick_button) strafe = -0.6;
        if (gamepad1.left_stick_button) strafe = 0.6;
        //if ( Math.abs(strafe) < 0.8 && Math.abs(strafe) > 0.02 ) strafe = (strafe / 2.5);

        double turn = -gamepad.getRightX();
        if (gamepad1.left_trigger > 0) {
            turn = -gamepad1.left_trigger * 0.5;
        }

        if (gamepad1.right_trigger > 0) {
            turn = gamepad1.right_trigger * 0.5;
        }

        turn = Range.clip(turn / 1.3, -0.7, 0.7);

        robot.md.setDrivePowers(new PoseVelocity2d(new Vector2d(drive, strafe), turn));

        robot.md.updatePoseEstimate();

    }

    private void testar1servo(V5 robot, GamepadEx gamepad,OrdersManager carteiro) {

    }
    private void binds(V5 robot, GamepadEx gamepad, OrdersManager carteiro, GamepadEx gamepadEx1) {

        // X -> A -> B -> Y-> DPAD RIGHT
        if(gamepad.getButton(GamepadKeys.Button.A)){
            //intake normal
            //robot.intakeInferior.goToIntake(carteiro, getRuntime());
            robot.outtakeIntakeSuperior.goToIntakeCHAMBER(carteiro, getRuntime());

        }
        if(gamepad.getButton(GamepadKeys.Button.B)){
            //robot.intakeInferior.goToReadyTransfer(carteiro, getRuntime());
            //robot.outtakeIntakeSuperior.goToReadyTransfer(carteiro, getRuntime());
            robot.outtakeIntakeSuperior.goToOuttakeCHAMBER(carteiro,getRuntime());

        }

        if (gamepad.getButton(GamepadKeys.Button.X)) {
            //robot.intakeInferior.goToReadyToIntake(carteiro, getRuntime());
            robot.outtakeIntakeSuperior.goToReadOuttakeCHAMBER(carteiro, getRuntime());

        }
        if (gamepad.getButton(GamepadKeys.Button.Y)){
            //robot.outtakeIntakeSuperior.goToTransfer(carteiro, getRuntime());
            //robot.intakeInferior.goToTransfer(carteiro, getRuntime());

        }

        if (gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT) || needToOutake){
            //needToOutake = false;
            //robot.outtakeIntakeSuperior.goToOuttake(carteiro);
            //robot.intakeInferior.garraInferior.goToOuttake2();
        }
        if(gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            Actions.runBlocking(
                    robot.intakeInferior.garraInferior.goToAbrirGarra()
            );
        }
        if(gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            Actions.runBlocking(
                    robot.intakeInferior.garraInferior.goToFecharGarra()
            );
        }
        if (gamepad.getButton(GamepadKeys.Button.DPAD_UP)){
            Actions.runBlocking(
                    robot.outtakeIntakeSuperior.garraSuperior.fecharGarra()
            );
        }
        if (gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)){
            Actions.runBlocking(
                    robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()
            );
        }
    }

    private void linearVertical(LinearVertical vertical,GamepadEx gamepad, OrdersManager carteiro)  {
        vertical.PIDF();
       // todo: controle manual
        if(gamepad.getButton(GamepadKeys.Button.DPAD_UP)){
            vertical.upSetPoint();
        }
        if(gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)){
            vertical.downSetPoint();
        }

        if(gamepad.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)){
            carteiro.addOrder(vertical.ElevadorGoTo(1380), 0, "SKY-LINE", getRuntime());
        }
        if(gamepad.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
            carteiro.addOrder(vertical.ElevadorGoTo(50), 0, "FLOOR-LINE", getRuntime());
        }

    }
    private void bracoGarra(BracoGarraSuperior braco, GamepadEx gamepad, OrdersManager carteiro)  {
       // carteiro.addOrder(robot.outtakeIntakeSuperior.braco.goToInital(), 0,"braco superior inicial");
    }
    private void linearHorizontalSuperior(LinearHorizontalSuperior horizontal, GamepadEx gamepad, OrdersManager carteiro)  {
        Actions.runBlocking(
                horizontal.goToRetracted()
        );

    }
    private void linearHorizontalInferior(LinearHorizontalInferior horizontal, GamepadEx gamepad)  {

    }
    private void garraSuperior(GarraSuperior garra, OrdersManager carteiro, GamepadEx gamepad){
    }
    private void garraInferior(GarraInferior subsistemasInferiores, OrdersManager carteiro, GamepadEx gamepad){

    }
    private void SetPositionServos(){
        //robot.intakeInferior.bracoGarraInferior.bracoGarraInferior.setPosition(robot.intakeInferior.bracoGarraInferior.ServoBracoInferiorPosition);
        //robot.intakeInferior.garraInferior.servoRotacaoDaGarra.setPosition(robot.intakeInferior.garraInferior.ServoRotacaoInferiorPosition);
        //robot.intakeInferior.garraInferior.angulacaoGarraServo.setPosition(robot.intakeInferior.garraInferior.ServoAngulacaoPosition);

        robot.outtakeIntakeSuperior.garraSuperior.angulacaoGarraSuperiorServo.setPosition(robot.outtakeIntakeSuperior.garraSuperior.ServoAngulacaoSuperiorPosition);
        robot.outtakeIntakeSuperior.braco.bracoGarraSuperiorServo.setPosition(robot.outtakeIntakeSuperior.braco.ServoBracoSuperiorPosition);
        //robot.outtakeIntakeSuperior.horizontalSuperior.servoLinearHorizontal.setPosition(robot.outtakeIntakeSuperior.horizontalSuperior.servoLinearHorizontalPosition);
        robot.outtakeIntakeSuperior.horizontalSuperior.servoLinearHorizontal.setPosition(0.634);

    }
    private void gerenciarModo(V5 robot,GamepadEx gamepad) {


    }
    private void runActions(OrdersManager carteiro) {
        carteiro.checkIfCanRun(getRuntime());
        carteiro.runTeleopActions(getRuntime());
    }

}