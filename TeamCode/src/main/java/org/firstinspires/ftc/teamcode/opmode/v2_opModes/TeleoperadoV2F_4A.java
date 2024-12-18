package org.firstinspires.ftc.teamcode.opmode.v2_opModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.RobotState;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.agregadoras.V2;
import org.firstinspires.ftc.teamcode.agregadoras.Vertex;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.subsystems.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.Vertex.BracoGarra;
import org.firstinspires.ftc.teamcode.subsystems.Vertex.LinearHorizontal;
import org.firstinspires.ftc.teamcode.subsystems.Vertex.LinearVertical;

@TeleOp(name="Teleoperado V2f")
public class TeleoperadoV2F_4A extends OpMode {


    private V2 robot;
    GamepadEx gamepadEx;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public  void init() {

        robot = this.createRobot(hardwareMap);
        gamepadEx = new GamepadEx(gamepad2);
        robot.intakeOutake.RobotState = Vertex.vertexState.Initial;
        Globals.TA_STORED = true;


    }
    @Override
    public void loop() {
        this.gerenciarModo(robot);
        this.robotCentricDrive(robot);
        this.binds(robot);
        this.linearHorizontal(robot.intakeOutake.linearHorizontal);
        this.linearVertical(robot.intakeOutake.linearVertical);
        this.bracoGarra(robot.intakeOutake.braco);
        this.runActions(robot.intakeOutake.carteiro);
        telemetry.addData("Alvo",robot.intakeOutake.braco.targetPosition);
        telemetry.addData("Atual",robot.intakeOutake.braco.motorBracoGarra.getCurrentPosition());

    }

    @NonNull
    private V2 createRobot(HardwareMap hardwareMap) {

        return new V2(hardwareMap, telemetry);

    }
    private void robotCentricDrive(V2 robot)  {

    }
    private void binds(V2 robot) {

        if(gamepadEx.getButton(GamepadKeys.Button.A) || Globals.TA_INTAKE){
            robot.intakeOutake.IntermediatePosition(getRuntime());
            robot.intakeOutake.RobotState = Vertex.vertexState.Intermediate;


            Globals.TA_INTAKE = true;
            Globals.TA_PONTUANDO = false;
            Globals.TA_STORED = false;
        }
        if(gamepadEx.getButton(GamepadKeys.Button.B) || Globals.TA_STORED){
            robot.intakeOutake.InitialPosition(getRuntime());
            robot.intakeOutake.RobotState = Vertex.vertexState.Initial;

            Globals.TA_INTAKE = false;
            Globals.TA_PONTUANDO = false;
            Globals.TA_STORED = true;
        }
        /*if(gamepad2.x || robot.intakeOutake.getRobotState().equals(Vertex.vertexState.Intake)){
            robot.intakeOutake.IntakePosition(getRuntime());
            robot.intakeOutake.RobotState = Vertex.vertexState.Intake;
        } */
        if(gamepadEx.getButton(GamepadKeys.Button.Y) || Globals.TA_PONTUANDO){
            robot.intakeOutake.OuttakePosition(getRuntime());
            robot.intakeOutake.RobotState = Vertex.vertexState.Outtake;

            Globals.TA_INTAKE = false;
            Globals.TA_PONTUANDO = true;
            Globals.TA_STORED = false;
        }

        /* gamepadEx.getGamepadButton(GamepadKeys.Button.A);
                new SequentialAction(

                                        robot.intakeOutake.IntermediatePosition(getRuntime()),
                                        robot.intakeOutake.RobotState = Vertex.vertexState.Intermediate,

        );*/
    }
    private void linearHorizontal(LinearHorizontal horizontal)  {
    }
    private void linearVertical(LinearVertical vertical)  {
    }
    private void bracoGarra(BracoGarra braco)  {
        if(gamepad2.dpad_left){
            robot.intakeOutake.braco.upBraco(5);
        }
        if(gamepad2.dpad_right ){
            robot.intakeOutake.braco.downBraco(5);
        }
        robot.intakeOutake.braco.handleBracoTeleop(getRuntime());
    }
    private void gerenciarModo(V2 robot) {


    }
    private void runActions(OrdersManager carteiro) {

        carteiro.checkIfCanRun(getRuntime());
        carteiro.runTeleopActions(getRuntime());
        carteiro.removeOrderByName(robot.intakeOutake.getRobotState().name());

    }

}