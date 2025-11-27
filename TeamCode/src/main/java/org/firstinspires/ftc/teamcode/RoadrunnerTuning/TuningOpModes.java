package org.firstinspires.ftc.teamcode.RoadrunnerTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.AngularRampLogger;
import com.acmerobotics.roadrunner.ftc.DeadWheelDirectionDebugger;
import com.acmerobotics.roadrunner.ftc.DriveType;
import com.acmerobotics.roadrunner.ftc.DriveView;
import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.acmerobotics.roadrunner.ftc.EncoderGroup;
import com.acmerobotics.roadrunner.ftc.EncoderRef;
import com.acmerobotics.roadrunner.ftc.ForwardPushTest;
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger;
import com.acmerobotics.roadrunner.ftc.LateralPushTest;
import com.acmerobotics.roadrunner.ftc.LateralRampLogger;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxQuadratureEncoderGroup;
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner;
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger;
import com.acmerobotics.roadrunner.ftc.OTOSAngularScalarTuner;
import com.acmerobotics.roadrunner.ftc.OTOSEncoderGroup;
import com.acmerobotics.roadrunner.ftc.OTOSHeadingOffsetTuner;
import com.acmerobotics.roadrunner.ftc.OTOSIMU;
import com.acmerobotics.roadrunner.ftc.OTOSLinearScalarTuner;
import com.acmerobotics.roadrunner.ftc.OTOSPositionOffsetTuner;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrains.DeltaAuto;
import org.firstinspires.ftc.teamcode.Odometry.OTOSLocalizer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class TuningOpModes {
    public static final Class<?> DRIVE_CLASS = DeltaAuto.class;
    public static final String GROUP = "quickstart";
    public static final boolean DISABLED = false;

    private TuningOpModes() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

        DriveViewFactory dvf;
        if (DRIVE_CLASS.equals(DeltaAuto.class)) {
            dvf = hardwareMap -> {
                DeltaAuto md = new DeltaAuto(hardwareMap, new Pose2d(0, 0, 0));
                LazyImu lazyImu = md.lazyImu;

                List<EncoderGroup> encoderGroups = new ArrayList<>();
                List<EncoderRef> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
                List<EncoderRef> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
                if (md.localizer instanceof DeltaAuto.DriveLocalizer) {
                    DeltaAuto.DriveLocalizer dl = (DeltaAuto.DriveLocalizer) md.localizer;
                    encoderGroups.add(new LynxQuadratureEncoderGroup(
                            hardwareMap.getAll(LynxModule.class),
                            Arrays.asList(dl.leftFront, dl.leftBack, dl.rightFront, dl.rightBack)
                    ));
                    leftEncs.add(new EncoderRef(0, 0));
                    leftEncs.add(new EncoderRef(0, 1));
                    rightEncs.add(new EncoderRef(0, 2));
                    rightEncs.add(new EncoderRef(0, 3));
                } else if (md.localizer instanceof OTOSLocalizer) {
                    OTOSLocalizer ol = (OTOSLocalizer) md.localizer;
                    encoderGroups.add(new OTOSEncoderGroup(ol.otos));
                    parEncs.add(new EncoderRef(0, 0));
                    perpEncs.add(new EncoderRef(0, 1));
                    lazyImu = new OTOSIMU(ol.otos);
                } else {
                    throw new RuntimeException("unknown localizer: " + md.localizer.getClass().getName());
                }

                return new DriveView(
                    DriveType.MECANUM,
                        DeltaAuto.PARAMS.inPerTick,
                        DeltaAuto.PARAMS.maxWheelVel,
                        DeltaAuto.PARAMS.minProfileAccel,
                        DeltaAuto.PARAMS.maxProfileAccel,
                        encoderGroups,
                        Arrays.
                                asList(
                                md.leftFront,
                                md.leftBack
                        ),
                        Arrays.asList(
                                md.rightFront,
                                md.rightBack
                        ),
                        leftEncs,
                        rightEncs,
                        parEncs,
                        perpEncs,
                        lazyImu,
                        md.voltageSensor,
                        () -> new MotorFeedforward(DeltaAuto.PARAMS.kS,
                                DeltaAuto.PARAMS.kV / DeltaAuto.PARAMS.inPerTick,
                                DeltaAuto.PARAMS.kA / DeltaAuto.PARAMS.inPerTick),
                        0
                );
            };
        } else {
            throw new RuntimeException();
        }

        manager.register(metaForClass(AngularRampLogger.class), new AngularRampLogger(dvf));
        manager.register(metaForClass(ForwardPushTest.class), new ForwardPushTest(dvf));
        manager.register(metaForClass(ForwardRampLogger.class), new ForwardRampLogger(dvf));
        manager.register(metaForClass(LateralPushTest.class), new LateralPushTest(dvf));
        manager.register(metaForClass(LateralRampLogger.class), new LateralRampLogger(dvf));
        manager.register(metaForClass(ManualFeedforwardTuner.class), new ManualFeedforwardTuner(dvf));
        manager.register(metaForClass(MecanumMotorDirectionDebugger.class), new MecanumMotorDirectionDebugger(dvf));
        manager.register(metaForClass(DeadWheelDirectionDebugger.class), new DeadWheelDirectionDebugger(dvf));

        manager.register(metaForClass(ManualFeedbackTuner.class), ManualFeedbackTuner.class);
        manager.register(metaForClass(SplineTest.class), SplineTest.class);
        manager.register(metaForClass(LocalizationTest.class), LocalizationTest.class);

        manager.register(metaForClass(OTOSAngularScalarTuner.class), new OTOSAngularScalarTuner(dvf));
        manager.register(metaForClass(OTOSLinearScalarTuner.class), new OTOSLinearScalarTuner(dvf));
        manager.register(metaForClass(OTOSHeadingOffsetTuner.class), new OTOSHeadingOffsetTuner(dvf));
        manager.register(metaForClass(OTOSPositionOffsetTuner.class), new OTOSPositionOffsetTuner(dvf));

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                    AngularRampLogger.class,
                    ForwardRampLogger.class,
                    LateralRampLogger.class,
                    ManualFeedforwardTuner.class,
                    MecanumMotorDirectionDebugger.class,
                    ManualFeedbackTuner.class
            )) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}
