/*
 * Copyright 2024 pinorobotics
 * 
 * Website: https://github.com/pinorobotics/r2d2
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *   http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package pinorobotics.r2d2.control;

import id.jros2client.JRos2ClientConfiguration;
import id.jros2client.JRos2ClientFactory;
import id.jroscommon.RosName;
import id.opentelemetry.exporters.ElasticsearchMetricExporter;
import id.xfunction.ResourceUtils;
import id.xfunction.cli.ArgumentParsingException;
import id.xfunction.cli.CommandLineInterface;
import id.xfunction.cli.CommandOptions;
import id.xfunction.logging.XLogger;
import io.opentelemetry.sdk.OpenTelemetrySdk;
import io.opentelemetry.sdk.metrics.SdkMeterProvider;
import io.opentelemetry.sdk.metrics.export.PeriodicMetricReader;
import java.io.IOException;
import java.net.URI;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.Duration;
import java.util.Arrays;
import java.util.List;
import java.util.Map.Entry;
import java.util.Optional;
import java.util.function.Function;
import pinorobotics.drac.DornaClientConfig;
import pinorobotics.drac.DornaClientFactory;
import pinorobotics.drac.DornaRobotModel;
import pinorobotics.drac.Joints;
import pinorobotics.drac.messages.Motion;
import pinorobotics.jros2control.joint_trajectory_controller.ActuatorHardware;
import pinorobotics.jros2control.joint_trajectory_controller.ActuatorHardware.JointState;
import pinorobotics.jros2control.joint_trajectory_controller.JointStateBroadcaster;
import pinorobotics.jros2control.joint_trajectory_controller.JointTrajectoryControllerFactory;
import pinorobotics.jros2moveit.moveit_config.MoveItConfigUtils;
import pinorobotics.jros2moveit.moveit_config.models.Ros2Controller;
import pinorobotics.jros2moveit.moveit_config.models.RosParameters;

/**
 * @author aeon_flux aeon_flux@eclipso.ch
 */
public class Dorna2JointTrajectoryControllerApp {
    private static final XLogger LOGGER =
            XLogger.getLogger(Dorna2JointTrajectoryControllerApp.class);
    private static final int DEFAULT_BROADCASTER_RATE_IN_MILLIS = 100;
    private static final String DEFAULT_DORNA_URL = "ws://192.168.0.3:443";
    private static final String DEFAULT_CONTROLLER_NAME = "dorna2_arm_controller";

    private MoveItConfigUtils moveItConfigUtils = new MoveItConfigUtils();
    private String dornaUrl;
    private String elasticUrl;
    private RosName controllerName;
    private Duration broadcasterRate;
    private Path moveitConfigPath;
    private List<String> joints = List.of();

    public Dorna2JointTrajectoryControllerApp(CommandOptions properties) {
        dornaUrl = properties.getOption("dornaUrl").orElse(DEFAULT_DORNA_URL);
        elasticUrl = properties.getOption("exportMetricsToElastic").orElse("");
        controllerName =
                new RosName(properties.getOption("controllerName").orElse(DEFAULT_CONTROLLER_NAME));
        broadcasterRate =
                Duration.ofMillis(
                        properties
                                .getOptionInt("broadcastRateInMillis")
                                .orElse(DEFAULT_BROADCASTER_RATE_IN_MILLIS));
        moveitConfigPath =
                Paths.get(properties.getOption("moveItConfigPath").orElse("../r2d2_moveit_config"));
    }

    /** Setup OpenTelemetry to send metrics to Elasticsearch */
    private void setupMetrics() {
        var elasticUri =
                Optional.ofNullable(System.getenv("METRICS_ELASTIC_URL")).orElse(elasticUrl);
        if (elasticUri.isBlank()) {
            LOGGER.warning("Metrics are ignored because metrics URL is blank");
            return;
        }

        var metricReader =
                PeriodicMetricReader.builder(
                                new ElasticsearchMetricExporter(
                                        URI.create(elasticUri + "/r2d2_control"),
                                        Optional.empty(),
                                        Duration.ofSeconds(5),
                                        true))
                        .setInterval(Duration.ofSeconds(3))
                        .build();
        var sdkMeterProvider =
                SdkMeterProvider.builder().registerMetricReader(metricReader).build();
        OpenTelemetrySdk.builder().setMeterProvider(sdkMeterProvider).buildAndRegisterGlobal();
    }

    private void run() {
        var moveItConfig = moveItConfigUtils.read(moveitConfigPath);
        joints =
                Optional.ofNullable(moveItConfig.ros2Controllers().get(controllerName.name()))
                        .map(Ros2Controller::ros__parameters)
                        .map(RosParameters::joints)
                        .orElseThrow(
                                () ->
                                        new RuntimeException(
                                                "Could not find joints inside moveit_configs for"
                                                        + " controller "
                                                        + controllerName));
        Function<String, RuntimeException> exceptionFactory =
                jointName ->
                        new RuntimeException(
                                "Joint with name "
                                        + jointName
                                        + " is not listed among joints of controller "
                                        + controllerName);
        // sort initial positions with respect to "joints"
        var initialPositions =
                moveItConfig.initialPositions().initialPositions().entrySet().stream()
                        .sorted(
                                (e1, e2) -> {
                                    var joint1 = e1.getKey();
                                    var pos1 = joints.indexOf(joint1);
                                    if (pos1 == -1) throw exceptionFactory.apply(joint1);
                                    var joint2 = e2.getKey();
                                    var pos2 = joints.indexOf(joint2);
                                    if (pos2 == -1) throw exceptionFactory.apply(joint2);
                                    return pos1 - pos2;
                                })
                        .map(Entry::getValue)
                        .toList();
        setupMetrics();
        var dornaConfig =
                new DornaClientConfig.Builder(URI.create(dornaUrl), DornaRobotModel.DORNA2_BLACK)
                        //                .noopMode(true)
                        .build();
        var dornaClient = new DornaClientFactory().createClient(dornaConfig);
        var configBuilder = new JRos2ClientConfiguration.Builder();
        dornaClient.motor(true);
        ActuatorHardware dornaJointStateSender =
                jointState -> {
                    dornaClient.jmove(
                            Joints.ofRadians(jointState.positions()),
                            false,
                            true,
                            true,
                            dornaConfig.velocity(),
                            dornaConfig.acceleration(),
                            dornaConfig.jerk());
                };
        var controllers = List.of(dornaJointStateSender);
        // use configBuilder to override default parameters (network interface, RTPS settings etc)
        try (var client = new JRos2ClientFactory().createClient(configBuilder.build());
                var jointStateBroadcaster =
                        new JointStateBroadcaster(
                                client,
                                joints,
                                Optional.of(() -> toJointState(dornaClient.getLastMotion())),
                                broadcasterRate);
                var controller =
                        new JointTrajectoryControllerFactory()
                                .create(
                                        client,
                                        controllers,
                                        joints,
                                        initialPositions,
                                        controllerName); ) {
            jointStateBroadcaster.start();
            controller.start();
            CommandLineInterface.cli.askPressEnter();
        }
        // usually we need to close client once we are done
        // but here we keep it open so that subscriber will keep
        // printing messages indefinitely
    }

    private JointState toJointState(Motion motion) {
        var j = motion.joints().toArrayOfRadians();
        return new JointState(
                joints,
                Arrays.copyOf(j, joints.size()),
                Arrays.copyOf(new double[] {motion.vel()}, joints.size()));
    }

    private static void usage() throws IOException {
        new ResourceUtils().readResourceAsStream("README-r2d2.md").forEach(System.out::println);
    }

    public static void main(String[] args) throws IOException {
        CommandOptions properties = null;
        try {
            properties = CommandOptions.collectOptions(args);
            if (properties.getOption("h").isPresent() || properties.getOption("help").isPresent())
                throw new ArgumentParsingException("");
        } catch (ArgumentParsingException e) {
            usage();
            return;
        }
        XLogger.load(
                properties.isOptionTrue("debug")
                        ? "logging-r2d2-control-debug.properties"
                        : "logging-r2d2-control.properties");
        LOGGER.fine("Input arguments {0}", properties);
        new Dorna2JointTrajectoryControllerApp(properties).run();
    }
}
