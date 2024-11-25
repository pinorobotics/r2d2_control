/*
 * Copyright 2024 pinorobotics
 * 
 * Website: https://github.com/pinorobotics
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
import id.xfunction.cli.CommandLineInterface;
import id.xfunction.logging.XLogger;
import java.net.URI;
import java.time.Duration;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import pinorobotics.drac.DornaClientConfig;
import pinorobotics.drac.DornaClientFactory;
import pinorobotics.drac.DornaRobotModel;
import pinorobotics.drac.Joints;
import pinorobotics.drac.messages.Motion;
import pinorobotics.jros2control.JointStateBroadcaster;
import pinorobotics.jros2control.JointStateListener;
import pinorobotics.jros2control.JointStateListener.JointState;
import pinorobotics.jros2control.JointTrajectoryController;

public class Dorna2JointTrajectoryControllerApp {
    private static final int DEFAULT_BROADCASTER_RATE_IN_MILLIS = 100;
    // from ros2_controllers.yaml
    private static final List<String> joints =
            List.of("Joint_0", "Joint_1", "Joint_2", "Joint_3", "Joint_4");

    public static void main(String[] args) {
        XLogger.load("logging-r2d2-control-debug.properties");
        // initial_positions.yaml
        var initialPositions = List.of(3.0815, 3.0815, -2.3783, 2.261, 0.0);
        var dornaClient =
                new DornaClientFactory()
                        .createClient(
                                new DornaClientConfig.Builder(
                                                URI.create("ws://192.168.0.3:443"),
                                                DornaRobotModel.DORNA2_BLACK)
                                        .build());
        var configBuilder = new JRos2ClientConfiguration.Builder();
        dornaClient.motor(true);
        JointStateListener dornaJointStateSender =
                jointState -> {
                    dornaClient.jmove(Joints.ofRadians(jointState.positions()), false);
                };
        // use configBuilder to override default parameters (network interface, RTPS settings etc)
        try (var client = new JRos2ClientFactory().createClient(configBuilder.build());
                var jointStateBroadcaster =
                        new JointStateBroadcaster(
                                client,
                                joints,
                                Optional.of(() -> toJointState(dornaClient.getLastMotion())),
                                Duration.ofMillis(DEFAULT_BROADCASTER_RATE_IN_MILLIS));
                var controller =
                        new JointTrajectoryController(
                                client,
                                List.of(dornaJointStateSender),
                                joints,
                                initialPositions,
                                "/dorna2_arm_controller/joint_trajectory")) {
            jointStateBroadcaster.start();
            controller.start();
            CommandLineInterface.cli.askPressEnter();
        }
        // usually we need to close client once we are done
        // but here we keep it open so that subscriber will keep
        // printing messages indefinitely
    }

    private static JointState toJointState(Motion motion) {
        var j = motion.joints().toArrayOfRadians();
        return new JointState(
                Arrays.copyOf(j, joints.size()),
                Arrays.copyOf(new double[] {motion.vel()}, joints.size()));
    }
}
