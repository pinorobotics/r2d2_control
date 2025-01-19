/**
 * ROS2 Joint Trajectory Controller node for Dorna2 robotic arm.
 *
 * @see <a href="https://github.com/pinorobotics/r2d2">ros2dorna2</a>
 * @author aeon_flux aeon_flux@eclipso.ch
 */
module r2d2.control {
    requires jrosclient;
    requires jroscommon;
    requires jros2client;
    requires jros2control;
    requires jros2messages;
    requires id.xfunction;
    requires drac;
    requires io.opentelemetry.sdk.metrics;
    requires id.opentelemetry.exporters.pack;
    requires io.opentelemetry.sdk;
}
