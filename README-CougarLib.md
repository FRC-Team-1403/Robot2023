## CougarLib Overview

CougarLib is a collection of reusable classes and components for
programming [FRC](https://www.firstinspires.org/robotics/frc) robots
in Java using the standard
[FRC CcontrolSystem](https://docs.wpilib.org/en/stable/index.html)
and java [WPILib](https://docs.wpilib.org/en/stable/docs/software/what-is-wpilib.html).

CougarLib is designed to provide an additional abstraction layer to
provide clarity and reduce the complexity of non-trivial robots. It
provides an opinionated model that encourages practices that should
make code more consistent, readable, and testable but parts can be
used independent of the model since it is built on top of the standard
WPILib implementation.


---

## Cougar Robot Model

<table>
<tr><th>Entity</th><th>Summary</th></tr>
<tr><td>CougarLibInjectedParameters</td>
    <td>Allows the runtime environment to [inject
        dependencies](https://en.wikipedia.org/wiki/Dependency_injection)
        into the robot implementation so that custom robots can be thoroughly
        unit tested in a straight-forward way.
        <p>
        The framework will automatically populate these parameters when
        running onboard a real robot or simulation. Unit tests can tailor
        their own parameters to meet their needs for the type of test they
        are trying to perform.
    </td></tr>
<tr><td>CougarRobot</td>
    <td>Provides the abstraction for the robots that focuses on its unique
        behavior and capabilities.
        <p>
        Robot programs will create a custom subclass of `CougarRobot`, usually
        called `CougarRobotImpl` but the actual name is up to the robot program.
    </td></tr>
<tr><td>RobotConfig</td>
    <td>Provides the configuration parameters for robots so they can be tuned
        and experimented on without having to recompile and redeploy the robot
        program.
        <p>
        Since these are specific to individual robots, this class is
        typically introduced by custom Robot programs. The concept is
        part of the Cougar Robot Model, but is at a higher level of abstraction
        than the CougarLib itself.
    </td></tr>
<tr><td>CougarSubsystem</td>
    <td>Provides the abstraction for decomposing robots into their logical
        subsystems.
        <p>
        Typically CougarRobots (the `CougarRobotImpl` for a custom robot)
        are composed of one or more subsystems. These are each specialzed
        classes derived from `CougarSubsystem`.
    </td></tr>
<tr><td>Device</td>
    <td>Provides the abstraction for the hardware sensors and actuators that
        robots use to interact with the physical world. These try to leverage
        existing WPI components and interfaces where possible, but adds more
        consistent support for testing and instrumentation.
        <p>
        Each type of actuator and sensor introduces a specialized Device
        interface and one or more implementation classes. Devices that are
        interacting with a physical hardware component will often have one or
        more additional implementations that are suitable for unit testing
        without the physical hardware
        (e.g. "[fakes](https://en.wikipedia.org/wiki/Mock_object)")
    </td></tr>
<tr><td>DeviceFactory</td>
    <td>An abstraction for creating devices that hides the concrete device
        classes allowing tests to inject alternative implementations they can
        control and observe to verify the robot code is running as
        expected.
        </td></tr>
<tr><td>CougarCommand</td>
    <td>An alertnative base class for commands that utilizzes the programming
        model and practices encouraged within CougarLib.
        <p>
        Programmers typically write customized commands that implement the
        different actions that their robots can perform so that these can
        be automated, scripted, or controlled by human operators.
    </td></tr>
<tr><td>CougarScript</td>
    <td>A scripting language for writing robot "subroutines" as sequences of
        Commands, editable without having to recompile or redeploy the
        robot.
    </td></tr>
<tr><td>WpiLibAdapter</td>
    <td>A refinement of the WPI BaseRobot that integrates CougarRobots with
        the WPI framework.
        <p>
        From CougarLib's perspective the adapter provides some platform level
        services and is the gateway to the [Driver Station](https://docs.wpilib.org/en/stable/docs/software/driverstation/driver-station.html) and [Field
        Management System](https://wpilib.screenstepslive.com/s/fms/m/whitepaper/l/608744-fms-whitepaper),
        [seperating these concerns](https://en.wikipedia.org/wiki/Separation_of_concerns)
        from the CougarRobot whose focus is on playing a particular game.
    </td></tr>
</table>


---

## CougarLib Packages

All packages within cougar lib is in a package prefixed by `team1403.lib`.
In this section we will refer to the packages within CougarLib without this
ubiquitous prefix (e.g. `core` refers to `team1403.lib.core`).

Cougar lib packages ending with ".test" contain components only intended for
tests. If these turn out to be useful outside tests then they should be moved
into the prefix package. Tests for the package code itself are in the
`src/test` repository path. The difference is that the ".test" packages are
still available to other repositories as part of the CougarLib public interface.
It's just that those components are not intended for robot runtimes.


Package | Purpose
--------|--------
core | Defines most of the core concepts within the model.
device | Defines the device interfaces.
device.test | Defines test-only device implementations.
device.virtual | Defines device implementations that do not use hardware so are usable in both real and test environments.
device.wpi | Defines device implementations that rely on WPI library devices (or third party devices that are essentially extensions to the WPI library.
util | Defines other components useful to programming robots.
util.test | Defines other components only useful when writing tests.


---

## Other CougarLib Features

* `/buildSrc` Contains build rules and task configuration information.
  This includes configuration for
  [checkstyle](https://github.com/checkstyle/checkstyle)
  and [PMD](https://pmd.github.io/) and Java compilation
  rules that reports on [java code coverage](https://github.com/jacoco/jacoco).

* `/docs` Documentation for maintaining and for using CougarLib.


## Getting Help

If the `/docs` section does not provide the information you need,
You can post questions on one of our
[Discussion Boards](https://github.com/FRC-Team-1403/PublicCougarRobotTemplate/discussions)
such as the
[Q&A](https://github.com/FRC-Team-1403/PublicCougarRobotTemplate/discussions/categories/q-a).


## Making Suggestions and Providing Feedback

You can make suggestions or comments on the
[Discussion Boards](https://github.com/FRC-Team-1403/PublicCougarRobotTemplate/discussions)
such as [Ideas](https://github.com/FRC-Team-1403/PublicCougarRobotTemplate/discussions/categories/ideas) or [General](https://github.com/FRC-Team-1403/PublicCougarRobotTemplate/discussions/categories/general)



