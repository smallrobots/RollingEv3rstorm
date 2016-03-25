//////////////////////////////////////////////////////////////////////////////////////////////////
// Rolling Ev3rstorm                                                                            //
// Version 1.0                                                                                  //
//                                                                                              //
// Happily shared under the MIT License (MIT)                                                   //
//                                                                                              //
// Copyright(c) 2016 SmallRobots.it                                                             //
//                                                                                              //
// Permission is hereby granted, free of charge, to any person obtaining                        //
//a copy of this software and associated documentation files (the "Software"),                  //
// to deal in the Software without restriction, including without limitation the rights         //
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies             //
// of the Software, and to permit persons to whom the Software is furnished to do so,           //      
// subject to the following conditions:                                                         //
//                                                                                              //
// The above copyright notice and this permission notice shall be included in all               //
// copies or substantial portions of the Software.                                              //
//                                                                                              //
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,          //
// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR     //
// PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE           //
// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,          //
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE        //
// OR OTHER DEALINGS IN THE SOFTWARE.                                                           //
//                                                                                              //
// Visit http://wwww.smallrobots.it for tutorials and videos                                    //
//                                                                                              //
// References                                                                                   //
// Robot Square - Tutorial: Building BALANC3R                                                   //
// http://robotsquare.com/2014/06/23/tutorial-building-balanc3r/                                //
// and                                                                                          //
// Tomoaki Masuda                                                                               //
// http://www.moonmile.net/blog/archives/7525                                                   //
//////////////////////////////////////////////////////////////////////////////////////////////////

using SmallRobots.Ev3ControlLib;
using MonoBrickFirmware.Display;
using MonoBrickFirmware.Sensors;
using MonoBrickFirmware.UserInput;
using System.Threading;
using MonoBrickFirmware.Movement;
using System;
using System.Diagnostics;
using MonoBrickFirmware.Sound;

namespace SmallRobots.RollingEv3rstorm
{
    /// <summary>
    /// Rolling Ev3rstorm class
    /// </summary>
    public partial class RollingEv3rstorm : Robot
    {
        #region Fields
        public int speed;
        public int steering;

        /// <summary>
        /// Infrared Sensor (Port 4)
        /// </summary>
        public EV3IRSensor irSensor;

        /// <summary>
        /// Touch Sensor (Port 3)
        /// </summary>
        public EV3TouchSensor touchSensor;

        /// <summary>
        /// Head motor (Port B)
        /// </summary>
        public Motor headMotor;

        /// <summary>
        /// Gun Motor (Port C)
        /// </summary>
        public Motor gunMotor;

        /// <summary>
        /// True when the touch sensor is pressed
        /// </summary>
        bool touchSensorIsPressed;

        /// <summary>
        /// True when the escape button is pressed
        /// </summary>
        bool escapeButtonIsPressed;

        /// <summary>
        /// The beacon distance
        /// </summary>
        int beaconDistance;

        /// <summary>
        /// The beacon heading
        /// </summary>
        int beaconHeading;

        /// <summary>
        /// PID Task to trask the beacon with the robot head
        /// </summary>
        PIDController headBeaconTrackingPID;

        /// <summary>
        /// Signal to terminate the main program
        /// </summary>
        ManualResetEvent terminateProgram;

        /// <summary>
        /// Holds the last time of execution
        /// </summary>
        long lastExecutionTime;

        /// <summary>
        /// The pidstate
        /// </summary>
        PIDState ps;

        double SetPoint;
        double k1;
        double k2;
        double k3;
        double MaxPower;
        double MinPower;
        double LowPassConstant;
        double ProcessVariableSignal;
        sbyte OutputSignal;
        double kp;
        double ki;
        double kd;
        Stopwatch stopWatch;

        /// <summary>
        /// The balancing task
        /// </summary>
        BalancingTask balancingTask;
        #endregion

        #region Constructors
        /// <summary>
        /// Default constructor
        /// </summary>
        public RollingEv3rstorm() : base()
        {
            // Initializes sensors
            irSensor = new EV3IRSensor(SensorPort.In4, IRMode.Seek);
            touchSensor = new EV3TouchSensor(SensorPort.In3);

            // Initializes the motors
            headMotor = new Motor(MotorPort.OutB);
            headMotor.ResetTacho();

            gunMotor = new Motor(MotorPort.OutC);

            // Intializes the IO UpdateTask
            // TaskScheduler.Add(new PeriodicTask((Robot robot) => { IOUpdateTask(this); }, 25, "IOUpdateTask"));

            // Initializes the LCDUpdateTask
            // TaskScheduler.Add(new PeriodicTask((Robot robot) => { LCDUpdateTask(this); }, 200, "LCDUpdateTask"));

            // Initializes the BalancingTask
            balancingTask = new BalancingTask();
            TaskScheduler.Add(balancingTask);

            //// Initializes the Head PID Controller
            headBeaconTrackingPID = new PIDController(20);
            headBeaconTrackingPID.Kp = 1.7f;
            headBeaconTrackingPID.Ki = 0.002f;
            headBeaconTrackingPID.Kd = 0.5f;
            headBeaconTrackingPID.LowPassConstant = 1.0f;
            headBeaconTrackingPID.MaxPower = 75;
            headBeaconTrackingPID.MinPower = -75;
            // Append the periodic control loop to the robot scheduler
            // TaskScheduler.Add(thePeriodicTask: headBeaconTrackingPID);

            // Initializes the headBeaconTrackingTask
            TaskScheduler.Add(new PeriodicTask((Robot robot) => { HeadBeaconTrackingTask(this); }, 50, "HeadBeaconTrackingTask"));
            
            // Fields initialization
            lastExecutionTime = 0;
            ps = new PIDState();
            stopWatch = new Stopwatch();

            // PID Parameters
            kp = 1.5f;
            ki = 0.15f;
            kd = 0.0f;
            MaxPower = 50;
            MinPower = -MaxPower;
            LowPassConstant = 0.9f;

            UpdateKs();

            // Initialize the Main program termination signal
            terminateProgram = new ManualResetEvent(false);
        }
        #endregion

        #region Public Methods
        /// <summary>
        /// Starts the robot
        /// </summary>
        public void Start()
        {
            // Welcome messages
            LcdConsole.Clear();
            LcdConsole.WriteLine("*****************************");
            LcdConsole.WriteLine("*                           *");
            LcdConsole.WriteLine("*      SmallRobots.it       *");
            LcdConsole.WriteLine("*                           *");
            LcdConsole.WriteLine("*   Rolling Ev3rstorm 1.0   *");
            LcdConsole.WriteLine("*                           *");
            LcdConsole.WriteLine("*                           *");
            LcdConsole.WriteLine("*   Touch Sensor to start   *");
            LcdConsole.WriteLine("*   Escape to quit          *");
            LcdConsole.WriteLine("*                           *");
            LcdConsole.WriteLine("*****************************");

            // Busy wait for user
            bool touchSensorPressed = false;
            bool escapeButtonPressed = false;
            while (!(touchSensorPressed || escapeButtonPressed))
            {
                // Either the user presses the touch sensor, or presses the escape button
                // If users presses both, escape button will prevale
                touchSensorPressed = touchSensor.IsPressed();
                escapeButtonPressed = (Buttons.ButtonStates.Escape == Buttons.GetKeypress(new CancellationToken(true)));
            }

            if (escapeButtonPressed)
            {
                return;
            }

            if (touchSensorPressed)
            {
                LcdConsole.Clear();
                LcdConsole.WriteLine("*****************************");
                LcdConsole.WriteLine("*                           *");
                LcdConsole.WriteLine("*      SmallRobots.it       *");
                LcdConsole.WriteLine("*                           *");
                LcdConsole.WriteLine("*   Rolling Ev3rstorm 1.0   *");
                LcdConsole.WriteLine("*                           *");
                LcdConsole.WriteLine("*                           *");
                LcdConsole.WriteLine("*   Starting....            *");
                LcdConsole.WriteLine("*                           *");
                LcdConsole.WriteLine("*                           *");
                LcdConsole.WriteLine("*****************************");

                // Reset tachos
                headMotor.ResetTacho();

                // Acually starts the robot
                TaskScheduler.Start();

                // Reset the motors
                //Motor motorL = new Motor(MotorPort.OutA);
                //Motor motorR = new Motor(MotorPort.OutD);
                //motorL.Off();
                //motorR.Off();
                // Wait for termination
                // terminateProgram.WaitOne();
            }
        }
        #endregion

        #region Private Methods
        /// <summary>
        /// Updates Robot IO
        /// </summary>
        /// <param name="robot">The calling RollingEve3storm instance</param>
        void IOUpdateTask(RollingEv3rstorm robot)
        {
            // LcdConsole.WriteLine("IOUpdate");

            // Reads the inputs
            // Beacon
            BeaconLocation location = robot.irSensor.ReadBeaconLocation();
            beaconHeading = location.Location;
            beaconDistance = location.Distance;

            // Touch sensor
            touchSensorIsPressed = touchSensor.IsPressed();

            // Escape button
            escapeButtonIsPressed = (Buttons.ButtonStates.Escape == Buttons.GetKeypress(new CancellationToken(true)));
            if (escapeButtonIsPressed)
            {
                // Stop the scheduler
                robot.TaskScheduler.Stop();

                // Stops the motor
                robot.headMotor.Off();
            }

            // Commands the outputs
            //headBeaconTrackingPID.SetPoint = 0;
            //headBeaconTrackingPID.ProcessVariableSignal = beaconHeading;
            //headMotor.SetPower((sbyte)-headBeaconTrackingPID.OutputSignal);
        }

        /// <summary>
        /// Updates the LCD
        /// </summary>
        /// <param name="robot"></param>
        void LCDUpdateTask(RollingEv3rstorm robot)
        {
            // LcdConsole.WriteLine("LCDUpdate");
            LcdConsole.Clear();
            LcdConsole.WriteLine("Touch Pressed = " + robot.touchSensorIsPressed.ToString());
            LcdConsole.WriteLine("Escape pressed = " + robot.escapeButtonIsPressed.ToString());
            LcdConsole.WriteLine("Heading = " + robot.beaconHeading.ToString("F2"));
            LcdConsole.WriteLine("Distance = " + robot.beaconDistance.ToString("F2"));
        }

        private void UpdateKs()
        {
            k1 = ki + kp + kd;
            k2 = -kp - 2 * kd;
            k3 = kd;
        }

        void HeadBeaconTrackingTask(RollingEv3rstorm robot)
        {
            //const sbyte power = 30;
            //const byte numberOfSamples = 5;

            //double averageDirection = 0;
            //for (int i = 0; i < numberOfSamples; i++)
            //{
            //    averageDirection = +robot.irSensor.ReadBeaconLocation().Location;
            //}
            //averageDirection = averageDirection / numberOfSamples;

            //int tachoCount = robot.headMotor.GetTachoCount();
            //if (Math.Abs(averageDirection) > 3)
            //{
            //    if ((averageDirection > 0) && (tachoCount < 90))
            //    {
            //        headMotor.SetPower(power);
            //    }
            //    if ((averageDirection < 0) && (tachoCount > -90))
            //    {
            //        headMotor.SetPower(-power);
            //    }
            //}
            //else
            //{
            //    headMotor.SetSpeed(0);
            //}

            if (!stopWatch.IsRunning)
            {
                stopWatch.Start();
                // LcdConsole.WriteLine("Start Watch");
            }

            if (stopWatch.ElapsedMilliseconds > (lastExecutionTime + 1000))
            {
                lastExecutionTime = stopWatch.ElapsedMilliseconds;
                double avgPv = 0;
                int numberOfSamples = 10;
                for (int i = 0; i < numberOfSamples; i++)
                {
                    // LcdConsole.WriteLine("Sample " + i.ToString("F2"));
                    avgPv = avgPv + robot.irSensor.ReadBeaconLocation().Location;
                }
                SetPoint = robot.headMotor.GetTachoCount() + 2 * avgPv / numberOfSamples;
                // LcdConsole.WriteLine("ProcessVariableSignal " + ProcessVariableSignal.ToString("F2"));
            }

            ProcessVariableSignal = robot.headMotor.GetTachoCount();

            // Updating first order low pass filter "memory" variables
            ps.FilteredProcessVariableK1 = ps.FilteredProcessVariable;

            // updating "memory" variables
            ps.ErrorK2 = ps.ErrorK1;
            ps.ErrorK1 = ps.Error;

            // Filtering the process Variable
            ps.FilteredProcessVariable = ps.FilteredProcessVariableK1 + LowPassConstant * (ProcessVariableSignal - ps.FilteredProcessVariable);

            // updating error
            ps.Error = SetPoint - ps.FilteredProcessVariable;

            // Updating robot steering
            steering =  (int) ps.FilteredProcessVariable;
            if ((Math.Abs(ps.Error) < 10) && (robot.irSensor.ReadBeaconLocation().Distance > 20))
            {
                speed = 40;   
            }
            else
            {
                speed = 0;
            }

            // Fire if close enough

            if ((Math.Abs(ps.Error) < 10) && (robot.irSensor.ReadBeaconLocation().Distance <= 20)
                && (robot.irSensor.ReadBeaconLocation().Distance > 0))
            {
                gunMotor.SetPower(100);
            }
            else
            {
                gunMotor.SetPower(0);
            }

                // computating delta_u and u
                ps.DeltaControlSignal = k1 * ps.Error + k2 * ps.ErrorK1 + k3 * ps.ErrorK2;
            ps.ControlSignal = ps.ControlSignal + ps.DeltaControlSignal;

            // Saturatin u
            if (ps.ControlSignal > MaxPower)
                ps.ControlSignal = MaxPower;
            if (ps.ControlSignal < MinPower)
                ps.ControlSignal = MinPower;

            OutputSignal = (sbyte)ps.ControlSignal;
            // LcdConsole.WriteLine("OutputSignal " + ProcessVariableSignal.ToString("F2"));
            try
            {
                robot.headMotor.SetPower(OutputSignal);
            }
            catch (Exception ex)
            {
               LcdConsole.WriteLine(ex.Message);
            }
            // LcdConsole.WriteLine("Power out!");
        }
        #endregion
    }

    /// <summary>
    /// Task that turns the robot head to track the IR Beacon
    /// </summary>
    public partial class HeadBeaconTrackingTask : PeriodicTask
    {

        #region Fields
        /// <summary>
        /// Holds the last time of execution
        /// </summary>
        long lastExecutionTime;

        /// <summary>
        /// The pidstate
        /// </summary>
        PIDState ps;

        double SetPoint;
        double k1;
        double k2;
        double k3;
        double MaxPower;
        double MinPower;
        double LowPassConstant;
        double ProcessVariableSignal;
        sbyte OutputSignal;
        double kp;
        double ki;
        double kd;
        Stopwatch stopWatch;
        #endregion

        #region Constructors
        /// <summary>
        /// Default constructor
        /// </summary>
        public HeadBeaconTrackingTask() : base()
        {
            // Base class initialization
            Name = "HeadBeaconTrackingTask";
            Period = 20;
            Action = theTask;

            // Fields initialization
            lastExecutionTime = 0;
            ps = new PIDState();
            stopWatch = new Stopwatch();

            // PID Parameters
            kp = 2.5f;
            ki = 0.0f;
            kd = 0.0f;
            MaxPower = 30;
            MinPower = -MaxPower;
            LowPassConstant = 0.9f;

            UpdateKs();
        }
        #endregion

        #region Private methods
        private void UpdateKs()
        {
            k1 = ki + kp + kd;
            k2 = -kp - 2 * kd;
            k3 = kd;
        }

        private void theTask(Robot theRobot)
        {
            RollingEv3rstorm Ev = (RollingEv3rstorm)theRobot;

            if (!stopWatch.IsRunning)
            {
                stopWatch.Start();
                LcdConsole.WriteLine("Start Watch");
            }

            if (stopWatch.ElapsedMilliseconds > (lastExecutionTime + 1000))
            {
                lastExecutionTime = stopWatch.ElapsedMilliseconds;
                double avgPv = 0;
                int numberOfSamples = 10;
                for (int i = 0; i < numberOfSamples; i++)
                {
                    LcdConsole.WriteLine("Sample " + i.ToString("F2"));
                    avgPv = avgPv + Ev.irSensor.ReadBeaconLocation().Location;
                }
                ProcessVariableSignal = Ev.headMotor.GetTachoCount() + avgPv / numberOfSamples;
                LcdConsole.WriteLine("ProcessVariableSignal " + ProcessVariableSignal.ToString("F2"));
            }
            
            // Updating first order low pass filter "memory" variables
            ps.FilteredProcessVariableK1 = ps.FilteredProcessVariable;

            // updating "memory" variables
            ps.ErrorK2 = ps.ErrorK1;
            ps.ErrorK1 = ps.Error;

            // Filtering the process Variable
            ps.FilteredProcessVariable = ps.FilteredProcessVariableK1 + LowPassConstant * (ProcessVariableSignal - ps.FilteredProcessVariable);

            // updating error
            ps.Error = SetPoint - ps.FilteredProcessVariable;

            // computating delta_u and u
            ps.DeltaControlSignal = k1 * ps.Error + k2 * ps.ErrorK1 + k3 * ps.ErrorK2;
            ps.ControlSignal = ps.ControlSignal + ps.DeltaControlSignal;

            // Saturatin u
            if (ps.ControlSignal > MaxPower)
                ps.ControlSignal = MaxPower;
            if (ps.ControlSignal < MinPower)
                ps.ControlSignal = MinPower;

            OutputSignal = (sbyte)-ps.ControlSignal;
            LcdConsole.WriteLine("OutputSignal " + ProcessVariableSignal.ToString("F2"));
            try {
                Ev.headMotor.SetPower(OutputSignal);
            }
            catch (Exception ex)
            {
                LcdConsole.WriteLine(ex.Message);
            }
            LcdConsole.WriteLine("Power out!");
        }
        #endregion

    }

    /// <summary>
    /// The task that balnces the robot on its wheels
    /// </summary>
    public partial class BalancingTask : PeriodicTask
    {
        #region Fields
        bool initialized;

        /// <summary>
        /// The interval of time between each execution
        /// </summary>
        double deltaT;

        /// <summary>
        /// Wheel radius
        /// </summary>
        double wheelRadius;

        /// <summary>
        /// Type of gyro sensor
        /// 0: Lego 
        /// 1: HiTechnic 
        /// 2: Dexter
        /// </summary>
        int gyroType;

        /// <summary>
        /// Left motor (Port A)
        /// </summary>
        Motor motorL;

        /// <summary>
        /// Right motor (Port D)
        /// </summary>
        Motor motorR;

        /// <summary>
        /// Speaker
        /// </summary>
        Speaker speaker;

        /// <summary>
        /// Gyro sensor (Port 3)
        /// </summary>
        EV3GyroSensor gyroSensor;

        /// <summary>
        /// Commanded speed
        /// </summary>
        double speed = 0;

        /// <summary>
        /// Commanded differential speed
        /// </summary>
        double steering = 0;

        /// <summary>
        /// Previous steering differential speed
        /// </summary>
        double _old_steering = 0.0;

        /// <summary>
        /// PID Proportional constant
        /// </summary>
        double kp;

        /// <summary>
        /// PID Integral constant
        /// </summary>
        double ki;

        /// <summary>
        /// PID Derivative constant
        /// </summary>
        double kd;

        /// <summary>
        /// The weight for the robot angular velocity toward the vertical axis
        /// </summary>
        double gainAngularVelocity;

        /// <summary>
        /// The weight for the robot angle toward the vertical axis
        /// </summary>
        double gainAngle;

        /// <summary>
        /// The weight for the motor speed
        /// </summary>
        double gainMotorSpeed;

        /// <summary>
        /// The weight for the motor (angular) position
        /// </summary>
        double gainMotorPosition;

        /// <summary>
        /// The position reference. It's updated at each task iteration and represents where
        /// the robot has to stop as if its trajectory was linear
        /// </summary>
        double positionReference;

        /// <summary>
        /// Average gyro angular velocity
        /// </summary>
        double meanGyroAngularVelocity = 0;

        /// <summary>
        /// Average gyro angle
        /// </summary>
        double gyroAngle = 0;

        /// <summary>
        /// Index on the array used to compute the moving average fo wheel speed valued
        /// </summary>
        int _enc_index = 0;

        /// <summary>
        /// Array used to store the samples needed to compute the moving average on wheel speed values
        /// </summary>
        double[] _enc_val = new double[8];

        // Action to be performed when the cycle is arrested due to an exception
        public event Action OnStop;

        /// <summary>
        /// PID accumulated error
        /// </summary>
        double acc_err = 0;

        /// <summary>
        /// PID error at step k-1
        /// </summary>
        double prev_err = 0;

        /// <summary>
        /// True if the power is out of bound
        /// </summary>
        bool outOfBoundNow = false;

        /// <summary>
        /// True if the power was out of bound at the last step
        /// </summary>
        bool outOfBoundAtTheLastStep= false;

        /// <summary>
        /// Number of consecutive detected bound violation
        /// </summary>
        double outOfBoundsDetected = 0;
        #endregion

        #region Public Methods (check if must be converted to private)
        /// <summary>
        /// Main balancing task
        /// </summary>
        /// <param name="state">Robot state, it is used to exchange data with the robot public data structure</param>
        public void OnBalanceLoop(Robot robot)
        {
            if (!initialized)
            {
                initialized = true;

                double GAIN_ANGLE_VELOCITY = 1.3;
                int GAIN_ANGLE = 25;
                int GAIN_WHEEL_SPEED = 75;
                int GAIN_WHEEL_POSITION = 350;

                // Initialize the balance loop controller
                Initialize(
                    0,  // LEGO Jyro 
                    42, // wheel 42 mm
                    22);// sample time 22msec

                // Set controller feedback gains
                SetConstants(
                    0.6,    // Kp
                    14,     // Ki
                    0.005,  // Kd
                    GAIN_ANGLE_VELOCITY,    // Gain Angular Velocity 
                    GAIN_ANGLE,             // Gain Angle    
                    GAIN_WHEEL_SPEED,       // Gain Wheel speed 
                    GAIN_WHEEL_POSITION);   // Gain Wheel position 
            }

            try
            {
                double robotPosition = 0.0;
                double robotSpeed = 0.0;
                double angle = 0.0;
                double angleVelocity = 0.0;

                // requested steering and speed
                steering = ((RollingEv3rstorm)robot).steering;
                speed = ((RollingEv3rstorm)robot).speed;

                // Estimated motors angular position
                double motorPosition = EvaluateReferencePosition();

                // Reads actual motors angular position and speed
                ReadEncoders(out robotPosition, out robotSpeed);

                // Reads gyro values
                ReadGyro(out angle, out angleVelocity);

                // Combines sensor values
                double inputVal = CombineSensorValues(angleVelocity, angle, robotSpeed, robotPosition, motorPosition);

                // Evaluates the PID controller outputs
                double pid = PID(kp, ki, kd, deltaT, 0.0, inputVal);

                // Evaluates errors magnitude to eventually raise an exception
                Errors(pid);

                // Applies power to the motors
                SetMotorPower(steering, pid);
            }
            catch (Exception ex)
            {
                if (robot == null)
                {
                    LcdConsole.WriteLine("Robot is null");
                }
                LcdConsole.WriteLine(ex.Message);

                // Stop routine
                if (OnStop != null)
                {
                    Font font = Font.MediumFont;
                    Lcd.Update();
                    Lcd.WriteText(font, new MonoBrickFirmware.Display.Point(3, 5), "ERROR", false);

                    // Stop the timer
                    // balanceLoopTimer.Stop();

                    // Stop the timer
                    OnStop();
                }
                robot.TaskScheduler.Stop();
                motorL.Off();
                motorR.Off();
            }
        }

        /// <summary>
        /// Reads wheels encoders to compute angular speed and angular position
        /// </summary>
        public void ReadEncoders(out double angularPosition, out double angularSpeed)
        {
            double motorSpeed = GetMotorSpeed();
            angularSpeed = (wheelRadius * motorSpeed) / 57.3;      

            double tc = (motorL.GetTachoCount() + motorR.GetTachoCount()) / 2.0;
            angularPosition = (wheelRadius * tc) / 57.3;
        }

        /// <summary>
        /// Reads gyro angle and gyro angular velocity
        /// </summary>
        /// <param name="angle">Gyro angle</param>
        /// <param name="angleVelocity">Gyro angular speed</param>
        public void ReadGyro(out double angle, out double angleVelocity)
        {
            double curr_val = GyroRate();
            meanGyroAngularVelocity = (meanGyroAngularVelocity * (1.0 - deltaT * 0.2) + (curr_val * deltaT * 0.2));
            double ang_vel = curr_val - meanGyroAngularVelocity;
            gyroAngle += deltaT * ang_vel;
            // 戻り値
            angle = gyroAngle;
            angleVelocity = ang_vel;
        }
        #endregion

        #region Private Methods
        /// <summary>
        /// Throws an Exception if too many regulation errors are detected
        /// CAUTION: the robot will fall is the exception is thrown, if it is not already fallen, ;-D
        /// </summary>
        /// <param name="averagePower">Average applied power</param>
        private void Errors(double averagePower)
        {
            outOfBoundNow = Math.Abs(averagePower) > 100.0;
            if (outOfBoundNow & outOfBoundAtTheLastStep)
            {
                outOfBoundsDetected++;
            }
            else
            {
                outOfBoundsDetected = 0;
            }
            if (outOfBoundsDetected > 20)
            {
                Thread.Sleep(100);
                motorL.Off();
                motorR.Off();
                Lcd.Clear();
                Font font = Font.MediumFont;
                Lcd.WriteText(font, new MonoBrickFirmware.Display.Point(3, 5), "ERROR", false);
                speaker.PlayTone(800, 100, 50);
                speaker.PlayTone(600, 100, 50);
                speaker.PlayTone(300, 100, 50);
                Thread.Sleep(4000);
                throw new Exception("ERROR");
            }
            else
            {
                outOfBoundAtTheLastStep = outOfBoundNow;
            }
        }

        /// <summary>
        /// Saturate a signal x to its admitted lower and upper bound
        /// </summary>
        /// <param name="x">signal to saturate</param>
        /// <param name="lower">lower bound</param>
        /// <param name="upper">upper bound</param>
        /// <returns></returns>
        private double Limit(double x, double lower, double upper)
        {
            if (x < lower) return lower;
            if (x > upper) return upper;
            return x;
        }

        /// <summary>
        /// Computes the power to apply to the motors
        /// motorR : portA
        /// motorL : portD
        /// </summary>
        /// <param name="steering">Steering power</param>
        /// <param name="averagePower">Average power</param>
        private void SetMotorPower(double steering, double averagePower)
        {
            double new_steering = Limit(steering, -50, 50);
            double extra_pwr = 0.0;

            extra_pwr = new_steering * -0.5;

            double power_c = averagePower - extra_pwr;
            double power_b = averagePower + extra_pwr;
            _old_steering = new_steering;

            double pwrA = power_b * 0.021 / wheelRadius;
            double pwrD = power_c * 0.021 / wheelRadius;
            motorR.SetPower((sbyte)pwrA);
            motorL.SetPower((sbyte)pwrD);
        }

        /// <summary>
        /// The PID algorithm
        /// </summary>
        /// <param name="kp">The PID proportional constant</param>
        /// <param name="ki">The PID integrative constant</param>
        /// <param name="kd">The PID derivative constant</param>
        /// <param name="sampleTime">The sample time</param>
        /// <param name="referenceVal">The reference signal</param>
        /// <param name="inputVal">The process signak</param>
        /// <returns>The computed control signal</returns>
        private double PID(double kp, double ki, double kd, double sampleTime, double referenceVal, double inputVal)
        {
            double curr_err = inputVal - referenceVal;
            acc_err = acc_err + curr_err * deltaT;
            double dif_err = (curr_err - prev_err) / deltaT;
            double c = kp * curr_err;
            double b = ki * acc_err;
            double a = kd * dif_err;
            double output = a + b + c;
            prev_err = curr_err;
            return output;
        }

        /// <summary>
        /// Combine the sensor values for wheighted full state feedback
        /// </summary>
        /// <param name="gyroAngularSpeed"></param>
        /// <param name="gyroAngle"></param>
        /// <param name="robotSpeed"></param>
        /// <param name="robotPosition"></param>
        /// <param name="motorPosition"></param>
        /// <returns>The combined value</returns>
        private double CombineSensorValues(double gyroAngularSpeed, double gyroAngle, double robotSpeed, double robotPosition, double motorPosition)
        {
            double c = gainMotorPosition * (robotPosition - motorPosition);
            double d = gainMotorSpeed * robotSpeed;
            double a = gainAngle * gyroAngle;
            double b = gainAngularVelocity * gyroAngularSpeed;
            double w = a + b + c + d;
            return w;
        }

        /// <summary>
        /// The average wheel motor speed is computed as a moving average on 8 samples
        /// </summary>
        /// <returns>The average wheel speed</returns>
        private double GetMotorSpeed()
        {
            // Updates the array
            _enc_index++;
            if (_enc_index >= _enc_val.Length) _enc_index = 0;
            int compare_index = _enc_index + 1;
            if (compare_index >= _enc_val.Length) compare_index = 0;
            int ave = (motorL.GetTachoCount() + motorR.GetTachoCount()) / 2;
            _enc_val[_enc_index] = ave;

            // Computes the average speed
            double speed = (_enc_val[_enc_index] - _enc_val[compare_index]) / (_enc_val.Length * deltaT);
            return speed;
        }
        /// <summary>
        /// Evaluates the reference position based on the commanded speed
        /// </summary>
        /// <returns>The reference position</returns>
        private double EvaluateReferencePosition()
        {
            positionReference += deltaT * speed * 0.002;
            return positionReference;
        }

        /// <summary>
        /// Gets the gyro rate as the average value of 10 samples 
        /// Throws an exception if the sensor type is not Lego
        /// </summary>
        /// <returns>The gyro raye</returns>
        private int GyroRate()
        {
            // Only for the LEGO sensor
            if (gyroType == 0)
            {
                int sum = 0;
                for (int i = 0; i < 10; i++)
                {
                    sum += gyroSensor.Read();
                }
                return sum / 10;
            }
            else
            {
                throw new ArgumentException("LEGO Gyro sensor only");
            }
        }

        /// <summary>
        /// Sets constants initial values
        /// </summary>
        /// <param name="kp">The PID proportional constant</param>
        /// <param name="ki">The PID integrative constant</param>
        /// <param name="kd">The PID derivative constant</param>
        /// <param name="angleVelocityGain">Gain for the gyro angular velocity</param>
        /// <param name="angleGain">Gain for gyro angle</param>
        /// <param name="wheelSpeedGain">Gain for the wheel angular velocity</param>
        /// <param name="wheelPositionGain">Gain for the wheel angle</param>
        private void SetConstants(double kp, double ki, double kd, double angleVelocityGain, int angleGain, int wheelSpeedGain, int wheelPositionGain)
        {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
            gainAngularVelocity = angleVelocityGain;
            gainAngle = angleGain;
            gainMotorSpeed = wheelSpeedGain;
            gainMotorPosition = wheelPositionGain;
        }

        /// <summary>
        /// Calibrates the gyro sensor
        /// </summary>
        /// <returns>meanSpeed</returns>
        private double Calibrate()
        {
            // Play a tone to signal the start of the calibraion procedure
            speaker.PlayTone(440, 100, 10);
            Thread.Sleep(100);
            meanGyroAngularVelocity = 0;

            // takes 20 angle rates from the gyro sensor and computes the average angle rate value
            for (int i = 0; i < 20; i++)
            {
                meanGyroAngularVelocity += GyroRate();
                Thread.Sleep(5); // 
            }

            meanGyroAngularVelocity = meanGyroAngularVelocity / 20.0;
            Thread.Sleep(100);  // 

            // Plays two tones to signal the end of the calibration procedure
            speaker.PlayTone(440, 100, 10);
            Thread.Sleep(100);
            speaker.PlayTone(440, 100, 10);

            return meanGyroAngularVelocity;
        }

        /// <summary>
        /// Initialize the specified gyro sensor type, the wheel diameter and the sample time.
        /// </summary>
        /// <param name="chooseSensor">gyro sensor type (0: Lego, 1: HiTechnic, 2:Dexter</param>
        /// <param name="wheelDiameter">Wheel diameter (mm)</param>
        /// <param name="sampleTime">Sample time (ms)</param>
        private void Initialize(int gyroSensorType, int wheelDiameter, int sampleTime)
        {
            deltaT = (sampleTime - 2) / 1000.0;
            wheelRadius = wheelDiameter / 2000.0;
            gyroType = gyroSensorType;

            // Reset the motors
            motorL.Off();
            motorR.Off();
            motorL.ResetTacho();
            motorR.ResetTacho();

            // Resete the position reference
            positionReference = 0;

            // wait 0.1s
            Thread.Sleep(100);

            // Calibrate the gyro sensor
            meanGyroAngularVelocity = this.Calibrate();
            speed = 0;
            steering = 0;
        }
        #endregion

        #region Constructors
        /// <summary>
        /// Default Constructor
        /// </summary>
        public BalancingTask() : base()
        {
            speaker = new Speaker(100);
            motorL = new Motor(MotorPort.OutD);
            motorR = new Motor(MotorPort.OutA);
            gyroSensor = new EV3GyroSensor(SensorPort.In2);
            gyroSensor.Mode = GyroMode.AngularVelocity;

            // Set the action
            Action = OnBalanceLoop;

            // Set the period (20ms)
            Period = 20;
        }
        #endregion
    }
}
