//////////////////////////////////////////////////////////////////////////////////////////////////
// Ev3 Control Library (Ev3ControlLib)                                                          //
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
//////////////////////////////////////////////////////////////////////////////////////////////////

using MonoBrickFirmware.Display;
using System;
using System.Threading;

namespace SmallRobots.Ev3ControlLib
{
    /// <summary>
    /// Proportional Integrative Derivative (PID) Controller
    /// </summary>
    public partial class PIDController : PeriodicTask
    {
        #region Properties
        /// <summary>
        /// Gets or sets the input used for PID calculation
        /// </summary>
        /// <value>The input.</value>
        public float ProcessVariableSignal { get; set; }

        protected sbyte outputSignal;
        /// <summary>
        /// Gets the the control signal of the PID controller
        /// </summary>
        /// <value>The output.</value>
        public virtual sbyte OutputSignal
        {
            get
            {
                return outputSignal;
            }
            protected set
            {
                if (outputSignal != value)
                {
                    outputSignal = value;
                }
            }
        }

        /// <summary>
        /// Get or sets the SetPoint used for regulation
        /// </summary>
        public float SetPoint { get; set; }

        /// <summary>
		/// Get or sets the minimum power
		/// or in other words the minimum allowable
		/// value for the control signal
		/// </summary>
		/// <value>The minimum power.</value>
		public sbyte MinPower { set; get; }

        /// <summary>
        /// Get or sets the maximum power
        /// or in other words the maximum allowable
        /// value for the control signal
        /// </summary>
        /// <value>The max power.</value>
        public sbyte MaxPower { get; set; }


        private float kp;
        /// <summary>
        /// Gets or sets the proportional constant
        /// </summary>
        /// <value>The Kp</value>
        /// <exception cref="ArgumentException">Thrown if value is non positive</exception>
        public float Kp
        {
            get
            {
                return kp;
            }
            set
            {
                if (value < 0)
                {
                    ArgumentException ex = new ArgumentException("The proportional constant must be a positive number");
                    throw (ex);
                }
                else if (kp != value)
                {
                    kp = value;
                    UpdateKs();
                }
            }
        }

        private float ki;
        /// <summary>
        /// Get or sets the integral constant
        /// </summary>
        /// <value>The Ki</value>
        /// <exception cref="ArgumentException">Thrown if value is non positive</exception>
        public float Ki
        {
            get
            {
                return ki;
            }
            set
            {
                if (value < 0)
                {
                    ArgumentException ex = new ArgumentException("The integral constant must be a positive number");
                    throw (ex);
                }
                else if (ki != value)
                {
                    ki = value;
                    UpdateKs();
                }
            }
        }

        private float kd;
        /// <summary>
        /// Gets or sets the derivative constant
        /// </summary>
        /// <value>The Kd</value>
        /// <exception cref="ArgumentException">Thrown if value is non positive</exception>
        public float Kd
        {
            get
            {
                return kd;
            }
            set
            {
                if (value < 0)
                {
                    ArgumentException ex = new ArgumentException("The derivative constant must be a positive number");
                    throw (ex);
                }
                else if (kd != value)
                {
                    kd = value;
                    UpdateKs();
                }
            }
        }

        private float lowPassConstant;
        /// <summary>
        /// Gets or sets the constant for the IIR (infinite impulse response)
        /// low pass filter to be applied to the ProcessVariableSignal signals
        /// </summary>
        /// <value>The low pass constant</value>
        public float LowPassConstant
        {
            get
            {
                return lowPassConstant;
            }
            set
            {
                if ((value <= 0) || (value > 1))
                {
                    ArgumentException ex = new ArgumentException("The low pass filter constant must be between 0 and 1");
                    throw (ex);
                }
                else
                {
                    lowPassConstant = value;
                }
            }
        }
        #endregion

        #region Fields
        /// <summary>
        /// Constants used int the "speed form" of the discrete PID algorithm
        /// </summary>
        private float k1;
        private float k2;
        private float k3;
        private PIDState ps;
        #endregion

        #region Constructors
        /// <summary>
        /// Creates an empty PID Controller with a default sample time of 10ms
        /// </summary>
        public PIDController()
        {
            // Fields initialization
            init();
        }

        /// <summary>
        /// Constructs a PID Controller with the specified period expressed in milliseconds (ms)
        /// </summary>
        /// <param name="thePeriod">The period (sample time)</param>
        public PIDController(int thePeriod) : base()
        {
            // Fields initialization
            init();
            Period = thePeriod;
        }

        /// <summary>
        /// Fields initialization
        /// </summary>
        private void init()
        {
            Ki = 0;
            Kp = 0;
            Kd = 0;
            lowPassConstant = 0;
            ps = new PIDState();
            action = new Action<Robot>((Robot robot) => PIDAlgorithm(robot));
        }
    #endregion

        #region Private Methods
        /// <summary>
        /// Updates the K1, K2 and K3 constants
        /// </summary>
        private void UpdateKs()
            {
                k1 = ki + kp + kd;
                k2 = -kp - 2 * kd;
                k3 = kd;
            }

        /// <summary>
        /// The core of the PID alogrithm computation
        /// </summary>
        protected virtual void PIDAlgorithm(Robot robot)
        {
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

            OutputSignal = (sbyte)ps.ControlSignal;
        }
        #endregion

    }

    /// <summary>
    /// This class holds the state of a PID controller at instant k
    /// </summary>
    public partial class PIDState
    {
        #region Properties
        private double error;
        /// <summary>
        /// Gets or sets the error at instant k
        /// </summary>
        /// <value>The error.</value>
        public double Error
        {
            get
            {
                return error;
            }
            set
            {
                error = value;
            }
        }

        private double errorK1;
        /// <summary>
        /// Gets or sets the error at instant k-1.
        /// </summary>
        /// <value>The error k1.</value>
        public double ErrorK1
        {
            get
            {
                return errorK1;
            }
            set
            {
                errorK1 = value;
            }
        }

        private double errorK2;
        /// <summary>
        /// Gets or sets the error at instant k-2.
        /// </summary>
        /// <value>The error k2.</value>
        public double ErrorK2
        {
            get
            {
                return errorK2;
            }
            set
            {
                errorK2 = value;
            }
        }

        private double controlSignal;
        /// <summary>
        /// Gets or sets the control singal at instant k
        /// </summary>
        public double ControlSignal
        {
            get
            {
                return controlSignal;
            }
            set
            {
                controlSignal = value;
            }
        }

        private double deltaControlSignal;
        /// <summary>
        /// Gets or sets the control signal increment at instant k
        /// </summary>
        /// <value>The delta control signal.</value>
        public double DeltaControlSignal
        {
            get
            {
                return deltaControlSignal;
            }
            set
            {
                deltaControlSignal = value;
            }
        }

        private double filteredProcessVariableK1;
        /// <summary>
        /// Gets or sets the Filtered Process Variable at instant k-1
        /// </summary>
        /// <value>The Filtered Process Variable.</value>
        public double FilteredProcessVariableK1
        {
            get
            {
                return filteredProcessVariableK1;
            }
            set
            {
                filteredProcessVariableK1 = value;
            }
        }

        private double filteredProcessVariable;
        /// <summary>
        /// Gets or sets the Filtered Process Variable ar instant k
        /// </summary>
        /// <value>The Filtered Process Variable.</value>
        public double FilteredProcessVariable
        {
            get
            {
                return filteredProcessVariable;
            }
            set
            {
                filteredProcessVariable = value;
            }
        }


        #endregion

        #region Constructors
        /// <summary>
        /// Initializes a new instance of the <see cref="SmallRobots.Controllers.PIDState"/> class.
        /// </summary>
        public PIDState()
        {
            // Initialize the properties
            error = 0.0f;
            errorK1 = 0.0f;
            errorK2 = 0.0f;
            controlSignal = 0.0f;
            deltaControlSignal = 0.0f;
            filteredProcessVariableK1 = 0.0f;
            filteredProcessVariable = 0.0f;

        }
        #endregion
    }
}
