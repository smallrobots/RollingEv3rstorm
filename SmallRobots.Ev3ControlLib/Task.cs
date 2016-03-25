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

using System;

namespace SmallRobots.Ev3ControlLib
{
    public abstract partial class Task
    {
        #region Fields
        /// <summary>
        /// The action for this Task
        /// </summary>
        protected Action<Robot> action;

        /// <summary>
        /// The name of the this Task
        /// </summary>
        protected string name;
        #endregion

        #region Properties
        /// <summary>
        /// Gets or sets the action for this Task
        /// The <Robot> parameter holds the Robot State on which the action works
        /// </summary>
        public Action<Robot> Action
        {
            get
            {
                return action;
            }
            set
            {
                action = value;
            }
        }

        /// <summary>
        /// Gets or sets the name of this task
        /// </summary>
        public string Name
        {
            get
            {
                return name;
            }
            set
            {
                name = value;
            }
        }
        #endregion
    }

    public partial class PeriodicTask : Task
    {
        #region Fields
        /// <summary>
        /// The period of this Periodic Task
        /// </summary>
        protected int period;
        #endregion

        #region Properties
        /// <summary>
        /// Gets or sets the Period of this Control Loop
        /// </summary>
        public int Period
        {
            get
            {
                return period;
            }
            set
            {
                period = value;
            }
        }
        #endregion

        #region Constructors
        /// <summary>
        /// Constructs an empty Periodic Control Loop
        /// </summary>
        public PeriodicTask() : base()
        {
            // Empty
        }

        /// <summary>
        /// Constructs a new Periodic Task with the specified parameters
        /// </summary>
        /// <param name="thePeriodicAction">The Action to be executed each period</param>
        /// <param name="thePeriod">The time expressed in milliseconds (ms) between each Action execution</param>
        public PeriodicTask(Action<Robot> thePeriodicAction, int thePeriod) : base()
        {
            action = thePeriodicAction;
            period = thePeriod;
        }

        /// <summary>
        /// Constructs a new Periodic Task with the specified parameters
        /// </summary>
        /// <param name="thePeriodicAction">The Action to be executed each period</param>
        /// <param name="thePeriod">The time expressed in milliseconds (ms) between each Action execution</param>
        /// <param name="theName">The name of the Periodic Task (used in execution logs)</param>
        public PeriodicTask(Action<Robot> thePeriodicAction, int thePeriod, string theName) : this(thePeriodicAction, thePeriod)
        {
            name = theName;
        }
        #endregion
    }

    public partial class ContinuousTask : Task
    {

    }
}
