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

using System.Threading;
using SmallRobots.Ev3ControlLib.Menu;
using MonoBrickFirmware.Display;
using MonoBrickFirmware.Movement;
using MonoBrickFirmware.Display.Menus;

namespace SmallRobots.RollingEv3rstorm
{
    class Program
    {
        #region Static Fields
        private static int previousNeckValue = 0;
        private static int previousHeadValue = 0;
        #endregion

        static public MenuContainer container;

        public static void Main(string[] args)
        {
            Menu menu = new Menu("Rolling Eve3rstorm");
            container = new MenuContainer(menu);
            menu.AddItem(new ItemWithNumericInput("Calibrate Head", 0, CalibrateHead, -30, 30));
            menu.AddItem(new MainMenuItem("Start", Start_OnEnterPressed));
            menu.AddItem(new MainMenuItem("Quit", Quit_OnEnterPressed));

            container.Show();
        }

        public static void TerminateMenu()
        {
            container.Terminate();
        }

        public static void CalibrateHead(int newValue)
        {
            sbyte maxSpeed = 10;
            sbyte speed = 0;
            Motor Motor = new Motor(MotorPort.OutB);

            if (newValue > previousHeadValue)
            {
                speed = (sbyte)-maxSpeed;
            }
            else
            {
                speed = (sbyte)maxSpeed;
            }
            previousHeadValue = newValue;

            Motor.SpeedProfileTime(speed, 100, 100, 100, true);
        }

        public static void Start_OnEnterPressed()
        {
            container.SuspendButtonEvents();
            SmallRobots.RollingEv3rstorm.RollingEv3rstorm rollingEv3rstorm = new RollingEv3rstorm();
            rollingEv3rstorm.Start();
            container.ResumeButtonEvents();
        }

        public static void Quit_OnEnterPressed()
        {
            LcdConsole.Clear();
            LcdConsole.WriteLine("Terminating");
            // Wait a bit
            Thread.Sleep(1000);
            TerminateMenu();
        }

    }
}
