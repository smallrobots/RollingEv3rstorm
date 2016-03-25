using System;
using MonoBrickFirmware;
using MonoBrickFirmware.Display.Dialogs;
using MonoBrickFirmware.Display;
using MonoBrickFirmware.Movement;
using System.Threading;
using MonoBrickFirmware.Display.Menus;

namespace SmallRobots.Ev3ControlLib.Menu
{
        /// <summary>
        /// MenuItem that can be added to the MonoBrickFirmware.Display.Menus.Menu Class
        /// It features a delegate that can be istantiated to provide a callback for the Enter key pressed event
        /// </summary>
        public class MainMenuItem : ChildItemWithParent
        {
            /// <summary>
            /// Delegate for the OnEnterPressed() Event
            /// </summary>
            public delegate void RedefinedOnEnterPressed();

            /// <summary>
            /// Delegate handler for the OnEnterPressed() Event
            /// </summary>
            public RedefinedOnEnterPressed RedefinedOnEnterPressedHandler;

            #region Constructors
            /// <summary>
            /// Initializes a new instance of the <see cref="SmallRobots.Menus.MainMenuItem"/> class.
            /// </summary>
            /// <param name="mainMenuItem">String for the menuItem</param>
            /// <param name="OnEnterPressed">Delegate handler for the OnEnterPressed() Event.</param>
            public MainMenuItem(string menuItem, RedefinedOnEnterPressed OnEnterPressed) : base(menuItem)
            {
                RedefinedOnEnterPressedHandler = OnEnterPressed;
            }
            #endregion

            #region Public Methods
            public override void OnEnterPressed()
            {
                base.OnEnterPressed();
                if (RedefinedOnEnterPressedHandler != null)
                {
                    RedefinedOnEnterPressedHandler();
                }
            }
            #endregion
        }
}
