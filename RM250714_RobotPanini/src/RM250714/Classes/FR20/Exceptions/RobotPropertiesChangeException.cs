using System;

namespace RM.src.RM250714.Classes.FR20.Exceptions
{
    /// <summary>
    /// Eccezioni per metodi che cambiano o leggono le proprietà del robot
    /// </summary>
    public class RobotPropertiesChangeException : Exception
    {
        /// <summary>
        /// Istanzia l'eccezione
        /// </summary>
        /// <param name="message"></param>
        public RobotPropertiesChangeException(string message) : base(message) { }
    }
}
