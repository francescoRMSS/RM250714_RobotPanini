using System;

namespace RM.src.RM250714.Classes.FR20.Exceptions
{
    /// <summary>
    /// Eccezioni per movimenti del robot
    /// </summary>
    public class RobotMovementException : Exception
    {
        /// <summary>
        /// Istanzia l'eccezione
        /// </summary>
        /// <param name="message"></param>
        public RobotMovementException(string message) : base(message) { }
    }
}
