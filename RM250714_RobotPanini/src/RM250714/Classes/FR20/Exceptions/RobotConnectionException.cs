using System;

namespace RM.src.RM250714.Classes.FR20.Exceptions
{
    /// <summary>
    /// Eccezioni per disconnesione del robot
    /// </summary>
    public class RobotConnectionException : Exception
    {
        /// <summary>
        /// Istanzia l'eccezione
        /// </summary>
        /// <param name="message"></param>
        public RobotConnectionException(string message) : base(message) { }
    }
}
