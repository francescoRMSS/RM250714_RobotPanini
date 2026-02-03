using System;

namespace RM.src.RM250714.Classes.FR20.Exceptions
{
    /// <summary>
    /// Eccezioni per i calcoli di cinametica inversa o cinematica diretta da parte del robot
    /// </summary>
    public class RobotKinException : Exception
    {
        /// <summary>
        /// Istanzia l'eccezione
        /// </summary>
        /// <param name="message"></param>
        public RobotKinException(string message) : base(message) { }
    }
}
