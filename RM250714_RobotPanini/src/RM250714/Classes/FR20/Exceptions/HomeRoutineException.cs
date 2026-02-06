using System;

namespace RM.src.RM250714.Classes.FR20.Exceptions
{
    /// <summary>
    /// Eccezioni per mancanze di consensi o errori durante la home routine
    /// </summary>
    public class HomeRoutineException : Exception
    {
        /// <summary>
        /// Istanzia l'eccezione
        /// </summary>
        /// <param name="message"></param>
        public HomeRoutineException(string message) : base(message) { }
    }
}
