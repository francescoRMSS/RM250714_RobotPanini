using System;

namespace RM.src.RM250714.Classes.FR20.Exceptions
{
    /// <summary>
    /// Eccezioni generiche che avvengono a runtime 
    /// </summary>
    public class RunTimeException : Exception
    {
        /// <summary>
        /// Istanzia l'eccezione
        /// </summary>
        /// <param name="message"></param>
        public RunTimeException(string message) : base(message) { }
    }
}
