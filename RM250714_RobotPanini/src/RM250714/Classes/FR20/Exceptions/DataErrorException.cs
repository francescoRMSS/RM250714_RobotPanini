using System;

namespace RM.src.RM250714.Classes.FR20.Exceptions
{
    /// <summary>
    /// Eccezioni per errori di dati mancanti o dati errati provenienti da database o strutture dati
    /// </summary>
    public class DataErrorException : Exception
    {
        /// <summary>
        /// Istanzia l'eccezione 
        /// </summary>
        /// <param name="message"></param>
        public DataErrorException(string message) : base(message) { }
    }
}
