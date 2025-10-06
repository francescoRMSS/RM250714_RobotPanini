using System;
using System.Collections.Generic;

namespace RM.src.RM250728
{
    /// <summary>
    /// Contiene la struttura di un'applicazione Robot
    /// </summary>
    public class RobotApplication
    {
        #region Parametri di RobotApplication
        /// <summary>
        /// Id dell'applicazione
        /// </summary>
        public int id { get; set; }

        /// <summary>
        /// Timestamp di creazione dell'applicazione
        /// </summary>
        public String creation { get; set; }

        /// <summary>
        /// Timestamp dell'ultima modifica dell'applicazione
        /// </summary>
        public String lastUpdate { get; set; }

        /// <summary>
        /// Note applicazione
        /// </summary>
        public String note { get; set; }

        /// <summary>
        /// Lista di posizioni che contiene l'applicazione
        /// </summary>
        public List<ApplicationPositions> positions { get; set; }
        #endregion

        /// <summary>
        /// Costruttore parametrizzato con id e posizioni relative
        /// </summary>
        /// <param name="id">Id applicazione</param>
        /// <param name="positions">Posizioni applicazione</param>
        public RobotApplication(int id, List<ApplicationPositions> positions)
        {
            this.id = id;
            this.positions = positions;
        }

        /// <summary>
        /// Costruttore parametrizzato con id, timestamp di creazione, timestamp di ultima modifica e posizioni relative
        /// </summary>
        /// <param name="id">Id applicazione</param>
        /// <param name="creation">Timestamp di creazione applicazione</param>
        /// <param name="lastUpdate">Timestamp di ultima modifica applicazione</param>
        /// <param name="positions">Lista dei punti del'applicazione Robot</param>
        public RobotApplication(int id, string creation, string lastUpdate, List<ApplicationPositions> positions)
        {
            this.id = id;
            this.creation = creation;
            this.lastUpdate = lastUpdate;
            this.positions = positions;
        }

    }
}
