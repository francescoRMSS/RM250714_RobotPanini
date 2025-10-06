namespace RM.src.RM250728.Classes.FR20.Applications.Application
{
    /// <summary>
    /// Contiene la struttura delle impostazioni della pistola relative ad ogni posizione di un'applicazione Robot
    /// </summary>
    public class GunSettings
    {
        #region Parametri di GunSettings

        /// <summary>
        /// Identificativo univoco delle impostazioni della pistola 
        /// </summary>
        public string guid { get; set; }

        /// <summary>
        /// Identificativo univoco della posizione a cui fanno riferimento le impostazioni della pistola
        /// </summary>
        public string guid_ele { get; set; }

        /// <summary>
        /// Numero intero progressivo che identifica le impostazioni della pistola di una determinata applicazione Robot
        /// </summary>
        public int? id { get; set; }

        /// <summary>
        /// Polvere pistola
        /// </summary>
        public int? feed_air { get; set; }

        /// <summary>
        /// Dosaggio pistola
        /// </summary>
        public int? dosage_air { get; set; }

        /// <summary>
        /// Aria pistola
        /// </summary>
        public int? gun_air { get; set; }

        /// <summary>
        /// kV pistola
        /// </summary>
        public int? kV { get; set; }

        /// <summary>
        /// Microampere pistola
        /// </summary>
        public int? microampere { get; set; }

        /// <summary>
        /// Stato pistola (ON/OFF)
        /// </summary>
        public int? status { get; set; }

        /// <summary>
        /// Identificativo univoco dell'applicazione a cui fanno riferimento le impostazioni della pistola
        /// </summary>
        public string application { get; set; }

        #endregion

        /// <summary>
        /// Costruttore
        /// </summary>
        /// <param name="guid">Identificativo univoco delle impostazioni pistola</param>
        /// <param name="guid_ele">Identificativo univoco della posizione a cui fanno riferimento le impostazioni della pistola</param>
        /// <param name="id">Numero intero progressivo che identifica le impostazioni della pistola di una determinata applicazione Robot</param>
        /// <param name="feed_air">Polvere pistola</param>
        /// <param name="dosage_air">Dosaggio pistola</param>
        /// <param name="gun_air">Aria pistola</param>
        /// <param name="kV">kV pistola</param>
        /// <param name="microampere">Microampere pistola</param>
        /// <param name="status">Stato pistola (ON/OFF)</param>
        /// <param name="application">Identificativo univoco dell'applicazione a cui fanno riferimento le impostazioni della pistola</param>
        public GunSettings(string guid, string guid_ele, int? id, int? feed_air, int? dosage_air, int? gun_air, int? kV, int? microampere, int? status, string application)
        {
            this.guid = guid;
            this.guid_ele = guid_ele;
            this.id = id;
            this.feed_air = feed_air;
            this.dosage_air = dosage_air;
            this.gun_air = gun_air;
            this.kV = kV;
            this.microampere = microampere;
            this.status = status;
            this.application = application;
        }

        /// <summary>
        /// Costruttore vuoto
        /// </summary>
        public GunSettings()
        { }
    }
}
