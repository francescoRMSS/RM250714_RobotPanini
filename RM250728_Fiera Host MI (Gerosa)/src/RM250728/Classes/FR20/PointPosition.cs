using fairino;
using RM.src.RM250728.Classes.FR20.Applications.Application;

namespace RM.src.RM250728
{
    /// <summary>
    /// Definisce una posizione di un punto con il relativo timestamp, 
    /// modalità di registrazione e nome della posizione
    /// </summary>
    public struct PointPosition
    {
        #region Proprietà classe PointPosition

        /// <summary>
        /// Identificatore univoco della posizione
        /// </summary>
        public string guid;

        /// <summary>
        /// Contiene le coordinate x-y-z e le rotazioni rx-ry-rz
        /// </summary>
        public DescPose position;

        /// <summary>
        /// Timestamp in cui il punto è stato preso in drag mode
        /// </summary>
        public string timeStamp;

        /// <summary>
        /// Modalità PTP o Linear della drag mode
        /// </summary>     
        public string mode;

        /// <summary>
        /// Nome posizione
        /// </summary>
        public string positionName;

        /// <summary>
        /// Impostazioni della pistola per una determinata posizione
        /// </summary>
        public GunSettings gunSettings;

        #endregion

        /// <summary>
        /// Costruisce un oggetto contenente le informazioni relative ad un singolo punto 
        /// preso tramite modalità insegnamento
        /// </summary>
        /// <param name="guid">Identificatore univoco della posizione</param>
        /// <param name="pos">Coordinate del punto</param>
        /// <param name="time">Timestamp del punto</param>
        /// <param name="modality">Modalità di registrazione del punto</param>
        /// <param name="posName">Nome del punto</param>
        /// <param name="gunSettings">Impostazioni della pistola per una determinata posizione</param>
        public PointPosition(string guid, DescPose pos, string time, string modality, string posName, GunSettings gunSettings)
        {
            this.guid = guid;
            position = pos;
            timeStamp = time;
            mode = modality;
            positionName = posName;
            this.gunSettings = gunSettings;
        }
    }
}
