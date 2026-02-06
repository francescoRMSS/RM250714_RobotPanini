using fairino;

namespace RM.src.RM250714
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

        #endregion
    }
}
