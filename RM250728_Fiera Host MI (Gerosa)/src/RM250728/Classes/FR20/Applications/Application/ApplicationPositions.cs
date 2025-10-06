using RM.src.RM250728.Classes.FR20.Applications.Application;

namespace RM.src.RM250728
{
    /// <summary>
    /// Contiene la struttura di una posizione all'interno di un'applicazione robot
    /// </summary>
    public class ApplicationPositions
    {
        #region Parametri di ApplicationPositions

        /// <summary>
        /// Identificativo univoco della posizione
        /// </summary>
        public string guid { get; set; }

        /// <summary>
        /// id della posizione
        /// </summary>
        public int id { get; set; }

        /// <summary>
        /// x della posizione
        /// </summary>
        public float x { get; set; }

        /// <summary>
        /// y della posizione
        /// </summary>
        public float y { get; set; }

        /// <summary>
        /// z della posizione
        /// </summary>
        public float z { get; set; }

        /// <summary>
        /// rx della posizione
        /// </summary>
        public float rx { get; set; }

        /// <summary>
        /// ry della posizione
        /// </summary>
        public float ry { get; set; }

        /// <summary>
        /// rz della posizione
        /// </summary>
        public float rz { get; set; }

        /// <summary>
        /// Nome della posizione
        /// </summary>
        public string name { get; set; }

        /// <summary>
        /// Impostazioni della pistola per una determinata posizione
        /// </summary>
        public GunSettings gunSettings { get; set; }

        #endregion

        /// <summary>
        /// Costruttore parametrizzato di una posizione all'interno di un'applicazione robot
        /// </summary>
        /// <param name="guid">Identificatore univoco della posizione</param>
        /// <param name="id">id posizione</param>
        /// <param name="x">x posizione</param>
        /// <param name="y">y posizione</param>
        /// <param name="z">z posizione</param>
        /// <param name="rx">rx posizione</param>
        /// <param name="ry">ry posizione</param>
        /// <param name="rz">rz posizione</param>
        /// <param name="name">name posizione</param>
        /// <param name="gunSettings">gunSettings</param>
        public ApplicationPositions(string guid, int id, float x, float y, float z, float rx, float ry, float rz, string name, GunSettings gunSettings)
        {
            this.guid = guid;
            this.id = id;
            this.x = x;
            this.y = y;
            this.z = z;
            this.rx = rx;
            this.ry = ry;
            this.rz = rz;
            this.name = name;
            this.gunSettings = gunSettings;
        }
    }
}
