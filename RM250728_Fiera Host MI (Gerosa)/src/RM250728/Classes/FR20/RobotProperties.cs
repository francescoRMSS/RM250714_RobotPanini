namespace RM.src.RM250728
{
    /// <summary>
    /// Oggetto che contiene tutte le proprietà del Robot editabili dall'utente. 
    /// I permessi della modifica vengono gestiti dal modulo della sicurezza.
    /// Valori presi dal database.
    /// Creato nel metodo SetRobotProperties.
    /// </summary>
    public class RobotProperties
    {
        #region Proprietà della classe RobotProperties
        /// <summary>
        /// Velocità del robot
        /// </summary>
        public int Speed;

        /// <summary>
        /// Percentuale di velocità del robot
        /// </summary>
        public float Velocity;

        /// <summary>
        /// Smoothing time
        /// </summary>
        public float Blend;

        /// <summary>
        /// Percentuale di accelerazione
        /// </summary>
        public float Acceleration;

        /// <summary>
        /// Fattore di scalatura di velocità
        /// </summary>
        public float Ovl;

        /// <summary>
        /// Tool utilizzato
        /// </summary>
        public int Tool;

        /// <summary>
        /// User utilizzato
        /// </summary>
        public int User;

        /// <summary>
        /// Peso in kg da 1 a FRX che il robot usa per aiutare i movimenti durante la drag mode
        /// </summary>
        public int Weight;

        /// <summary>
        /// Frequenza registrazione punti in DragMode
        /// </summary>
        public int VelRec;
        #endregion

        #region Lista delle velocità



        #endregion

        /// <summary>
        /// Costruttore dell'oggetto RobotProperties
        /// </summary>
        /// <param name="speed">Velocità del robot</param>
        /// <param name="velocity">Percentuale di velocità del robot</param>
        /// <param name="blend">Smoothing time</param>
        /// <param name="acceleration">Percentuale di accelerazione</param>
        /// <param name="ovl">Fattore di scalatura di velocità</param>
        /// <param name="tool">Tool utilizzato</param>
        /// <param name="user">User utilizzato</param>
        /// <param name="weight">Carico massimo del robot</param>
        /// <param name="velRec">Velocità di registrazione dei punti</param>
        public RobotProperties(int speed, float velocity, float blend, float acceleration, float ovl, int tool, int user, int weight, int velRec)
        {
            Speed = speed;
            Velocity = velocity;
            Blend = blend;
            Acceleration = acceleration;
            Ovl = ovl;
            Tool = tool;
            User = user;
            Weight = weight;
            VelRec = velRec;
        }
    }
}
