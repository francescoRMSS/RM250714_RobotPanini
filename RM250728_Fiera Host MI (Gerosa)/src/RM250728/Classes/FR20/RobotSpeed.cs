namespace RM.src.RM250728.Classes.FR20
{
    /// <summary>
    /// Contiene le velocità già scalate
    /// </summary>
    public class RobotSpeed
    {
        /// <summary>
        /// Id numerico della speed
        /// </summary>
        int id;
        /// <summary>
        /// Velocity e Acceleration calcolate come percentuale di speed in base a override e speed mode. 
        /// Usata durante la fase di lavoro a movimento lineare.
        /// </summary>
        int workSpeed;
        /// <summary>
        /// Velocity e Acceleration calcolate come percentuale di speed in base a override e speed mode. 
        /// Usata durante la fase di movimento tra zone di lavoro.
        /// </summary>
        int movementSpeed;
        /// <summary>
        /// Velocity e Acceleration calcolate come percentuale di speed in base a override e speed mode. 
        /// Usata durante la fase di lavoro a movimento angolare.
        /// </summary>
        int angularSpeed;
        /// <summary>
        /// Velocity e Acceleration calcolate come percentuale di speed in base a override e speed mode. 
        /// Usata durante la fase di pick.
        /// </summary>
        int pickSpeed;
        /// <summary>
        /// Velocity e Acceleration calcolate come percentuale di speed in base a override e speed mode. 
        /// Usata durante la fase di place.
        /// </summary>
        int placeSpeed;

        public RobotSpeed(int id)
        {
            this.id = id;
            workSpeed = 0;
            movementSpeed = 0;
            angularSpeed = 0;
            pickSpeed = 0;
            placeSpeed = 0;
        }

        public void SetSpeed(int velOverride, SpeedMode mode)
        {
            
        }
    }
}
