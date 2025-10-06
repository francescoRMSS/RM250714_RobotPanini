namespace RM.src.RM250728.Classes.FR20
{
    /// <summary>
    /// Contiene le proprietà utili al movimento del Robot in Jog
    /// </summary>
    public class JogRobotProperties
    {
        /// <summary>
        /// Movimento (+) sull'asse x
        /// </summary>
        public bool jogXpos { get; set; }

        /// <summary>
        /// Movimento (-) sull'asse x
        /// </summary>
        public bool jogXneg { get; set; }

        /// <summary>
        /// Movimento (+) sull'asse y
        /// </summary>
        public bool jogYpos { get; set; }

        /// <summary>
        /// Movimento (-) sull'asse y
        /// </summary>
        public bool jogYneg { get; set; }

        /// <summary>
        /// Movimento (+) sull'asse z
        /// </summary>
        public bool jogZpos { get; set; }

        /// <summary>
        /// Movimento (-) sull'asse z
        /// </summary>
        public bool jogZneg { get; set; }

        /// <summary>
        /// Movimento (+) sull'asse rx
        /// </summary>
        public bool jogRXpos { get; set; }

        /// <summary>
        /// Movimento (-) sull'asse rx
        /// </summary>
        public bool jogRXneg { get; set; }

        /// <summary>
        /// Movimento (+) sull'asse ry
        /// </summary>
        public bool jogRYpos { get; set; }

        /// <summary>
        /// Movimento (-) sull'asse ry
        /// </summary>
        public bool jogRYneg { get; set; }

        /// <summary>
        /// Movimento (+) sull'asse rz
        /// </summary>
        public bool jogRZpos { get; set; }

        /// <summary>
        /// Movimento (-) sull'asse rz
        /// </summary>
        public bool jogRZneg { get; set; }

        /// <summary>
        /// Stato precedente di movimento (+) sull'asse x
        /// </summary>
        public bool prevJogXpos { get; set; }

        /// <summary>
        /// Stato precedente di movimento (-) sull'asse x
        /// </summary>
        public bool prevJogXneg { get; set; }

        /// <summary>
        /// Stato precedente di movimento (+) sull'asse y
        /// </summary>
        public bool prevJogYpos { get; set; }

        /// <summary>
        /// Stato precedente di movimento (-) sull'asse y
        /// </summary>
        public bool prevJogYneg { get; set; }

        /// <summary>
        /// Stato precedente di movimento (+) sull'asse z
        /// </summary>
        public bool prevJogZpos { get; set; }

        /// <summary>
        /// Stato precedente di movimento (-) sull'asse z
        /// </summary>
        public bool prevJogZneg { get; set; }

        /// <summary>
        /// Stato precedente di movimento (+) sull'asse rx
        /// </summary>
        public bool prevJogRXpos { get; set; }

        /// <summary>
        /// Stato precedente di movimento (-) sull'asse rx
        /// </summary>
        public bool prevJogRXneg { get; set; }

        /// <summary>
        /// Stato precedente di movimento (+) sull'asse ry
        /// </summary>
        public bool prevJogRYpos { get; set; }

        /// <summary>
        /// Stato precedente di movimento (-) sull'asse ry
        /// </summary>
        public bool prevJogRYneg { get; set; }

        /// <summary>
        /// Stato precedente di movimento (+) sull'asse rz
        /// </summary>
        public bool prevJogRZpos { get; set; }

        /// <summary>
        /// Stato precedente di movimento (-) sull'asse rz
        /// </summary>
        public bool prevJogRZneg { get; set; }

        /// <summary>
        /// Soglia di spostamento su asse x
        /// </summary>
        public int JogX_treshold { get; set; }

        /// <summary>
        /// Soglia di spostamento su asse y
        /// </summary>
        public int JogY_treshold { get; set; }

        /// <summary>
        /// Soglia di spostamento su asse z
        /// </summary>
        public int JogZ_treshold { get; set; }

        /// <summary>
        /// Soglia di spostamento su asse rx
        /// </summary>
        public int JogRX_treshold { get; set; }

        /// <summary>
        /// Soglia di spostamento su asse ry
        /// </summary>
        public int JogRY_treshold { get; set; }

        /// <summary>
        /// Soglia di spostamento su asse rz
        /// </summary>
        public int JogRZ_treshold { get; set; }

        /// <summary>
        /// Velocità di spostamento in jog
        /// </summary>
        public int JogSpeed { get; set; }

        /// <summary>
        /// Accelerazione del movimento in jop
        /// </summary>
        public int JogAcceleration { get; set; }

        /// <summary>
        /// 0-joint, 2-base, 4-tool, 8-workpiece
        /// </summary>
        public byte JogRefType { get; set; }

        /// <summary>
        /// Numero di asse selezionato
        /// </summary>
        public int axisSelection { get; set; }
    }
}
