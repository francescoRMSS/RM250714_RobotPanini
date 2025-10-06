
namespace RM.src.RM250728.Classes.PLC
{
    /// <summary>
    /// Contiene i tag degli indirizzi PLC
    /// </summary>
    public class PLCTagName
    {
        #region Scrittura a PLC

        #region Robot

        /// <summary>
        /// Clock da scrivere ogni secondo (il clock su PLC gira ogni 2s)
        /// </summary>
        public const string LifeBit_out = "PLC1_" + "LifeBit_out";

        /// <summary>
        /// Se il valore è 1, indica che la lista di lavoro del robot contiene almeno un programma
        /// </summary>
        public const string Hw_ready_to_start = "PLC1_" + "Com_robot_attiva";

        /// <summary>
        /// A 1 quando quando il robot lavora in modalità automatico
        /// </summary>
        public const string Automatic_Start = "PLC1_" + "Automatic_Start";

        /// <summary>
        /// A 1 quando il robot sta comunicando con il programma
        /// </summary>
        public const string ApplicationComRobot_active = "PLC1_" + "ApplicationComRobot_active";

        /// <summary>
        /// A 1 quando quando il robot ha almeno un programma i nmemoria
        /// </summary>
        public const string Program_In_Memory = "PLC1_" + "Program_In_Memory";

        /// <summary>
        /// Anno - versione robot
        /// </summary>
        public const string VersionYear = "PLC1_" + "VersionYear";

        /// <summary>
        /// Mese - versione robot
        /// </summary>
        public const string VersionMonth = "PLC1_" + "VersionMonth";

        /// <summary>
        /// Giorno - versione robot
        /// </summary>
        public const string VersionDay = "PLC1_" + "VersionDay";

        /// <summary>
        /// Feedback in scrittura dello stato di pausa del robot
        /// </summary>
        public const string Move_InPause = "PLC1_" + "Move_InPause";

        /// <summary>
        /// Errore nel robot
        /// </summary>
        public const string Robot_error = "PLC1_" + "Robot_error";

        /// <summary>
        /// Stato robot abilitato
        /// </summary>
        public const string Robot_enable = "PLC1_" + "Robot_enable";

        /// <summary>
        /// Stato robot
        /// </summary>
        public const string Robot_status = "PLC1_" + "Robot_status";

        /*
        /// <summary>
        /// Livello di collisione corrente del robot
        /// </summary>
        public const string ACT_CollisionLevel = "PLC1_" + "ACT_CollisionLevel";*/

        /// <summary>
        /// tool corrente del robot
        /// </summary>
        public const string ACT_N_Tool = "PLC1_" + "ACT_N_Tool";

        /// <summary>
        /// user corrente del robot
        /// </summary>
        public const string ACT_N_Frame = "PLC1_" + "ACT_N_Frame";

        /// <summary>
        /// Posizione sicura del robot
        /// </summary>
        public const string SafePos = "PLC1_" + "SafePos";

        /// <summary>
        /// Ciclo auto terminato
        /// </summary>
        public const string Auto_Cycle_End = "PLC1_" + "Auto_Cycle_End";

        /// <summary>
        /// A 1 quando il Robot si trova in zona di home
        /// </summary>
        public const string ACT_Zone_Home_inPos = "PLC1_" + "ACT_Zone_Home_inPos";

        /// <summary>
        /// A 1 quando il Robot si trova in zona di pick
        /// </summary>
        public const string ACT_Zone_Pick_inPos = "PLC1_" + "ACT_Zone_Pick_inPos";

        /// <summary>
        /// A 1 quando il Robot si trova in zona di place
        /// </summary>
        public const string ACT_Zone_Place_inPos = "PLC1_" + "ACT_Zone_Place_inPos";

        /// <summary>
        /// A 1 quando è avviato il ciclo main
        /// </summary>
        public const string CycleRun_Main = "PLC1_" + "CycleRun_Main";

        /// <summary>
        /// A 1 quando è avviato il ciclo home
        /// </summary>
        public const string CycleRun_Home = "PLC1_" + "CycleRun_Home";

        /// <summary>
        /// A 1 quando è avviato il ciclo pick
        /// </summary>
        public const string CycleRun_Pick = "PLC1_" + "CycleRun_Pick";

        /// <summary>
        /// A 1 quando è avviato il ciclo place
        /// </summary>
        public const string CycleRun_Place = "PLC1_" + "CycleRun_Place";

        /// <summary>
        /// coord x di home point
        /// </summary>
        public const string HomePoint_X = "PLC1_" + "HomePoint_X";

        /// <summary>
        /// coord y di home point
        /// </summary>
        public const string HomePoint_Y = "PLC1_" + "HomePoint_Y";

        /// <summary>
        /// coord z di home point
        /// </summary>
        public const string HomePoint_Z = "PLC1_" + "HomePoint_Z";

        /// <summary>
        /// coord rx di home point
        /// </summary>
        public const string HomePoint_RX = "PLC1_" + "HomePoint_RX";

        /// <summary>
        /// coord ry di home point
        /// </summary>
        public const string HomePoint_RY = "PLC1_" + "HomePoint_RY";

        /// <summary>
        /// coord rz di home point
        /// </summary>
        public const string HomePoint_RZ = "PLC1_" + "HomePoint_RZ";

        /// <summary>
        /// coord x di pick box 1
        /// </summary>
        public const string PickPoint_Box1_X = "PLC1_" + "PickPoint_Box1_X";

        /// <summary>
        /// coord y di pick box 1
        /// </summary>
        public const string PickPoint_Box1_Y = "PLC1_" + "PickPoint_Box1_Y";

        /// <summary>
        /// coord z di pick box 1
        /// </summary>
        public const string PickPoint_Box1_Z = "PLC1_" + "PickPoint_Box1_Z";

        /// <summary>
        /// coord rx di pick box 1
        /// </summary>
        public const string PickPoint_Box1_RX = "PLC1_" + "PickPoint_Box1_RX";

        /// <summary>
        /// coord ry di pick box 1
        /// </summary>
        public const string PickPoint_Box1_RY = "PLC1_" + "PickPoint_Box1_RY";

        /// <summary>
        /// coord rz di pick box 1
        /// </summary>
        public const string PickPoint_Box1_RZ = "PLC1_" + "PickPoint_Box1_RZ";

        /// <summary>
        /// coord x di pick box 2
        /// </summary>
        public const string PickPoint_Box2_X = "PLC1_" + "PickPoint_Box2_X";

        /// <summary>
        /// coord y di pick box 2
        /// </summary>
        public const string PickPoint_Box2_Y = "PLC1_" + "PickPoint_Box2_Y";

        /// <summary>
        /// coord z di pick box 2
        /// </summary>
        public const string PickPoint_Box2_Z = "PLC1_" + "PickPoint_Box2_Z";

        /// <summary>
        /// coord rx di pick box 2
        /// </summary>
        public const string PickPoint_Box2_RX = "PLC1_" + "PickPoint_Box2_RX";

        /// <summary>
        /// coord ry di pick box 2
        /// </summary>
        public const string PickPoint_Box2_RY = "PLC1_" + "PickPoint_Box2_RY";

        /// <summary>
        /// coord rz di pick box 2
        /// </summary>
        public const string PickPoint_Box2_RZ = "PLC1_" + "PickPoint_Box2_RZ";

        /// <summary>
        /// coord x di pick box 3
        /// </summary>
        public const string PickPoint_Box3_X = "PLC1_" + "PickPoint_Box3_X";

        /// <summary>
        /// coord y di pick box 3
        /// </summary>
        public const string PickPoint_Box3_Y = "PLC1_" + "PickPoint_Box3_Y";

        /// <summary>
        /// coord z di pick box 3
        /// </summary>
        public const string PickPoint_Box3_Z = "PLC1_" + "PickPoint_Box3_Z";

        /// <summary>
        /// coord rx di pick box 3
        /// </summary>
        public const string PickPoint_Box3_RX = "PLC1_" + "PickPoint_Box3_RX";

        /// <summary>
        /// coord ry di pick box 3
        /// </summary>
        public const string PickPoint_Box3_RY = "PLC1_" + "PickPoint_Box3_RY";

        /// <summary>
        /// coord rz di pick box 3
        /// </summary>
        public const string PickPoint_Box3_RZ = "PLC1_" + "PickPoint_Box3_RZ";

        /// <summary>
        /// coord x di place pallet 1 box 1
        /// </summary>
        public const string PlacePoint_Pallet1_Box1_X = "PLC1_" + "PlacePoint_Pallet1_Box1_X";

        /// <summary>
        /// coord y di place pallet 1 box 1
        /// </summary>
        public const string PlacePoint_Pallet1_Box1_Y = "PLC1_" + "PlacePoint_Pallet1_Box1_Y";

        /// <summary>
        /// coord z di place pallet 1 box 1
        /// </summary>
        public const string PlacePoint_Pallet1_Box1_Z = "PLC1_" + "PlacePoint_Pallet1_Box1_Z";

        /// <summary>
        /// coord rx di place pallet 1 box 1
        /// </summary>
        public const string PlacePoint_Pallet1_Box1_RX = "PLC1_" + "PlacePoint_Pallet1_Box1_RX";

        /// <summary>
        /// coord ry di place pallet 1 box 1
        /// </summary>
        public const string PlacePoint_Pallet1_Box1_RY = "PLC1_" + "PlacePoint_Pallet1_Box1_RY";

        /// <summary>
        /// coord rz di place pallet 1 box 1
        /// </summary>
        public const string PlacePoint_Pallet1_Box1_RZ = "PLC1_" + "PlacePoint_Pallet1_Box1_RZ";

        /// <summary>
        /// coord x di place pallet 1 box 2
        /// </summary>
        public const string PlacePoint_Pallet1_Box2_X = "PLC1_" + "PlacePoint_Pallet1_Box2_X";

        /// <summary>
        /// coord y di place pallet 1 box 2
        /// </summary>
        public const string PlacePoint_Pallet1_Box2_Y = "PLC1_" + "PlacePoint_Pallet1_Box2_Y";

        /// <summary>
        /// coord z di place pallet 1 box 2
        /// </summary>
        public const string PlacePoint_Pallet1_Box2_Z = "PLC1_" + "PlacePoint_Pallet1_Box2_Z";

        /// <summary>
        /// coord rx di place pallet 1 box 2
        /// </summary>
        public const string PlacePoint_Pallet1_Box2_RX = "PLC1_" + "PlacePoint_Pallet1_Box2_RX";

        /// <summary>
        /// coord ry di place pallet 1 box 2
        /// </summary>
        public const string PlacePoint_Pallet1_Box2_RY = "PLC1_" + "PlacePoint_Pallet1_Box2_RY";

        /// <summary>
        /// coord rz di place pallet 1 box 2
        /// </summary>
        public const string PlacePoint_Pallet1_Box2_RZ = "PLC1_" + "PlacePoint_Pallet1_Box2_RZ";

        /// <summary>
        /// coord x di place pallet 1 box 3
        /// </summary>
        public const string PlacePoint_Pallet1_Box3_X = "PLC1_" + "PlacePoint_Pallet1_Box3_X";

        /// <summary>
        /// coord y di place pallet 1 box 3
        /// </summary>
        public const string PlacePoint_Pallet1_Box3_Y = "PLC1_" + "PlacePoint_Pallet1_Box3_Y";

        /// <summary>
        /// coord z di place pallet 1 box 3
        /// </summary>
        public const string PlacePoint_Pallet1_Box3_Z = "PLC1_" + "PlacePoint_Pallet1_Box3_Z";

        /// <summary>
        /// coord rx di place pallet 1 box 3
        /// </summary>
        public const string PlacePoint_Pallet1_Box3_RX = "PLC1_" + "PlacePoint_Pallet1_Box3_RX";

        /// <summary>
        /// coord ry di place pallet 1 box 3
        /// </summary>
        public const string PlacePoint_Pallet1_Box3_RY = "PLC1_" + "PlacePoint_Pallet1_Box3_RY";

        /// <summary>
        /// coord rz di place pallet 1 box 3
        /// </summary>
        public const string PlacePoint_Pallet1_Box3_RZ = "PLC1_" + "PlacePoint_Pallet1_Box3_RZ";

        /// <summary>
        /// coord x di place pallet 2 box 1
        /// </summary>
        public const string PlacePoint_Pallet2_Box1_X = "PLC1_" + "PlacePoint_Pallet2_Box1_X";

        /// <summary>
        /// coord y di place pallet 2 box 1
        /// </summary>
        public const string PlacePoint_Pallet2_Box1_Y = "PLC1_" + "PlacePoint_Pallet2_Box1_Y";

        /// <summary>
        /// coord z di place pallet 2 box 1
        /// </summary>
        public const string PlacePoint_Pallet2_Box1_Z = "PLC1_" + "PlacePoint_Pallet2_Box1_Z";

        /// <summary>
        /// coord rx di place pallet 2 box 1
        /// </summary>
        public const string PlacePoint_Pallet2_Box1_RX = "PLC1_" + "PlacePoint_Pallet2_Box1_RX";

        /// <summary>
        /// coord ry di place pallet 2 box 1
        /// </summary>
        public const string PlacePoint_Pallet2_Box1_RY = "PLC1_" + "PlacePoint_Pallet2_Box1_RY";

        /// <summary>
        /// coord rz di place pallet 2 box 1
        /// </summary>
        public const string PlacePoint_Pallet2_Box1_RZ = "PLC1_" + "PlacePoint_Pallet2_Box1_RZ";

        /// <summary>
        /// coord x di place pallet 2 box 2
        /// </summary>
        public const string PlacePoint_Pallet2_Box2_X = "PLC1_" + "PlacePoint_Pallet2_Box2_X";

        /// <summary>
        /// coord y di place pallet 2 box 2
        /// </summary>
        public const string PlacePoint_Pallet2_Box2_Y = "PLC1_" + "PlacePoint_Pallet2_Box2_Y";

        /// <summary>
        /// coord z di place pallet 2 box 2
        /// </summary>
        public const string PlacePoint_Pallet2_Box2_Z = "PLC1_" + "PlacePoint_Pallet2_Box2_Z";

        /// <summary>
        /// coord rx di place pallet 2 box 2
        /// </summary>
        public const string PlacePoint_Pallet2_Box2_RX = "PLC1_" + "PlacePoint_Pallet2_Box2_RX";

        /// <summary>
        /// coord ry di place pallet 2 box 2
        /// </summary>
        public const string PlacePoint_Pallet2_Box2_RY = "PLC1_" + "PlacePoint_Pallet2_Box2_RY";

        /// <summary>
        /// coord rz di place pallet 2 box 2
        /// </summary>
        public const string PlacePoint_Pallet2_Box2_RZ = "PLC1_" + "PlacePoint_Pallet2_Box2_RZ";

        /// <summary>
        /// bit di controllo per start ciclo
        /// </summary>
        public const string Hmi_startCycle = "PLC1_" + "Hmi_startCycle";

        /// <summary>
        /// bit di controllo per stop ciclo
        /// </summary>
        public const string Hmi_stopCycle = "PLC1_" + "Hmi_stopCycle";

        /// <summary>
        /// bit di controllo per go to home position
        /// </summary>
        public const string Hmi_homeRoutine = "PLC1_" + "Hmi_homeRoutine";

        #endregion

        #region HMI

        /// <summary>
        /// Livello collisione robot
        /// </summary>
        public const string ACT_CollisionLevel = "PLC1_" + "ACT_CollisionLevel";

        /// <summary>
        /// Numero di step main cycle
        /// </summary>
        public const string ACT_Step_Cycle_Home = "PLC1_" + "ACT_Step_HomeCycle";

        /// <summary>
        /// Numero di step main cycle
        /// </summary>
        public const string ACT_Step_MainCycle = "PLC1_" + "ACT_Step_MainCycle";

        /// <summary>
        /// Numero di step ciclo di pick
        /// </summary>
        public const string ACT_Step_Cycle_Pick = "PLC1_" + "ACT_Step_Cycle_Pick";

        /// <summary>
        /// Numero di step ciclo di place
        /// </summary>
        public const string ACT_Step_Cycle_Place = "PLC1_" + "ACT_Step_Cycle_Place";

        /// <summary>
        /// x axis actual position [mm]
        /// </summary>
        public const string x_actual_pos = "PLC1_" + "x_actual_pos";

        /// <summary>
        /// y axis actual position [mm]
        /// </summary>
        public const string y_actual_pos = "PLC1_" + "y_actual_pos";

        /// <summary>
        /// z axis actual position [mm]
        /// </summary>
        public const string z_actual_pos = "PLC1_" + "z_actual_pos";

        /// <summary>
        /// rx axis actual position [°]
        /// </summary>
        public const string rx_actual_pos = "PLC1_" + "rx_actual_pos";

        /// <summary>
        /// ry axis actual position [°]
        /// </summary>
        public const string ry_actual_pos = "PLC1_" + "ry_actual_pos";

        /// <summary>
        /// rz axis actual position [°]
        /// </summary>
        public const string rz_actual_pos = "PLC1_" + "rz_actual_pos";

        /// <summary>
        /// j1 actual position [°]
        /// </summary>
        public const string j1_actual_pos = "PLC1_" + "j1_actual_pos";

        /// <summary>
        /// j2 actual position [°]
        /// </summary>
        public const string j2_actual_pos = "PLC1_" + "j2_actual_pos";

        /// <summary>
        /// j3 actual position [°]
        /// </summary>
        public const string j3_actual_pos = "PLC1_" + "j3_actual_pos";

        /// <summary>
        /// j4 actual position [°]
        /// </summary>
        public const string j4_actual_pos = "PLC1_" + "j4_actual_pos";

        /// <summary>
        /// j5 actual position [°]
        /// </summary>
        public const string j5_actual_pos = "PLC1_" + "j5_actual_pos";

        /// <summary>
        /// j6 actual position [°]
        /// </summary>
        public const string j6_actual_pos = "PLC1_" + "j6_actual_pos";

        #endregion

        #endregion

        #region Lettura da PLC

        #region Robot

        /// <summary>
        /// Clock da leggere ogni secondo (su clock su PLC gira ogni 2s)
        /// </summary>
        public static string LifeBit_in = "PLC1_" + "com_robot_1";

        /// <summary>
        /// A 1 durante tutta la durante del ciclo
        /// </summary>
        public static string Start_Auto_Robot = "PLC1_" + "com_robot_2";

        /// <summary>
        /// spare
        /// </summary>
        public static string spare_to_read_50 = "PLC1_" + "com_robot_3";

        /// <summary>
        /// spare
        /// </summary>
        public static string CMD_OverrideAuto = "PLC1_" + "com_robot_4";

        /// <summary>
        /// 0-joint, 2-base, 4-tool, 8-workpiece
        /// </summary>
        public const string Jog_Ref_type = "PLC1_" + "com_robot_5";

        /// <summary>
        /// Jog speed set [%]
        /// </summary>
        public const string Jog_speed_set = "PLC1_" + "com_robot_6";

        /// <summary>
        /// 1=x / 2=y / 3=z / 4=rx / 5=ry / 6=rz
        /// </summary>
        public const string Axis_selection = "PLC1_" + "com_robot_7";

        /// <summary>
        /// x axis threshold [mm]
        /// </summary>
        public const string x_threshold = "PLC1_" + "com_robot_8";

        /// <summary>
        /// y axis threshold [mm]
        /// </summary>
        public const string y_threshold = "PLC1_" + "com_robot_9";

        /// <summary>
        /// z axis threshold [mm]
        /// </summary>
        public const string z_threshold = "PLC1_" + "com_robot_10";

        /// <summary>
        /// rx axis threshold [°]
        /// </summary>
        public const string rx_threshold = "PLC1_" + "com_robot_11";

        /// <summary>
        /// ry axis threshold [°]
        /// </summary>
        public const string ry_threshold = "PLC1_" + "com_robot_12";

        /// <summary>
        /// rz axis threshold [°]
        /// </summary>
        public const string rz_threshold = "PLC1_" + "com_robot_13";

        /// <summary>
        /// spare
        /// </summary>
        public const string spare_52_to_read = "PLC1_" + "com_robot_14";

        /// <summary>
        /// Enable robot from PLC
        /// </summary>
        public const string Enable = "PLC1_" + "com_robot_15";

        /// <summary>
        /// 0=Off, 1=auto, 2=man
        /// </summary>
        public const string Operating_Mode = "PLC1_" + "com_robot_16";

        /// <summary>
        /// 0...1(bool)
        /// </summary>
        public static string JogX_pos = "PLC1_" + "com_robot_17";

        /// <summary>
        /// 0...1(bool)
        /// </summary>
        public static string JogX_neg = "PLC1_" + "com_robot_18";

        /// <summary>
        /// 0...1(bool)
        /// </summary>
        public static string JogY_pos = "PLC1_" + "com_robot_29";

        /// <summary>
        /// 0...1(bool)
        /// </summary>
        public static string JogY_neg = "PLC1_" + "com_robot_20";

        /// <summary>
        /// 0...1(bool)
        /// </summary>
        public static string JogZ_pos = "PLC1_" + "com_robot_21";

        /// <summary>
        /// 0...1(bool)
        /// </summary>
        public static string JogZ_neg = "PLC1_" + "com_robot_22";

        /// <summary>
        /// 0...1(bool)
        /// </summary>
        public static string JogRX_pos = "PLC1_" + "com_robot_23";

        /// <summary>
        /// 0...1(bool)
        /// </summary>
        public static string JogRX_neg = "PLC1_" + "com_robot_24";

        /// <summary>
        /// 0...1(bool)
        /// </summary>
        public static string JogRY_pos = "PLC1_" + "com_robot_25";

        /// <summary>
        /// 0...1(bool)
        /// </summary>
        public static string JogRY_neg = "PLC1_" + "com_robot_26";

        /// <summary>
        /// 0...1(bool)
        /// </summary>
        public static string JogRZ_pos = "PLC1_" + "com_robot_27";

        /// <summary>
        /// 0...1(bool)
        /// </summary>
        public static string JogRZ_neg = "PLC1_" + "com_robot_28";

        #endregion

        #endregion


    }
}
