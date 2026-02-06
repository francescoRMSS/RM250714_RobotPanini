using fairino;
using RMLib.DataAccess;
using RMLib.Logger;
using System;
using System.Collections.Generic;
using System.Data;

namespace RM.src.RM250714.Classes.FR20
{
    /// <summary>
    /// Rappresenta un frame pronto per essere inserito nella lista di frame
    /// </summary>
    struct FrameStruct
    {
        public int id; // ID frame [0 = frame di default del robot], è quello che viene scritto al robot
        public string name; // Nome da usare per cambiare il frame del robot in quello scelto
        public DescPose pose; // Pose che identifica i punti calcolati dalla matrice di roto-traslazione dal robot
    }

    /// <summary>
    /// Gestisce i frame del robot
    /// </summary>
    public class Frames
    {
        /// <summary>
        /// Logger
        /// </summary>
        private static readonly log4net.ILog log = LogHelper.GetLogger();

        private static readonly RobotDAOSqlite RobotDAO = new RobotDAOSqlite();
        private static readonly SqliteConnectionConfiguration DatabaseConnection = new SqliteConnectionConfiguration();
        private static readonly string ConnectionString = DatabaseConnection.GetConnectionString();

        private readonly List<FrameStruct> _frames;

        /// <summary>
        /// Costruisce il frame manager
        /// </summary>
        public Frames()
        {
            _frames = new List<FrameStruct>();

            InitList();
        }

        private void InitList()
        {
            DataTable _table = RobotDAO.GetRobotFrames(ConnectionString);
            FrameStruct _frame;

            if (_table != null)
            {
                foreach (DataRow row in _table.Rows)
                {
                    _frame = new FrameStruct
                    {
                        id = Convert.ToInt32(row[RobotDAOSqlite.ROBOT_FRAMES_ID_COLUMN_NAME]),
                        name = row[RobotDAOSqlite.ROBOT_FRAMES_NAME_COLUMN_NAME].ToString(),
                        pose = new DescPose(
                            Convert.ToDouble(row[RobotDAOSqlite.ROBOT_FRAMES_X_COLUMN_NAME]),
                            Convert.ToDouble(row[RobotDAOSqlite.ROBOT_FRAMES_Y_COLUMN_NAME]),
                            Convert.ToDouble(row[RobotDAOSqlite.ROBOT_FRAMES_Z_COLUMN_NAME]),
                            Convert.ToDouble(row[RobotDAOSqlite.ROBOT_FRAMES_RX_COLUMN_NAME]),
                            Convert.ToDouble(row[RobotDAOSqlite.ROBOT_FRAMES_RY_COLUMN_NAME]),
                            Convert.ToDouble(row[RobotDAOSqlite.ROBOT_FRAMES_RZ_COLUMN_NAME])
                            )
                    };
                    _frames.Add(_frame);
                }
            }
        }

        private FrameStruct? ReadFrameData(int frameId)
        {
            FrameStruct _data = new FrameStruct();

            foreach (FrameStruct _struct in _frames)
            {
                if (_struct.id == frameId)
                {
                    _data.id = _struct.id;
                    _data.name = _struct.name;
                    _data.pose = _struct.pose;

                    return _data;
                }
            }

            return null;
        }

        private FrameStruct? ReadFrameData(string frameName)
        {
            FrameStruct _data = new FrameStruct();

            foreach (FrameStruct _struct in _frames)
            {
                if (_struct.name.Equals(frameName))
                {
                    _data.id = _struct.id;
                    _data.name = _struct.name;
                    _data.pose = _struct.pose;

                    return _data;
                }
            }

            return null;
        }

        /// <summary>
        /// Modifica il frame in uso del robot e controlla che sia stato cambiato effettivamente
        /// </summary>
        /// <param name="frameId"></param>
        /// <returns></returns>
        public bool ChangeRobotFrame(int frameId)
        {
            int _checkNewFrame = -1;
            FrameStruct? _data;
            int errNum = 10;

            if (frameId < 0) // Frame non valido
                errNum = 0;
            else
            {
                _data = ReadFrameData(frameId);
                if (_data == null) // ID Frame non trovato nella lista
                    errNum = 1;
                else
                {
                    RobotManager.SetWObjCoord(frameId, _data.Value.pose, 0);
                    RobotManager.GetActualWobjCoord(ref _checkNewFrame);

                    if (_checkNewFrame != frameId)    // ID di risposta diverso da ID settato
                        errNum = 3;
                    else
                    {
                        RobotManager.user = frameId;
                    }
                }
            }
            if (IsErrorBlocking(errNum))
            {
                log.Error("[Frames] Errore durante cambio frame: " + GetErrorCode(errNum));
                RobotManager.GenerateAlarm(0, 1);
                RobotManager.robotPropertiesError = true;
                return false;
            }
            log.Info("[Frames] Cambio di frame a " + frameId);
            return true;
        }

        /// <summary>
        /// Modifica il frame in uso del robot e controlla che sia stato cambiato effettivamente
        /// </summary>
        /// <param name="frameName"></param>
        /// <returns></returns>
        public bool ChangeRobotFrame(string frameName)
        {
            int _checkNewFrame = -1;
            FrameStruct? _data;
            int errNum = 10;

            if (frameName.Length == 0 || frameName == null) // Frame non valido
                errNum = 0;
            else
            {
                _data = ReadFrameData(frameName);
                if (_data == null) // ID Frame non trovato nella lista
                    errNum = 1;
                else
                {
                    RobotManager.SetWObjCoord(_data.Value.id, _data.Value.pose, 0);
                    RobotManager.GetActualWobjCoord(ref _checkNewFrame);

                    if (_checkNewFrame != _data.Value.id)    // ID di risposta diverso da ID settato
                        errNum = 3;
                    else
                    {
                        RobotManager.user = _data.Value.id;
                    }
                }
            }
            if (IsErrorBlocking(errNum))
            {
                log.Error("[Frames] Errore durante cambio frame: " + GetErrorCode(errNum));
                RobotManager.GenerateAlarm(0, 1);
                RobotManager.robotPropertiesError = true;
                return false;
            }
            log.Info("[Frames] Cambio di frame a " + frameName);
            return true;
        }

        /// <summary>
        /// In base all'errNum restituisce se l'errore è bloccante o meno
        /// </summary>
        /// <param name="errNum"></param>
        /// <returns></returns>
        public bool IsErrorBlocking(int errNum)
        {
            switch (errNum)
            {
                case 0:
                    return false;
                case 1:
                    return true;
                case 2:
                    return false;
                case 3:
                    return true;
                case 10:
                    return false;
                default:
                    return true;
            }
        }

        /// <summary>
        /// In base all'errNum restituisce il messaggio di errore
        /// </summary>
        /// <param name="errNum"></param>
        /// <returns></returns>
        public string GetErrorCode(int errNum)
        {
            string errCode = "";

            switch (errNum)
            {
                case 0:
                    errCode = "Frame ID minore di 0";
                    break;
                case 1:
                    errCode = "Frame ID o nome Frame non trovato";
                    break;
                case 2:
                    errCode = "Frame già impostato";
                    break;
                case 3:
                    errCode = "Frame impostato diverso dal frame desiderato";
                    break;
                case 10:
                    errCode = "Frame modificato correttamente";
                    break;
                default:
                    errCode = "Error number non trovato";
                    break;
            }

            return errCode;
        }
    }
}
