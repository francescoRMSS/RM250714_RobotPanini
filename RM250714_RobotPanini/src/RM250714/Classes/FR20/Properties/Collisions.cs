
using RMLib.DataAccess;
using RMLib.Logger;
using System;
using System.Collections.Generic;
using System.Data;

namespace RM.src.RM250714.Classes.FR20.Properties
{
    struct CollisionStruct
    {
        public int index;
        public int mode;
        public double[] levels;
        public int config;
    }

    /// <summary>
    /// Gestisce il livello di collisioni del robot
    /// </summary>
    public class Collisions
    {
        /// <summary>
        /// Logger
        /// </summary>
        private static readonly log4net.ILog log = LogHelper.GetLogger();

        private static readonly RobotDAOSqlite RobotDAO = new RobotDAOSqlite();
        private static readonly SqliteConnectionConfiguration DatabaseConnection = new SqliteConnectionConfiguration();
        private static readonly string ConnectionString = DatabaseConnection.GetConnectionString();

        private readonly List<CollisionStruct> _collisions;

        /// <summary>
        /// Costruisce il manager dei livelli di collisione
        /// </summary>
        public Collisions()
        {
            _collisions = new List<CollisionStruct>();

            InitList();
        }

        private void InitList()
        {
            DataTable _table = RobotDAO.GetRobotCollisionLevels(ConnectionString);
            CollisionStruct _collisionLevel;

            if (_table != null)
            {
                foreach (DataRow row in _table.Rows)
                {
                    _collisionLevel = new CollisionStruct
                    {
                        index = Convert.ToInt32(row[RobotDAOSqlite.ROBOT_COLLISION_LEVELS_ID_COLUMN_NAME]),
                        mode = Convert.ToInt32(row[RobotDAOSqlite.ROBOT_COLLISION_LEVELS_MODE_COLUMN_NAME]),
                        levels = new double[] {
                            Convert.ToDouble(row[RobotDAOSqlite.ROBOT_COLLISION_LEVELS_J1_COLUMN_NAME]),
                            Convert.ToDouble(row[RobotDAOSqlite.ROBOT_COLLISION_LEVELS_J2_COLUMN_NAME]),
                            Convert.ToDouble(row[RobotDAOSqlite.ROBOT_COLLISION_LEVELS_J3_COLUMN_NAME]),
                            Convert.ToDouble(row[RobotDAOSqlite.ROBOT_COLLISION_LEVELS_J4_COLUMN_NAME]),
                            Convert.ToDouble(row[RobotDAOSqlite.ROBOT_COLLISION_LEVELS_J5_COLUMN_NAME]),
                            Convert.ToDouble(row[RobotDAOSqlite.ROBOT_COLLISION_LEVELS_J6_COLUMN_NAME])
                            },
                        config = Convert.ToInt32(row[RobotDAOSqlite.ROBOT_COLLISION_LEVELS_CONFIG_COLUMN_NAME])
                    };
                    _collisions.Add(_collisionLevel);
                }
            }
        }

        private CollisionStruct? ReadCollisionData(int collisionIndex)
        {
            CollisionStruct _data = new CollisionStruct();

            foreach (CollisionStruct _struct in _collisions)
            {
                if (_struct.index == collisionIndex)
                {
                    _data.index = _struct.index;
                    _data.mode = _struct.mode;
                    _data.levels = _struct.levels;
                    _data.config = _struct.config;

                    return _data;
                }
            }

            return null;
        }

        /// <summary>
        /// Modifica il livello delle collisioni sul robot
        /// </summary>
        /// <param name="collisionIndex"></param>
        /// <returns></returns>
        public bool ChangeRobotCollision(int collisionIndex)
        {
            CollisionStruct? _data;
            int errNum = 10;

            if (collisionIndex < 0) // Id non valido
                errNum = 0;
            else
            {
                _data = ReadCollisionData(collisionIndex);
                if (_data == null) // Id non trovato nella lista
                    errNum = 1;
                else
                {
                    int err = RobotManager.SetAntiCollision(_data.Value.mode, _data.Value.levels, _data.Value.config);
                    if (err != 0)
                    {
                        errNum = 3;
                    }
                    else
                    {
                        RobotManager.currentCollisionLevel = collisionIndex;
                        //currentCollisionLevel = collisionIndex;
                    }
                }
            }

            if (IsErrorBlocking(errNum))
            {
                log.Error("[Collision levels] Errore durante cambio livello di collisioni: " + GetErrorCode(errNum));
                RobotManager.GenerateAlarm(0, 3);
                RobotManager.RobotPropertiesError = true;
                return false;
            }
            log.Info("[Collision levels] Cambio di livello collisioni a " + collisionIndex);
            return true;

        }

        /// <summary>
        /// In base all'errNum dice se l'errore è bloccante o meno
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
        /// In base all'errNum restituisce un messaggio di errore
        /// </summary>
        /// <param name="errNum"></param>
        /// <returns></returns>
        public string GetErrorCode(int errNum)
        {
            string errCode;

            switch (errNum)
            {
                case 0:
                    errCode = "Id numerico non valido";
                    break;
                case 1:
                    errCode = "Id non trovato nella lista";
                    break;
                case 2:
                    errCode = "Livello di collisioni già impostato";
                    break;
                case 3:
                    errCode = "Errore durante il cambio di livello di collisioni";
                    break;
                case 10:
                    errCode = "Livello di collisioni modificato correttamente";
                    break;
                default:
                    errCode = "Error number non trovato";
                    break;
            }

            return errCode;
        }
    }
}
