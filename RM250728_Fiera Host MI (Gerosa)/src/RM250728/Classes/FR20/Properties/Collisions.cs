using fairino;
using RMLib.DataAccess;
using System;
using System.Collections.Generic;
using System.Data;

namespace RM.src.RM250728.Classes.FR20.Properties
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
        private static readonly RobotDAOSqlite RobotDAO = new RobotDAOSqlite();
        private static readonly SqliteConnectionConfiguration DatabaseConnection = new SqliteConnectionConfiguration();
        private static readonly string ConnectionString = DatabaseConnection.GetConnectionString();

        List<CollisionStruct> _collisions;
        Robot _robot;
        int currentCollisionLevel = -1;

        public Collisions(Robot robot)
        {
            _robot = robot;
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

            CollisionStruct collision1 = new CollisionStruct
            {
                index = 1,
                mode = 0,
                levels = new double[] { 1, 1, 1, 1, 1, 1 },
                config = 0
            };
            _collisions.Add(collision1);

            CollisionStruct collision2 = new CollisionStruct
            {
                index = 1,
                mode = 0,
                levels = new double[] { 2, 2, 2, 2, 2, 2 },
                config = 0
            };
            _collisions.Add(collision2);

            CollisionStruct collision3 = new CollisionStruct
            {
                index = 1,
                mode = 0,
                levels = new double[] {3, 3, 3, 3, 3, 3 },
                config = 0
            };
            _collisions.Add(collision3);

            CollisionStruct collision4 = new CollisionStruct
            {
                index = 1,
                mode = 0,
                levels = new double[] { 4, 4, 4, 4, 4, 4 },
                config = 0
            };
            _collisions.Add(collision4);

            CollisionStruct collision5 = new CollisionStruct
            {
                index = 1,
                mode = 0,
                levels = new double[] { 5, 5, 5, 5, 5, 5 },
                config = 0
            };
            _collisions.Add(collision5);

            CollisionStruct collision6 = new CollisionStruct
            {
                index = 1,
                mode = 0,
                levels = new double[] { 6, 6, 6, 6, 6, 6 },
                config = 0
            };
            _collisions.Add(collision6);

            CollisionStruct collision7 = new CollisionStruct
            {
                index = 1,
                mode = 0,
                levels = new double[] { 7, 7, 7, 7, 7, 7 },
                config = 0
            };
            _collisions.Add(collision7);

            CollisionStruct collision8 = new CollisionStruct
            {
                index = 1,
                mode = 0,
                levels = new double[] { 8, 8, 8, 8, 8, 8 },
                config = 0
            };
            _collisions.Add(collision8);
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

        public int ChangeRobotCollision(int collisionIndex)
        {
            CollisionStruct? _data;

            if (collisionIndex < 0) // Frame non valido
                return 0;

            _data = ReadCollisionData(collisionIndex);
            if (_data == null) // ID Frame non trovato nella lista
                return 1;

            if (_data.Value.index == currentCollisionLevel)
                return 1;

            _robot.SetAnticollision(_data.Value.mode, _data.Value.levels, _data.Value.config);

            RobotManager.currentCollisionLevel = collisionIndex;
            currentCollisionLevel = collisionIndex;
            return 10;
        }

        public bool IsErrorBlocking(int errNum)
        {
            switch (errNum)
            {
                case 0:
                    return true;
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

        public string GetErrorCode(int errNum)
        {
            string errCode = "";

            switch (errNum)
            {
                case 0:
                    errCode = "Tool ID minore di 0";
                    break;
                case 1:
                    errCode = "Tool ID o nome Tool non trovato";
                    break;
                case 2:
                    errCode = "Tool già impostato";
                    break;
                case 3:
                    errCode = "Tool impostato diverso dal frame desiderato";
                    break;
                case 10:
                    errCode = "Tool modificato correttamente";
                    break;
                default:
                    errCode = "Error number non trovato";
                    break;
            }

            return errCode;
        }
    }
}
