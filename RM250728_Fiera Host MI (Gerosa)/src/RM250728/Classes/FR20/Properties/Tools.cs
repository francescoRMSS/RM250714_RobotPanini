using fairino;
using RMLib.DataAccess;
using System;
using System.Collections.Generic;
using System.Data;

namespace RM.src.RM250728.Classes.FR20
{
    /// <summary>
    /// Rappresenta un tool pronto per essere inserito nella lista di tool
    /// </summary>
    struct ToolStruct
    {
        public int id; // ID Tool [0 = tool di default del robot], è quello che viene scritto al robot
        public string name; // Nome da usare per cambiare il tool del robot in quello scelto
        public DescPose pose; // Pose che identifica i punti calcolati dalla matrice di roto-traslazione dal robot
        public int type;
        public int install;
    }

    /// <summary>
    /// Gstisce i tool del robot
    /// </summary>
    public class Tools
    {
        private static readonly RobotDAOSqlite RobotDAO = new RobotDAOSqlite();
        private static readonly SqliteConnectionConfiguration DatabaseConnection = new SqliteConnectionConfiguration();
        private static readonly string ConnectionString = DatabaseConnection.GetConnectionString();

        private List<ToolStruct> _tools;
        private Robot _robot;
        /// <summary>
        /// Tool corrente
        /// </summary>
        public int currentTool = -1;

        /// <summary>
        /// Costruisce il tool manager
        /// </summary>
        /// <param name="robot"></param>
        public Tools(Robot robot)
        {
            _robot = robot;
            _tools = new List<ToolStruct>();

            InitList();
        }

        private void InitList()
        {
            DataTable _table = RobotDAO.GetRobotTools(ConnectionString);
            ToolStruct _tool;

            if (_table != null)
            {
                foreach (DataRow row in _table.Rows)
                {
                    _tool = new ToolStruct
                    {
                        id = Convert.ToInt32(row[RobotDAOSqlite.ROBOT_TOOLS_ID_COLUMN_NAME]),
                        name = row[RobotDAOSqlite.ROBOT_TOOLS_NAME_COLUMN_NAME].ToString(),
                        pose = new DescPose(
                            Convert.ToDouble(row[RobotDAOSqlite.ROBOT_TOOLS_X_COLUMN_NAME]), 
                            Convert.ToDouble(row[RobotDAOSqlite.ROBOT_TOOLS_Y_COLUMN_NAME]), 
                            Convert.ToDouble(row[RobotDAOSqlite.ROBOT_TOOLS_Z_COLUMN_NAME]),
                            Convert.ToDouble(row[RobotDAOSqlite.ROBOT_TOOLS_RX_COLUMN_NAME]), 
                            Convert.ToDouble(row[RobotDAOSqlite.ROBOT_TOOLS_RY_COLUMN_NAME]), 
                            Convert.ToDouble(row[RobotDAOSqlite.ROBOT_TOOLS_RZ_COLUMN_NAME])
                            ),
                        type = Convert.ToInt32(row[RobotDAOSqlite.ROBOT_TOOLS_TYPE_COLUMN_NAME]),
                        install = Convert.ToInt32(row[RobotDAOSqlite.ROBOT_TOOLS_INSTALL_COLUMN_NAME])
                    };
                    _tools.Add(_tool);
                }
            }
        }

        private ToolStruct? ReadToolData(int toolId)
        {
            ToolStruct _data = new ToolStruct();

            foreach (ToolStruct _struct in _tools)
            {
                if (_struct.id == toolId)
                {
                    _data.id = _struct.id;
                    _data.name = _struct.name;
                    _data.pose = _struct.pose;

                    return _data;
                }
            }

            return null;
        }

        private ToolStruct? ReadToolData(string toolName)
        {
            ToolStruct _data = new ToolStruct();

            foreach (ToolStruct _struct in _tools)
            {
                if (_struct.name.Equals(toolName))
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
        /// Modifica il tool in uso del robot e controlla che sia stato cambiato effettivamente
        /// </summary>
        /// <param name="toolId"></param>
        /// <returns></returns>
        public int ChangeRobotTool(int toolId)
        {
            int _checkNewTool = -1;
            ToolStruct? _data;

            if (toolId < 0) // Frame non valido
                return 0;

            _data = ReadToolData(toolId);
            if (_data == null) // ID Frame non trovato nella lista
                return 1;

            if (_data.Value.id == currentTool)
                return 2;

            _robot.SetToolCoord(toolId, _data.Value.pose, _data.Value.type, _data.Value.install);
            _robot.GetActualTCPNum(0, ref _checkNewTool);

            if (_checkNewTool != toolId)    // ID di risposta diverso da ID settato
                return 3;

            RobotManager.tool = toolId;
            currentTool = toolId;
            return 10;
        }

        /// <summary>
        /// Modifica il tool in uso del robot e controlla che sia stato cambiato effettivamente
        /// </summary>
        /// <param name="toolName"></param>
        /// <returns></returns>
        public int ChangeRobotTool(string toolName)
        {
            int _checkNewTool = -1;
            ToolStruct? _data;

            if (toolName.Length == 0 || toolName == null) // Frame non valido
                return 0;

            _data = ReadToolData(toolName);
            if (_data == null) // ID Frame non trovato nella lista
                return 1;

            if (_data.Value.id == currentTool)
                return 2;

            _robot.SetToolCoord(_data.Value.id, _data.Value.pose, _data.Value.type, _data.Value.install);
            _robot.GetActualTCPNum(0, ref _checkNewTool);

            if (_checkNewTool != _data.Value.id)    // ID di risposta diverso da ID settato
                return 3;

            RobotManager.user = _data.Value.id;
            currentTool = _data.Value.id;
            return 10;
        }

        /// <summary>
        /// In base all'errNum stabilisce se l'errore è bloccante o meno
        /// </summary>
        /// <param name="errNum"></param>
        /// <returns></returns>
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

        /// <summary>
        /// In base all'errNum restituisce un messaggio di errore
        /// </summary>
        /// <param name="errNum"></param>
        /// <returns></returns>
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
