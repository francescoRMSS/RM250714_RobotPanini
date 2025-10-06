using fairino;
using RM.src.RM250728.Classes.FR20.Applications.Application;
using RMLib.Logger;
using System;
using System.Collections.Generic;
using System.Linq;

namespace RM.src.RM250728
{
    /// <summary>
    /// Gestisce il dizionario delle applicazioni del robot e gli eventi ad esse correlati
    /// </summary>
    public class ObservableApplicationsDictionary
    {
        #region Parametri di ObservableApplicationsDictionary

        /// <summary>
        /// logger
        /// </summary>
        private static readonly log4net.ILog log = LogHelper.GetLogger();

        /// <summary>
        /// Dizionario contenente key-RobotApplication
        /// </summary>
        private Dictionary<string, RobotApplication> applicationsDictionary = new Dictionary<string, RobotApplication>();

        #endregion

        #region Eventi del Dizionario delle Applicazioni
        /// <summary>
        /// Evento che intercetta il salvataggio dei dati da PLC tramite listView delle ricette
        /// </summary>
        public event EventHandler<RobotDictionaryChangedEventArgs> DataSaved;

        /// <summary>
        /// Evento che intercetta l'aggiunta di una nuova applicazione
        /// </summary>
        public event EventHandler<RobotDictionaryChangedEventArgs> ApplicationAdded;

        /// <summary>
        /// Evento che intercetta l'eliminazione di un'applicazione
        /// </summary>
        public event EventHandler<RobotDictionaryChangedEventArgs> ApplicationDeleted;

        /// <summary>
        /// Evento che intercetta l'eliminazione di un punto
        /// </summary>
        public event EventHandler<RobotDictionaryChangedEventArgs> ApplicationPositionDeleted;

        /// <summary>
        /// Evento che intercetta l'eliminazione di un punto a partire da un id
        /// </summary>
        public event EventHandler<RobotDictionaryChangedEventArgs> ApplicationPositionDeletedStartingFromId;

        /// <summary>
        /// Evento che intercetta l'eliminazione di un punto
        /// </summary>
        public event EventHandler<RobotDictionaryChangedEventArgs> ApplicationPositionDeletedFromId;

        /// <summary>
        /// Evento che intercetta l'update id di un'applicazione esistente
        /// </summary>
        public event EventHandler<RobotDictionaryChangedEventArgs> ApplicationIdUpdated;

        /// <summary>
        /// Evento che intercetta l'update note di un'applicazione esistente
        /// </summary>
        public event EventHandler<RobotDictionaryChangedEventArgs> ApplicationNoteUpdated;

        /// <summary>
        /// Evento che intercetta l'update name di un'applicaziozione esistente
        /// </summary>
        public event EventHandler<RobotDictionaryChangedEventArgs> ApplicationNameUpdated;

        /// <summary>
        /// Evento che intercetta l'update name di una posizione esistente
        /// </summary>
        public event EventHandler<RobotDictionaryChangedEventArgs> RobotPositionNameUpdated;

        /// <summary>
        /// Evento che intercetta la cancellazione di una posizione esistente
        /// </summary>
        public event EventHandler<RobotDictionaryChangedEventArgs> RobotPositionRemoved;
        #endregion

        #region Metodi di ObservableApplicationsDictionary

        /// <summary>
        /// Metodo get del dizionario
        /// </summary>
        /// <returns></returns>
        public Dictionary<string, RobotApplication> getDictionary()
        {
            return applicationsDictionary;
        }

        /// <summary>
        /// Restituisce la chiave della prima riga del dizionairo
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        public string GetApplicationName()
        {
            return applicationsDictionary.Keys.First();
        }

        /// <summary>
        /// Metodo che aggiunge applicazione al dizionario delle applicazioni e invoca
        /// evento ApplicationAdded così che la listview delle applicazioni lo intercetti 
        /// e aggiunga la nuova applicazione
        /// </summary>
        /// <param name="applicationName">nome applicazione</param>
        /// <param name="application">applicazione con tutti i suoi campi</param>
        public void AddApplication(string applicationName, RobotApplication application)
        {
            applicationsDictionary.Add(applicationName, application);
            ApplicationAdded?.Invoke(this, new RobotDictionaryChangedEventArgs(applicationName, application));
        }

        /// <summary>
        /// Metodo che cancella applicazione e relative posizioni dal dizionario delle applicazioni
        /// </summary>
        /// <param name="key">Nome della ricetta</param>
        public void DeleteApplication(string key)
        {
            applicationsDictionary.Remove(key);
            ApplicationDeleted?.Invoke(this, new RobotDictionaryChangedEventArgs());
        }

        /// <summary>
        /// Cancella la posizione dalla lista e invoca metodo che aggiorna lw_positions
        /// </summary>
        /// <param name="positionId">Indice posizione selezionata</param>
        public void RemovePositionFromApplication(int positionId)
        {
            // Ottieni l'oggetto RobotApplication dal dizionario
            RobotApplication application = applicationsDictionary[RobotManager.applicationName];

            // Verifica se la lista positions è inizializzata
            if (application.positions != null)
            {
                // Trova l'elemento da rimuovere
                ApplicationPositions positionToRemove = application.positions.FirstOrDefault(p => p.id == positionId);

                // Se l'elemento è trovato, rimuovilo dalla lista
                if (positionToRemove != null)
                {
                    application.positions.Remove(positionToRemove);
                    RobotPositionRemoved?.Invoke(this, new RobotDictionaryChangedEventArgs(positionId));

                    // Aggiorna gli ID delle voci rimanenti per mantenerli sequenziali
                    for (int i = 0; i < application.positions.Count; i++)
                    {
                        application.positions[i].id = i + 1;
                    }
                }
            }
        }

        /// <summary>
        /// Cancella la posizione dalla lista e invoca metodo che aggiorna lw_positions
        /// </summary>
        /// <param name="positionId">Indice posizione selezionata</param>
        /// <param name="applicationName">Applicazione in cui si trova la posizione da eliminare</param>
        public void RemovePositionFromApplication(int positionId, string applicationName)
        {
            // Ottieni l'oggetto RobotApplication dal dizionario
            RobotApplication application = applicationsDictionary[applicationName];

            // Verifica se la lista positions è inizializzata
            if (application.positions != null)
            {
                // Trova l'elemento da rimuovere
                ApplicationPositions positionToRemove = application.positions.FirstOrDefault(p => p.id == positionId);

                // Se l'elemento è trovato, rimuovilo dalla lista
                if (positionToRemove != null)
                {
                    application.positions.Remove(positionToRemove);
                    RobotPositionRemoved?.Invoke(this, new RobotDictionaryChangedEventArgs(positionId));

                    // Aggiorna gli ID delle voci rimanenti per mantenerli sequenziali
                    for (int i = 0; i < application.positions.Count; i++)
                    {
                        application.positions[i].id = i + 1;
                    }
                }
            }
        }

        /// <summary>
        /// Esegue get della posizione tramite il nome
        /// </summary>
        /// <param name="positionName">Nome della posizione</param>
        /// <returns></returns>
        public ApplicationPositions GetPosition(string positionName)
        {
            if (applicationsDictionary.ContainsKey(RobotManager.applicationName))
            {
                var application = applicationsDictionary[RobotManager.applicationName];
                return application.positions.FirstOrDefault(pos => pos.name.Equals(positionName, StringComparison.OrdinalIgnoreCase));
            }
            return null;
        }

        /// <summary>
        /// Esegue get della posizione tramite nome posizione e nome applicazione
        /// </summary>
        /// <param name="positionName">Nome posizione</param>
        /// <param name="applicationName">Nome applicazione</param>
        /// <returns></returns>
        public ApplicationPositions GetPosition(string positionName, string applicationName)
        {
            if (applicationsDictionary.ContainsKey("RM"))
            {
                var application = applicationsDictionary["RM"];
                return application.positions.FirstOrDefault(pos => pos.name.Equals(positionName, StringComparison.OrdinalIgnoreCase));
            }
            return null;
        }

        /// <summary>
        /// Metodo che aggiorna id del dizionario delle applicazioni
        /// </summary>
        /// <param name="key">Nome applicazione</param>
        /// <param name="id">ID applicazione</param>
        /// <param name="unixTimestamp"></param>
        public void UpdateRobotApplicationId(string key, int id, long unixTimestamp)
        {
            applicationsDictionary[key].lastUpdate = unixTimestamp.ToString();
            applicationsDictionary[key].id = id;

            // Formatto la data da secondi a data
            DateTime dateTime = DateTimeOffset.FromUnixTimeMilliseconds(long.Parse(unixTimestamp.ToString())).DateTime.ToLocalTime();
            string formattedDate = dateTime.ToString("dd-MM-yyyy HH:mm:ss");

            ApplicationIdUpdated?.Invoke(this, new RobotDictionaryChangedEventArgs(id, formattedDate));
        }

        /// <summary>
        /// Metodo che aggiorna nome del dizionario delle applicazioni
        /// </summary>
        /// <param name="key"></param>
        /// <param name="name"></param>
        /// <param name="unixTimestamp"></param>
        public void UpdateApplicationName(string key, string name, long unixTimestamp)
        {
            applicationsDictionary[key].lastUpdate = unixTimestamp.ToString();

            RobotApplication application = applicationsDictionary[key];
            applicationsDictionary.Remove(key);
            applicationsDictionary[name] = application;

            // Formatto la data da secondi a data
            DateTime dateTime = DateTimeOffset.FromUnixTimeMilliseconds(long.Parse(unixTimestamp.ToString())).DateTime.ToLocalTime();
            string formattedDate = dateTime.ToString("dd-MM-yyyy HH:mm:ss");

            ApplicationNameUpdated?.Invoke(this, new RobotDictionaryChangedEventArgs(name, formattedDate));
        }

        /// <summary>
        /// Metodo che aggiorna nota e lastUdpateTime del dizionario delle applicazioni
        /// </summary>
        /// <param name="key">Nome applicazione</param>
        /// <param name="note">Nuova nota</param>
        /// <param name="unixTimestamp">timeStamp in millisecondi</param>
        public void UpdateApplicationNote(string key, string note, long unixTimestamp)
        {
            applicationsDictionary[key].lastUpdate = unixTimestamp.ToString();
            applicationsDictionary[key].note = note;

            // Formatto la data da secondi a data
            DateTime dateTime = DateTimeOffset.FromUnixTimeMilliseconds(long.Parse(unixTimestamp.ToString())).DateTime.ToLocalTime();
            string formattedDate = dateTime.ToString("dd-MM-yyyy HH:mm:ss");

            ApplicationNoteUpdated?.Invoke(this, new RobotDictionaryChangedEventArgs(note, formattedDate));
        }

        /// <summary>
        /// Metodo che aggiunge una posizione all'applicazione del Robot
        /// </summary>
        /// <param name="pos"></param>
        /// <param name="applicationName"></param>
        /// <param name="id"></param>
        public void AddApplicationRobotPosition(string guid_pos, DescPose pos, string applicationName, int id, string positionName, GunSettings gunSettings)
        {
            applicationsDictionary[applicationName].positions.Add(
                new ApplicationPositions(
                    guid_pos,
                    id,
                    float.Parse(pos.tran.x.ToString()),
                    float.Parse(pos.tran.y.ToString()),
                    float.Parse(pos.tran.z.ToString()),
                    float.Parse(pos.rpy.rx.ToString()),
                    float.Parse(pos.rpy.ry.ToString()),
                    float.Parse(pos.rpy.rz.ToString()),
                    positionName,
                    gunSettings
                    ));
        }

        /// <summary>
        /// Aggiorna la posizione specificata nell'applicazione RobotManager.applicationName con le nuove coordinate fornite.
        /// </summary>
        /// <param name="positionName">Il nome della posizione da aggiornare.</param>
        /// <param name="pose">L'oggetto DescPose contenente le nuove coordinate della posizione.</param>
        /// <remarks>
        /// Questo metodo cerca una posizione con il nome specificato all'interno dell'applicazione RobotManager.applicationName nel dizionario.
        /// Se trova la posizione, aggiorna le coordinate X, Y, Z, RX, RY, RZ con i valori forniti.
        /// Registra le operazioni di aggiornamento e gli eventuali errori nei log.
        /// </remarks>
        public void UpdatePosition(string positionName, DescPose pose, string applicationName)
        {
            if (applicationsDictionary.ContainsKey(applicationName))
            {
                var application = applicationsDictionary[applicationName];
                ApplicationPositions position = application.positions.FirstOrDefault(pos => pos.name.Equals(positionName, StringComparison.OrdinalIgnoreCase));

                if (position != null)
                {
                    log.Info($"Aggiornamento della posizione '{positionName}' nell'applicazione {applicationName} " +
                             $"con nuove coordinate: X={pose.tran.x}, Y={pose.tran.y}, Z={pose.tran.z}, " +
                             $"RX={pose.rpy.rx}, RY={pose.rpy.ry}, RZ={pose.rpy.rz}");
                    position.x = (float)pose.tran.x;
                    position.y = (float)pose.tran.y;
                    position.z = (float)pose.tran.z;
                    position.rx = (float)pose.rpy.rx;
                    position.ry = (float)pose.rpy.ry;
                    position.rz = (float)pose.rpy.rz;
                    log.Info("Aggiornamento completato con successo.");
                }
                else
                {
                    log.Warn($"Posizione '{positionName}' non trovata nell'applicazione {applicationName}.");
                }
            }
            else
            {
                log.Warn($"Applicazione {applicationName} non trovata nel dizionario.");
            }
        }

        /// <summary>
        /// Aggiorna nome posizione in lw_positions
        /// </summary>
        /// <param name="positionName">Nome posizione</param>
        /// <param name="date">Data</param>
        public void UpdateRobotPositionName(string positionName, string date)
        {
            RobotPositionNameUpdated?.Invoke(this, new RobotDictionaryChangedEventArgs(positionName, date));
        }

        /// <summary>
        /// Metodo che cancella applicazione e relative posizioni dal dizionario delle applicazioni
        /// </summary>
        /// <param name="applicationName_Key">Nome della ricetta</param>
        /// <param name="positionID"></param>
        public void DeleteApplicationPosition(string applicationName_Key, int positionID)
        {
            applicationsDictionary[applicationName_Key].positions.RemoveAt(positionID);
            //shift dgli id successivi
            for (int i = positionID; i < applicationsDictionary[applicationName_Key].positions.Count; i++)
            {
                applicationsDictionary[applicationName_Key].positions[i].id--;
            }
            ApplicationPositionDeleted?.Invoke(this, new RobotDictionaryChangedEventArgs(positionID));
        }

        /// <summary>
        /// Metodo che cancella i punti di una applicazione a partire da un indice
        /// </summary>
        /// <param name="applicationName_Key">Nome della ricetta</param>
        /// <param name="positionID"></param>
        public void DeleteApplicationPositionStartingFromId(string applicationName_Key, int positionID)
        {
            // positionID è l'id del punto, non la sua posizione nella lista (indice)
            applicationsDictionary[applicationName_Key].positions.RemoveRange(positionID - 1,
                applicationsDictionary[applicationName_Key].positions.Count - (positionID - 1));

            ApplicationPositionDeletedStartingFromId?.Invoke(this, new RobotDictionaryChangedEventArgs(positionID));
        }

        /// <summary>
        /// Metodo che cancella il punto selezionato dal dizionario e invoca l'evento ApplicationPositionDeletedFromId
        /// utile all'aggiornamento della listView dei punti in debugMode
        /// </summary>
        /// <param name="applicationName_Key">Nome dell'applicazione</param>
        /// <param name="positionID">Punto selezionato da cancellare</param>
        public void DeleteApplicationPositionFromId(string applicationName_Key, int positionID)
        {
            try
            {
                applicationsDictionary[applicationName_Key].positions.RemoveAt(positionID);
                log.Info("Punto selezionato cancellato dal dizionario");
                log.Info("Invocazione evento: ApplicationPositionDeletedFromId");
                ApplicationPositionDeletedFromId?.Invoke(this, new RobotDictionaryChangedEventArgs(positionID));
            }
            catch (Exception ex)
            {
                log.Error($"Errore durante la cancellazione del punto selezionato dal dizionario: {ex}");
            }
        }

        #endregion
    }
}
