using RMLib.Logger;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

namespace RM.src.RM250728.Classes.FR20
{
    /// <summary>
    /// Definisce una struttura di supporto che gestisce i task, li mette in lista, li fa partire, fermare e
    /// controlla continuamente il loro stato notificando con un evento qualora uno degli stati in memoria cambiasse di stato.
    /// I task devono accettare un parametro CancellationToken che serve per far partire la procedura di terminazione del task.
    /// Si usa CancellationToken invece di un bool per via della sua natura thread safe.
    /// <br></br>
    /// Per natura, ogni task è considerato background e si può impostare il campo isLongRunning per indicare allo scheduler che il task
    /// potrebbe non durare poco nell'esecuzione, perciò potrebbe decidere di usare un nuovo task invece di usare quelli del pool standard.
    /// E' comunque preferibile utilizzare i task del pool di .net quando i task, sebbene possano durare molto, non sono cpu-bound, perciò quando
    /// un task è quasi sempre in riposo (con task.delay) è meglio usare un task TPL.
    /// <br></br> 
    /// Vantaggi: gestione centralizzata con livello di astrazione superiore. Utilizzo del thread pool fornito da .NET. Quando un task
    /// viene messo in pausa da Task.delay il thread non viene bloccato ma può eseguire altro codice.
    /// <br></br>
    /// Update futuri:
    /// <br>-implementare thread e thread STA</br>
    /// </summary>
    public class TaskManager
    {
        /// <summary>
        /// Logger
        /// </summary>
        private static readonly log4net.ILog log = LogHelper.GetLogger();

        /// <summary>
        /// Task interno che verifica continuamente lo stato dei task gestiti
        /// </summary>
        private Task _checkTask;
        /// <summary>
        /// Token source per controllo terminazione task di check
        /// </summary>
        CancellationTokenSource _checkTasksCTS = null;      
        /// <summary>
        /// Token per controllo terminazione task di check
        /// </summary>
        CancellationToken _checkTasksToken;
        /// <summary>
        /// Delay tra un ciclo di check e l'altro
        /// </summary>
        private const int _checkTaskDelay = 100;            

        /// <summary>
        /// Notifica gli ascoltatori che uno o più task ha cambiato stato
        /// </summary>
        public event EventHandler OneTaskChangedStatus;
        /// <summary>
        /// Lista di strutture contenenti le informazioni di un task gestito
        /// </summary>
        private readonly List<TaskModel> _tasks; 
        /// <summary>
        /// Oggetto su cui applicare lock, deve essere usato per leggere in sicurezza la lista
        /// </summary>
        private readonly object _tasksLock;

        /// <summary>
        /// Costruisce il task manager e inizializza la sua lista di task gestiti
        /// </summary>
        public TaskManager()
        {
            _tasks = new List<TaskModel>();
            _tasksLock = new object();
        }

        /// <summary>
        /// Fa partire il task che continuamente controlla lo stato dei task gestiti
        /// </summary>
        public void StartTaskChecker()
        {
            if(_checkTasksCTS == null)
            {
                log.Warn("[TASK] Richiesto start task CheckTasks");
                _checkTasksCTS = new CancellationTokenSource();
                _checkTasksToken = _checkTasksCTS.Token;
                _checkTask = Task.Run(() => CheckTasks(_checkTasksToken), _checkTasksToken);
            }
        }

        /// <summary>
        /// Ferma il task che controlla lo stato dei task gestiti
        /// </summary>
        public void StopTaskChecker()
        {
            if (_checkTasksCTS != null)
            {
                log.Warn("[TASK] Richiesto stop task CheckTasks");
                _checkTasksCTS.Cancel();
            }
        }

        /// <summary>
        /// Helper che ricava l'indice del task a partire dal suo nome
        /// </summary>
        /// <param name="name"></param>
        /// <returns></returns>
        private int GetTaskId(string name)
        {
            int id = -1;

            for(int i = 0; i < _tasks.Count; i++) //Scorro la lista di task gestiti
            {
                if (string.Equals(_tasks[i].Name, name)) //Per ogni task gestito controllo il nome
                {
                    id = i; //Se il nome combacia allora restituisco l'id numerico = indice
                    break;
                }
            }

            return id;
        }

        /// <summary>
        /// Aggiunge un task alla lista dei task gestiti
        /// </summary>
        /// <param name="name"></param>
        /// <param name="method"></param>
        /// <param name="type"></param>
        /// <param name="retain"></param>
        public void AddTask(string name, Func<CancellationToken, Task> method, TaskType type, bool retain = false)
        {
            InternalAddTask(name, method, type, retain);
        }

        /// <summary>
        /// Aggiunge un task alla lista e lo fa partire subito
        /// </summary>
        /// <param name="name"></param>
        /// <param name="method"></param>
        /// <param name="type"></param>
        /// <param name="retain"></param>
        public void AddAndStartTask(string name, Func<CancellationToken, Task> method, TaskType type, bool retain = false)
        {
            InternalAddTask(name, method, type, retain);
            InternalStartTask(name);
        }

        private void InternalAddTask(string name, Func<CancellationToken, Task> method, TaskType type, bool retain)
        {
            log.Info($"Richiesta Add task: {name}");
            //Controllo che il task non sia già presente
            foreach (TaskModel task in GetTaskList())
            {
                if (string.Equals(task.Name, name))
                {
                    log.Error("Tentativo di aggiunta task già presente in lista");
                    return;
                }
            }
            TaskModel newStruct = new TaskModel(name, null, method, null, type, retain);
            lock (_tasksLock)
            {
                _tasks.Add(newStruct);
            }
        }

        /// <summary>
        /// Fa partire un task a partire dal suo nome
        /// </summary>
        /// <param name="name"></param>
        public void StartTask(string name)
        {
            InternalStartTask(name);
        }

        private void InternalStartTask(string name)
        {
            log.Info($"Richiesta Start task: {name}");
            TaskModel managedTask;

            lock (_tasksLock)
            {
                managedTask = _tasks.FirstOrDefault(t => t.Name == name);
            }
           
            if (managedTask != null) //Se null allora il task non è stato trovato
            {
                // Controlla se è già in esecuzione
                if (managedTask.Task != null && !managedTask.Task.IsCompleted) return;

                //Creazione token source
                managedTask.Cts = new CancellationTokenSource();
                CancellationToken token = managedTask.Cts.Token;

                //Selezione impostazioni
                TaskCreationOptions options;
                switch (managedTask.Options)
                {
                    case TaskType.Default:
                        options = TaskCreationOptions.None;
                        break;
                    case TaskType.LongRunning:
                        options = TaskCreationOptions.LongRunning;
                        break;
                    case TaskType.Short:
                        options = TaskCreationOptions.None;
                        break;
                    default:
                        options = TaskCreationOptions.None;
                        break;
                }

                //Creazione Task, usiamo una lambda per passare il token al metodo eseguito
                if (managedTask.Options == TaskType.Short)
                {
                    managedTask.Task = Task.Run(() => managedTask.Method(token), token);
                }
                else
                {
                    //Con questo metodo sotto puoi usare long running
                    managedTask.Task = Task.Factory.StartNew(() => managedTask.Method(token),
                        token,
                        options,
                        TaskScheduler.Default
                        ).Unwrap();
                }
            }
            
        }

        /// <summary>
        /// Ferma l'esecuzione di un task a partire dal suo nome
        /// </summary>
        /// <param name="name"></param>
        public void StopTask(string name)
        {
            InternalStopTask(name);
        }

        private void InternalStopTask(string name)
        {
            log.Info($"Richiesta Stop task: {name}");
            lock (_tasksLock)
            {
                int taskId = GetTaskId(name);

                if (taskId != -1) //Se -1 allora il task non è stato trovato
                {
                    // Controllo che il task esista e che non sia già stata richiesta la sua terminazione
                    if (_tasks[taskId].Cts != null && !_tasks[taskId].Cts.IsCancellationRequested)
                    {
                        _tasks[taskId].Cts.Cancel();
                    }
                }
            }
        }

        /// <summary>
        /// Fa partire tutti i task gestiti
        /// </summary>
        public void StartAllTasks()
        {
            log.Info("Richiesto start di tutti i task");
            var snapshot = GetTaskList();
            foreach (var task in snapshot)
            {
                InternalStartTask(task.Name);
            }
        }

        /// <summary>
        /// Fa partire tutti i task di una specifica categoria
        /// </summary>
        /// <param name="type"></param>
        public void StartAllTaskByType(TaskType type)
        {
            log.Info($"Richiesto start di tutti i task della categoria {type}");
            var snapshot = GetTaskList();
            foreach (var task in snapshot)
            {
                if (task.Options == type)
                {
                    InternalStartTask(task.Name);
                }
            }
        }

        /// <summary>
        /// Fa partire la procedura di stop per ogni task gestito
        /// </summary>
        public void StopAllTasks()
        {
            log.Info("Richiesto stop di tutti i task");
            var snapshot = GetTaskList();
            foreach (var task in snapshot)
            {
                InternalStopTask(task.Name);
            }
        }

        /// <summary>
        /// Fa partire la procedura di stop per ogni task di una specifica categoria
        /// </summary>
        /// <param name="type"></param>
        public void StopAllTaskByType(TaskType type)
        {
            log.Info($"Richiesto stop di tutti i task della categoria {type}");
            var snapshot = GetTaskList();
            foreach (var task in snapshot)
            {
                if (task.Options == type)
                {
                    InternalStopTask(task.Name);
                }  
            }
        }

        //public async Task RunTemporizedTask()

        /// <summary>
        /// Metodo eseguito dal Task interno, controlla lo stato di ogni task gestito ad ogni ciclo. 
        /// Notifica chi interessato tramite un evento.
        /// </summary>
        private async Task CheckTasks(CancellationToken token)
        {
            try
            {
                if(_checkTask.Status == TaskStatus.Running ||
                   _checkTask.Status == TaskStatus.WaitingForActivation)
                {
                    log.Warn($"[TASK] CheckTasks - RUNNING");
                }

                // Un dizionario che funge da "memoria". La chiave è il nome univoco del task,
                // il valore è l'ultimo stato che abbiamo visto per quel task.
                var previousStatuses = new Dictionary<string, TaskStatus>();
                List<TaskModel> taskToRemove = new List<TaskModel>();
                bool anyStatusChanged = false;

                while (!token.IsCancellationRequested)
                {
                    List<TaskModel> tasksToCheck = GetTaskList();

                    foreach (TaskModel currentTask in tasksToCheck)
                    {
                        // Se il task non è ancora stato avviato, non ha uno stato. Passiamo al prossimo.
                        if (currentTask.Task == null) continue;

                        TaskStatus currentStatus = currentTask.Task.Status;

                        // La logica di controllo:
                        // 1. Il task è nuovo e non è nella nostra "memoria"? OPPURE
                        // 2. Il task era già noto, ma il suo stato attuale è DIVERSO da quello che ricordavamo
                        if (!previousStatuses.ContainsKey(currentTask.Name) || previousStatuses[currentTask.Name] != currentStatus)
                        {
                           
                            //Filtro per evitare troppi eventi
                            if (currentStatus == TaskStatus.Canceled ||
                               currentStatus == TaskStatus.Faulted ||
                               currentStatus == TaskStatus.Running ||
                               currentStatus == TaskStatus.RanToCompletion ||
                               currentStatus == TaskStatus.WaitingForActivation
                               )
                            {
                                anyStatusChanged = true;
                            }

                            // Aggiorno la "memoria" con il nuovo stato
                            previousStatuses[currentTask.Name] = currentStatus;

                            switch (currentStatus)
                            {
                                case TaskStatus.Running:
                                case TaskStatus.WaitingForActivation:
                                    log.Warn($"[TASK] {currentTask.Name} - RUNNING");
                                    break;
                                case TaskStatus.Canceled:
                                    if (!currentTask.Retain) //Se non retain allora rimuoviamo il task terminato
                                    {
                                        taskToRemove.Add(currentTask);
                                        log.Warn($"[TASK] {currentTask.Name} - CANCELLATO - verra' rimosso dalla lista");
                                    }
                                    else
                                    {
                                        log.Warn($"[TASK] {currentTask.Name} - CANCELLATO");
                                    }
                                    break;
                                case TaskStatus.RanToCompletion:
                                    if (!currentTask.Retain) //Se non retain allora rimuoviamo il task terminato
                                    {
                                        taskToRemove.Add(currentTask);
                                        log.Warn($"[TASK] {currentTask.Name} - TERMINATO - verra' rimosso dalla lista");
                                    }
                                    else
                                    {
                                        log.Warn($"[TASK] {currentTask.Name} - TERMINATO");
                                    }
                                    break;
                                case TaskStatus.Faulted:
                                    if (!currentTask.Retain) //Se non retain allora rimuoviamo il task terminato
                                    {
                                        taskToRemove.Add(currentTask);
                                        log.Warn($"[TASK] {currentTask.Name} - ERRORE - verra' rimosso dalla lista");
                                    }
                                    else
                                    {
                                        log.Error($"[TASK] {currentTask.Name} - ERRORE");
                                    }
                                    break;
                            }
                        }
                    }
                    // Rimozione dei task non retain
                    if (taskToRemove.Any())
                    {
                        lock (_tasksLock)
                        {
                            // Rimuovi in una sola operazione tutti gli elementi di '_tasks'
                            // che sono presenti nella lista 'taskToRemove'.
                            _tasks.RemoveAll(task => taskToRemove.Contains(task));
                        }
                    }
                    taskToRemove.Clear();

                    if(anyStatusChanged)
                    {
                        // Lanciamo l'evento per notificare chiunque sia in ascolto (es. l'interfaccia utente)
                        try
                        {
                            OneTaskChangedStatus?.Invoke(this, EventArgs.Empty);
                        }
                        catch (Exception ex)
                        {
                            log.Error($"[TASK] CheckTasks - Eccezione generata durante invio evento {nameof(OneTaskChangedStatus)} : {ex}");
                        }
                    }
                    anyStatusChanged = false;

                    // Aspetta prima del prossimo controllo per non consumare la CPU.
                    await Task.Delay(_checkTaskDelay);
                }
                token.ThrowIfCancellationRequested();
            }
            catch (OperationCanceledException)
            {
                log.Warn("[TASK] CheckTasks - TERMINATO");
            }
            catch (Exception ex)
            {
                log.Error("[TASK] CheckTasks - ERRORE : " + ex);
            }
            finally
            {
                _checkTasksCTS = null;
            }
        }

        /// <summary>
        /// Fornisce una copie della lista di task così che chi interessato può vedere lo stato dei task
        /// </summary>
        /// <returns></returns>
        public List<TaskModel> GetTaskList()
        {
            lock (_tasksLock)
            {
                return new List<TaskModel>(_tasks);
            }
        }
    }
}
