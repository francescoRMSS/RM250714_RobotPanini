using System.Threading.Tasks;
using System.Threading;
using System;

namespace RM.src.RM250728.Classes.FR20
{
    /// <summary>
    /// Contiene le informazioni di un task gestito
    /// </summary>
    public class TaskModel
    {
        /// <summary>
        /// Nome del task
        /// </summary>
        public string Name { get; set; }
        /// <summary>
        /// //Riferimento al task creato
        /// </summary>
        public Task Task { get; set; }
        /// <summary>
        /// Riferimento al metodo da eseguire
        /// </summary>
        public Func<CancellationToken, Task> Method { get; set; }
        /// <summary>
        /// Riferimento al token di cancellazione
        /// </summary>
        public CancellationTokenSource Cts { get; set; }
        /// <summary>
        /// Indica allo scheduler come comportarsi con il task
        /// </summary>
        public TaskType Options { get; set; }
        /// <summary>
        /// Indica se il task deve essere mantenuto in lista alla fine della sua esecuzione
        /// </summary>
        public bool Retain { get; set; }

        /// <summary>
        /// Costruttore del task
        /// </summary>
        /// <param name="name"></param>
        /// <param name="task"></param>
        /// <param name="method"></param>
        /// <param name="cts"></param>
        /// <param name="options"></param>
        /// <param name="retain"></param>
        public TaskModel(string name, Task task, Func<CancellationToken, Task> method,
            CancellationTokenSource cts, TaskType options, bool retain)
        {
            Name = name;
            Task = task;
            Method = method;
            Cts = cts;
            Options = options;
            Retain = retain;
        }
    }
}
