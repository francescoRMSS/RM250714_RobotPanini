namespace RM.src.RM250728.Classes.FR20
{
    /// <summary>
    /// I diversi tipi di esecuzione di un task
    /// </summary>
    public enum TaskType
    {
        /// <summary>
        /// Task che girano sempre o eseguono operazioni pesanti cpu bound
        /// </summary>
        LongRunning,
        /// <summary>
        /// Task che eseguono operazioni e vanno in riposo
        /// </summary>
        Default,
        /// <summary>
        /// Task che eseguono uno o più cicli per poi fermarsi
        /// </summary>
        Short
    }
}
