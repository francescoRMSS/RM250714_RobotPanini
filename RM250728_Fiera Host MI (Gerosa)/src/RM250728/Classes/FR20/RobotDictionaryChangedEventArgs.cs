using RMLib.Recipes;
using System.Windows.Forms;

namespace RM.src.RM250728
{
    /// <summary>
    /// Definisce tipologia evento
    /// </summary>
    public class RobotDictionaryChangedEventArgs
    {
        #region Proprietà di RobotDictionaryChangedEventArgs
        /// <summary>
        /// Riferimento alla ricetta
        /// </summary>
        public Recipe Recipe { get; }

        /// <summary>
        /// Chiave univoca
        /// </summary>
        public string Key { get; }

        /// <summary>
        /// Nuovo valore
        /// </summary>
        public object NewValue { get; }

        /// <summary>
        /// Tipo di ricetta
        /// </summary>
        public string Type { get; }

        /// <summary>
        /// Item della lista
        /// </summary>
        public ListViewItem RecipeItem { get; }

        /// <summary>
        /// Nome componente
        /// </summary>
        public string Component { get; }

        /// <summary>
        /// Data con formattazione
        /// </summary>
        public string FormattedDate { get; }

        /// <summary>
        /// Componente
        /// </summary>
        public RecipeComponents RecipeComponent { get; }

        /// <summary>
        /// Indice del componente
        /// </summary>
        public int IndexRecipeComponent { get; }

        /// <summary>
        /// Timestamp numerico
        /// </summary>
        public long UnixTimestamp { get; }

        /// <summary>
        /// ID
        /// </summary>
        public int Id { get; }

        /// <summary>
        /// Valore in stringa
        /// </summary>
        public string StringValue { get; }

        /// <summary>
        /// Applicazione usata
        /// </summary>
        public RobotApplication Application { get; }
        #endregion

        /// <summary>
        /// Costruttore utilizzato per aggiornare valore di un AppVariable
        /// </summary>
        /// <param name="key">Chiave</param>
        /// <param name="newValue">Nuovo valore</param>
        /// <param name="type">Tipologia dato</param>
        public RobotDictionaryChangedEventArgs(string key, object newValue, string type)
        {
            Key = key;
            NewValue = newValue;
            Type = type;
        }

        public RobotDictionaryChangedEventArgs(string key, RobotApplication application)
        {
            Key = key;
            Application = application;
        }

        /// <summary>
        /// Costruttore utilizzato per riempire di appVariable vuote il dizionario
        /// </summary>
        /// <param name="key">Chiave</param>
        /// <param name="newValue">Nuovo valore</param>
        public RobotDictionaryChangedEventArgs(string key, object newValue)
        {
            Key = key;
            NewValue = newValue;
        }

        /// <summary>
        /// Costruttore utilizzato per la modifica di una nota di una ricetta esistente
        /// </summary>
        /// <param name="value">Valore</param>
        /// <param name="formattedDate">Data</param>
        public RobotDictionaryChangedEventArgs(string value, string formattedDate)
        {
            StringValue = value;
            FormattedDate = formattedDate;
        }

        /// <summary>
        /// Costruttore utilizzato per leggere la ricetta dal PLC ed aggiornare il database
        /// </summary>
        /// <param name="recipe">ricetta</param>
        /// <param name="component">componente della ricetta</param>

        public RobotDictionaryChangedEventArgs(ListViewItem recipe, string component)
        {
            RecipeItem = recipe;
            Component = component;
        }

        /// <summary>
        /// Costruttore utilizzato per aggiornare LasUpdateTime della ricetta dopo che
        /// i valori sono stati importati dal PLC
        /// </summary>
        /// <param name="formattedDate">nuova data LastUpdateTime</param>
        public RobotDictionaryChangedEventArgs(string formattedDate)
        {
            FormattedDate = formattedDate;
            Key = formattedDate;
        }

        /// <summary>
        /// Costruttore utilizzato per la modifica di un id ricetta esistente
        /// </summary>
        /// <param name="id">Id ricetta</param>
        /// <param name="formattedDate">Data</param>
        public RobotDictionaryChangedEventArgs(int id, string formattedDate)
        {
            Id = id;
            FormattedDate = formattedDate;
        }

        /// <summary>
        /// Costruttore utilizzato per aggiunta di una nuova ricetta al dizionario
        /// </summary>
        /// <param name="key">recipeName</param>
        /// <param name="recipe">ricetta con tutti i suoi campi</param>
        public RobotDictionaryChangedEventArgs(string key, Recipe recipe)
        {
            this.Key = key;
            this.Recipe = recipe;
        }

        /// <summary>
        /// Costruttore utilizzato per leggere la ricetta dal PLC ed aggiornare il database
        /// </summary>
        /// <param name="recipeComponent"></param>
        /// <param name="indexRecipeComponent"></param>
        public RobotDictionaryChangedEventArgs(RecipeComponents recipeComponent, int indexRecipeComponent)
        {
            this.RecipeComponent = recipeComponent;
            this.IndexRecipeComponent = indexRecipeComponent;
        }

        /// <summary>
        /// Costruttore utilizzato per leggere una nuova ricetta dal PLC e aggiornare il database 
        /// </summary>
        /// <param name="component"></param>
        /// <param name="indexRecipeComponent"></param>
        /// <param name="unixTimestamp"></param>
        public RobotDictionaryChangedEventArgs(RecipeComponents component, int indexRecipeComponent, long unixTimestamp)
        {
            this.RecipeComponent = component;
            this.IndexRecipeComponent = indexRecipeComponent;
            this.UnixTimestamp = unixTimestamp;
        }

        /// <summary>
        /// Costruttore usato per refreshare gli id dopo una cancellazione
        /// </summary>
        /// <param name="pointId"></param>
        public RobotDictionaryChangedEventArgs(int pointId)
        {
            Id = pointId;
        }

        /// <summary>
        /// Costruttore vuoto
        /// </summary>
        public RobotDictionaryChangedEventArgs()
        {
        }
    }
}
