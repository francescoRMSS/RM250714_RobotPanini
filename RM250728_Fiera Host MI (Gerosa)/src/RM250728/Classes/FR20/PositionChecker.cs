using fairino;
using System;

namespace RM.src.RM250728
{
    /// <summary>
    /// Classe necessario per individuare zone di ingombro, zone di sicurezza e in position
    /// </summary>
    public class PositionChecker
    {
        #region Parametri di PositionChecker

        /// <summary>
        /// Soglia da utilizzare
        /// </summary>
        private double delta;

        #endregion

        /// <summary>
        /// Costruttore con soglia da utilizzare
        /// </summary>
        /// <param name="delta">Soglia</param>
        public PositionChecker(double delta)
        {
            this.delta = delta;
        }

        #region Metodi di PositionChecker

        /// <summary>
        /// Controlla se il punto si trova nella posizione passata come parametro
        /// </summary>
        /// <param name="endingPoint">Posizione che si vuole controllare</param>
        /// <param name="currentPoint">Posizione corrente</param>
        /// <returns></returns>
        public bool IsInPosition(DescPose endingPoint, DescPose currentPoint)
        {
            double closestDistance = double.MaxValue;

            double distance = CalculateDistance(endingPoint, currentPoint);

            if (distance <= delta && distance < closestDistance)
            {
                return true;
            }

            return false;
        }

        /// <summary>
        /// Controlla se la Y del punto corrente è minore della Y del punto finale
        /// </summary>
        /// <param name="endingPoint">Punto passato come parametro</param>
        /// <param name="currentPoint">Posizione attuale del robot</param>
        /// <returns></returns>
        public bool IsYLessThan(DescPose endingPoint, DescPose currentPoint)
        {

            if (currentPoint.tran.y < endingPoint.tran.y + delta)
            {
                return true;
            }
            return false;
        }

        /// <summary>
        /// Controlla se il robot si trova in un'area passando un punto come parametro
        /// </summary>
        /// <param name="endingPoint">Punto da cui parte l'area</param>
        /// <param name="currentPoint">Poszione corrente</param>
        /// <returns></returns>
        public bool IsInObstruction(DescPose endingPoint, DescPose currentPoint)
        {
            double dx = Math.Abs(endingPoint.tran.x - currentPoint.tran.x);
            double dy = Math.Abs(endingPoint.tran.y - currentPoint.tran.y);
            double dz = Math.Abs(endingPoint.tran.z - currentPoint.tran.z);

            if (dx <= delta && dy <= delta && dz <= delta)
            {
                return true;
            }

            return false;
        }

        /// <summary>
        /// Calcolo della distanza tra due punti
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <returns></returns>
        private double CalculateDistance(DescPose p1, DescPose p2)
        {
            double dx = p1.tran.x - p2.tran.x;
            double dy = p1.tran.y - p2.tran.y;
            double dz = p1.tran.z - p2.tran.z;
            double drx = p1.rpy.rx - p2.rpy.rx;
            double dry = p1.rpy.ry - p2.rpy.ry;
            double drz = p1.rpy.rz - p2.rpy.rz;

            return Math.Sqrt(dx * dx + dy * dy + dz * dz + drx * drx + dry * dry + drz * drz);
        }

        #endregion
    }
}
