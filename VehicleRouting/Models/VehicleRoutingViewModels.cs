using System;
using System.Collections.Generic;

namespace VehicleRouting.Models
{
    public class ProductPackInputModel
    {
        public int ProductID { get; set; }
        public int PointOfDeliveryID { get; set; }
        public int Amount { get; set; }
    }

    public class LocationsViewModel
    {
        public IEnumerable<PointOfDelivery> PointsOfDelivery { get; set; }
        public IEnumerable<Vehicle> Vehicles { get; set; }
    }

    public class SolverViewModel
    {
        public LocationsViewModel LocationsViewModel { get; set; }
        public List<ProductPack> ProductPacks { get; set; }
        public List<int> VehiclesIDs { get; set; }
    }

    public class SolverReturnViewModel
    {
        public int MetricType { get; set; }
        public List<int> VehiclesIDs { get; set; }
        public List<int> ProductPacks { get; set; }
    }

    public class SolverResultViewModel
    {
        public LocationsViewModel LocationsViewModel { get; set; }

        /// <summary>
        ///     Dictionary of keys - vehicle ids, values - sorted list of points they have to reach
        /// </summary>
        public Dictionary<int, List<ValueTuple<float, float>>> AlgorithmResult { get; set; }

        /// <summary>
        ///     Dictionary of keys - vehicle ids, values - time of algorithm execution and total distance calculated
        /// </summary>
        public Dictionary<int, ValueTuple<float,float>> TimeAndDistance { get; set; }

        /// <summary>
        ///     Dictionary of keys - vehicle ids, values - exceeded capacity
        /// </summary>
        public Dictionary<int, ValueTuple<float, int>> VehiclesExceededCapacity { get; set; }
    }
}