using System.Web.Mvc;

namespace VehicleRouting.Controllers
{
    public class SolverController : Controller
    {
        public ActionResult Index()
        {
            this.ViewBag.Title = "Application";

            return this.View();
        }
    }
}