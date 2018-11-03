using System.Web.Mvc;

namespace VehicleRouting.Controllers
{
    public class AppController : Controller
    {
        public ActionResult Index()
        {
            this.ViewBag.Title = "Application";

            return this.View("~/Views/App/App.cshtml"); // TODO: Can we add this without explicitly specifying the path?
        }
    }
}