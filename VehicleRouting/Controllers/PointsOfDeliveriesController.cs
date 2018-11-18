using System.Data.Entity;
using System.Linq;
using System.Net;
using System.Web.Mvc;
using VehicleRouting.Models;

namespace VehicleRouting.Controllers
{
    public class PointsOfDeliveriesController : Controller
    {
        private VehicleDbContext db = new VehicleDbContext();

        // GET: PointsOfDeliveries
        public ActionResult Index()
        {
            return this.View(this.db.PointsOfDeliveries.ToList());
        }

        // GET: PointsOfDeliveries/Details/5
        public ActionResult Details(int? id)
        {
            if (id == null)
            {
                return new HttpStatusCodeResult(HttpStatusCode.BadRequest);
            }
            PointOfDelivery pointOfDelivery = this.db.PointsOfDeliveries.Find(id);
            if (pointOfDelivery == null)
            {
                return this.HttpNotFound();
            }
            return this.View(pointOfDelivery);
        }

        // GET: PointsOfDeliveries/Create
        public ActionResult Create()
        {
            return this.View();
        }

        // POST: PointsOfDeliveries/Create
        // To protect from overposting attacks, please enable the specific properties you want to bind to, for 
        // more details see http://go.microsoft.com/fwlink/?LinkId=317598.
        [HttpPost]
        [ValidateAntiForgeryToken]
        public ActionResult Create([Bind(Include = "ID,Name,CoordX,CoordY")] PointOfDelivery pointOfDelivery)
        {
            if (this.ModelState.IsValid)
            {
                this.db.PointsOfDeliveries.Add(pointOfDelivery);
                this.db.SaveChanges();
                return this.RedirectToAction("Index");
            }

            return this.View(pointOfDelivery);
        }

        // GET: PointsOfDeliveries/Edit/5
        public ActionResult Edit(int? id)
        {
            if (id == null)
            {
                return new HttpStatusCodeResult(HttpStatusCode.BadRequest);
            }
            PointOfDelivery pointOfDelivery = this.db.PointsOfDeliveries.Find(id);
            if (pointOfDelivery == null)
            {
                return this.HttpNotFound();
            }
            return this.View(pointOfDelivery);
        }

        // POST: PointsOfDeliveries/Edit/5
        // To protect from overposting attacks, please enable the specific properties you want to bind to, for 
        // more details see http://go.microsoft.com/fwlink/?LinkId=317598.
        [HttpPost]
        [ValidateAntiForgeryToken]
        public ActionResult Edit([Bind(Include = "ID,Name,CoordX,CoordY")] PointOfDelivery pointOfDelivery)
        {
            if (this.ModelState.IsValid)
            {
                this.db.Entry(pointOfDelivery).State = EntityState.Modified;
                this.db.SaveChanges();
                return this.RedirectToAction("Index");
            }
            return this.View(pointOfDelivery);
        }

        // GET: PointsOfDeliveries/Delete/5
        public ActionResult Delete(int? id)
        {
            if (id == null)
            {
                return new HttpStatusCodeResult(HttpStatusCode.BadRequest);
            }
            PointOfDelivery pointOfDelivery = this.db.PointsOfDeliveries.Find(id);
            if (pointOfDelivery == null)
            {
                return this.HttpNotFound();
            }
            return this.View(pointOfDelivery);
        }

        // POST: PointsOfDeliveries/Delete/5
        [HttpPost, ActionName("Delete")]
        [ValidateAntiForgeryToken]
        public ActionResult DeleteConfirmed(int id)
        {
            PointOfDelivery pointOfDelivery = this.db.PointsOfDeliveries.Find(id);
            this.db.PointsOfDeliveries.Remove(pointOfDelivery);
            this.db.SaveChanges();
            return this.RedirectToAction("Index");
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                this.db.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
