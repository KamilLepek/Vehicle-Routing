using System.Data.Entity;
using System.Linq;
using System.Net;
using System.Web.Mvc;
using VehicleRouting.Models;

namespace VehicleRouting.Controllers
{
    public class VehiclesController : Controller
    {
        private VehicleDbContext db = new VehicleDbContext();

        // GET: Vehicles
        public ActionResult Index()
        {
            return this.View(this.db.Vehicles.ToList());
        }

        // GET: Vehicles/Details/5
        public ActionResult Details(int? id)
        {
            if (id == null)
            {
                return new HttpStatusCodeResult(HttpStatusCode.BadRequest);
            }
            Vehicle vehicle = this.db.Vehicles.Find(id);
            if (vehicle == null)
            {
                return this.HttpNotFound();
            }
            return this.View(vehicle);
        }

        // GET: Vehicles/Create
        public ActionResult Create()
        {
            return this.View();
        }

        // POST: Vehicles/Create
        // To protect from overposting attacks, please enable the specific properties you want to bind to, for 
        // more details see http://go.microsoft.com/fwlink/?LinkId=317598.
        [HttpPost]
        [ValidateAntiForgeryToken]
        public ActionResult Create([Bind(Include = "ID,Name,Capacity,SpawnPointX,SpawnPointY")] Vehicle vehicle)
        {
            if (this.ModelState.IsValid)
            {
                this.db.Vehicles.Add(vehicle);
                this.db.SaveChanges();
                return this.RedirectToAction("Index");
            }

            return this.View(vehicle);
        }

        // GET: Vehicles/Edit/5
        public ActionResult Edit(int? id)
        {
            if (id == null)
            {
                return new HttpStatusCodeResult(HttpStatusCode.BadRequest);
            }
            Vehicle vehicle = this.db.Vehicles.Find(id);
            if (vehicle == null)
            {
                return this.HttpNotFound();
            }
            return this.View(vehicle);
        }

        // POST: Vehicles/Edit/5
        // To protect from overposting attacks, please enable the specific properties you want to bind to, for 
        // more details see http://go.microsoft.com/fwlink/?LinkId=317598.
        [HttpPost]
        [ValidateAntiForgeryToken]
        public ActionResult Edit([Bind(Include = "ID,Name,Capacity,SpawnPointX,SpawnPointY")] Vehicle vehicle)
        {
            if (this.ModelState.IsValid)
            {
                this.db.Entry(vehicle).State = EntityState.Modified;
                this.db.SaveChanges();
                return this.RedirectToAction("Index");
            }
            return this.View(vehicle);
        }

        // GET: Vehicles/Delete/5
        public ActionResult Delete(int? id)
        {
            if (id == null)
            {
                return new HttpStatusCodeResult(HttpStatusCode.BadRequest);
            }
            Vehicle vehicle = this.db.Vehicles.Find(id);
            if (vehicle == null)
            {
                return this.HttpNotFound();
            }
            return this.View(vehicle);
        }

        // POST: Vehicles/Delete/5
        [HttpPost, ActionName("Delete")]
        [ValidateAntiForgeryToken]
        public ActionResult DeleteConfirmed(int id)
        {
            Vehicle vehicle = this.db.Vehicles.Find(id);
            this.db.Vehicles.Remove(vehicle);
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
